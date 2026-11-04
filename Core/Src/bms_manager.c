#include "bms_manager.h"

#include "app.h"
#include "bms_board.h"
#include "bms_store.h"
#include "bms_lowbatt_policy.h"
#include "bq76922.h"
#include "ldo_link.h"
#include "ldo_prereg.h"
#include "power_stage.h"
#include "psu_gui_api.h"

#include <limits.h>
#include <string.h>

/*
 * Debug mailbox for ST-Link reads (CubeProgrammer memory view, 32-bit words).
 * Address: see build map (symbol g_bms_dbg). Word layout:
 *  0 magic 0xB5F01F95   1 c1_mv   2 c2_mv  3 c3_mv  4 c4_mv  5 c5_mv
 *  6 ts1 0.1K   7 die 0.1K   8 cc2 mA (signed)   9 fet|sa<<8|sb<<16|sc<<24
 * 10 golden_ok|otp<<1|cfg<<2|present<<3|init_step<<8|fp_fail_addr<<16
 * 11 soc permille   12 fcc mAh   13 gauge flags   14 store seq   15 balance mask
 * 16 stack mV  17 pack mV  18 passQ mAh  19 cfg_fail  20 sample tick ms
 */
volatile uint32_t g_bms_dbg[24];

static bool s_loaded;
static bool s_loaded_clean;
static bool s_last_save_ok = true;
static uint32_t s_last_sample_ms;
static uint32_t s_last_save_ms;
static bool s_pack_seen;
static BmsLowBatt_Config_t s_lb_cfg;
static BmsLowBatt_State_t s_lb;

static int16_t dk_to_dc(int16_t dk)
{
    /* 0.1 K → 0.1 °C; 0 means "not read yet". */
    if (dk == 0) {
        return INT16_MIN;
    }
    return (int16_t)(dk - 2732);
}

static bool output_idle(void)
{
    return !App_IsStageEnabled() && !PowerStage_IsEnabled() &&
           (App_GetRequestedMode() == MODE_IDLE);
}

void BmsMgr_Init(void)
{
    BmsGauge_Config_t cfg;
    BmsStore_Record_t rec;

    BmsLowBatt_DefaultConfig(&s_lb_cfg);
    s_lb_cfg.warn_mv = (int16_t)BMS_WARN_CELL_LOW_MV;
    s_lb_cfg.cutoff_mv = (int16_t)BMS_CUTOFF_CELL_MV;
    BmsGauge_DefaultConfig(&cfg);
    cfg.design_mah = BMS_DESIGN_CAPACITY_MAH;
    cfg.series = (uint8_t)BMS_SERIES_COUNT;
    cfg.taper_ma = BMS_DESIGN_CAPACITY_MAH / 20;
    cfg.full_cell_mv = (int16_t)(BMS_CHARGE_TARGET_CELL_MV - 50U);
    BmsGauge_Init(&cfg);

    s_loaded = BmsStore_Load(&rec);
    if (s_loaded) {
        s_loaded_clean = rec.clean;
        BmsGauge_Restore(&rec.gauge, rec.clean);
    }
}

static bool do_save(bool clean)
{
    BmsGauge_Persist_t p;

    BmsGauge_GetPersist(&p);
    s_last_save_ok = BmsStore_Save(&p, clean);
    s_last_save_ms = HAL_GetTick();
    return s_last_save_ok;
}

void BmsMgr_Task(uint32_t now_ms)
{
    BQ76922_Snapshot_t s;

    BQ76922_GetSnapshot(&g_bq76922, &s);
    if (!s.present) {
        if (s_pack_seen) {
            BmsGauge_NotePackLost();
            BmsLowBatt_PackLost(&s_lb);
            s_pack_seen = false;
        }
        return;
    }
    if (s.configured && s.sample_valid && (s.sample_tick_ms != s_last_sample_ms)) {
        BmsGauge_Input_t in;

        s_last_sample_ms = s.sample_tick_ms;
        s_pack_seen = true;
        memset(&in, 0, sizeof(in));
        in.now_ms = s.sample_tick_ms;
        in.afe_valid = true;
        in.passq_valid = s.passq_valid;
        in.passq_whole = s.passq_whole;
        in.passq_frac = s.passq_frac;
        in.passq_time_s = s.passq_time_s;
        in.current_ma = s.cc2_ma;
        in.cell_mv = s.cell_mv;
        in.temp_dc = dk_to_dc(s.ts1_temp_dk);
        BmsGauge_Update(&in);
        if (BmsLowBatt_Step(&s_lb_cfg, &s_lb, s.sample_tick_ms,
                            s.min_cell_mv, s.cc2_ma)) {
            bool saved;

            (void)BmsMgr_OrderlyShutdown(&saved);
            return;
        }
    }

    {
        BmsGauge_Output_t go;
        uint8_t i;

        BmsGauge_GetOutput(&go);
        g_bms_dbg[0] = 0xB5F01F95UL;
        for (i = 0U; i < 5U; i++) {
            g_bms_dbg[1U + i] = (uint32_t)(int32_t)s.cell_mv[i];
        }
        g_bms_dbg[6] = (uint32_t)(int32_t)s.ts1_temp_dk;
        g_bms_dbg[7] = (uint32_t)(int32_t)s.int_temp_dk;
        g_bms_dbg[8] = (uint32_t)(int32_t)s.cc2_ma;
        g_bms_dbg[9] = (uint32_t)s.fet_status | ((uint32_t)s.safety_status_a << 8) |
                       ((uint32_t)s.safety_status_b << 16) | ((uint32_t)s.safety_status_c << 24);
        g_bms_dbg[10] = (s.golden_ok ? 1U : 0U) | (s.golden_otp ? 2U : 0U) |
                        (s.configured ? 4U : 0U) | (s.present ? 8U : 0U) |
                        ((uint32_t)s.init_step << 8) | ((uint32_t)s.fp_fail_addr << 16);
        g_bms_dbg[11] = go.soc_permille;
        g_bms_dbg[12] = (uint32_t)go.fcc_mah;
        g_bms_dbg[13] = go.flags;
        g_bms_dbg[14] = BmsStore_LastSeq();
        g_bms_dbg[15] = s.balance_mask;
        g_bms_dbg[16] = (uint32_t)(int32_t)s.stack_mv;
        g_bms_dbg[17] = (uint32_t)(int32_t)s.pack_mv;
        g_bms_dbg[18] = (uint32_t)s.passq_mah;
        g_bms_dbg[19] = s.cfg_fail_count;
        g_bms_dbg[20] = s.sample_tick_ms;
    }

    if (!output_idle()) {
        return;
    }
    {
        uint32_t since = now_ms - s_last_save_ms;
        bool dirty = BmsGauge_IsDirty();
        int32_t unsaved = BmsGauge_UnsavedMah();

        if ((dirty && (since >= BMS_MGR_EVENT_SAVE_MS)) ||
            ((unsaved >= BMS_MGR_MIN_DELTA_MAH) && (since >= BMS_MGR_IDLE_SAVE_MS))) {
            (void)do_save(false);
        }
    }
}

bool BmsMgr_SaveForShutdown(void)
{
    return do_save(true);
}

BQ76922_Status_t BmsMgr_OrderlyShutdown(bool *saved)
{
    bool ok;

    /* Kill the PSU path first so the PACK load does not fight FET-off. */
    LdoLink_RequestOutput(false);
    LdoPrereg_SetForceDisable(true);
    LdoPrereg_SetPermitOverrideOff(true);
    PSU_Stop();
    /* Converter is stopped: safe to program flash.  Clean record = the pack
     * rests from now on. */
    ok = do_save(true);
    if (saved != NULL) {
        *saved = ok;
    }
    /* Let the UART flush before the AFE drops. */
    HAL_Delay(20U);
    return BQ76922_EnterShutdown(&g_bq76922);
}

uint32_t BmsMgr_LowBattFlags(void)
{
    return BmsLowBatt_Flags(&s_lb);
}

bool BmsMgr_SaveNow(bool *refused_output_on)
{
    if (!output_idle()) {
        if (refused_output_on != NULL) {
            *refused_output_on = true;
        }
        return false;
    }
    if (refused_output_on != NULL) {
        *refused_output_on = false;
    }
    return do_save(false);
}

void BmsMgr_ResetPack(void)
{
    BmsGauge_ResetPack();
}

void BmsMgr_GetGauge(BmsGauge_Output_t *out)
{
    BmsGauge_GetOutput(out);
}

bool BmsMgr_StoreLoaded(void)
{
    return s_loaded;
}

uint32_t BmsMgr_StoreSeq(void)
{
    return BmsStore_LastSeq();
}

bool BmsMgr_LastSaveOk(void)
{
    return s_last_save_ok;
}

void BmsMgr_FillBms(TlmBuilder_t *b)
{
    BQ76922_Snapshot_t s;
    BmsGauge_Output_t g;
    uint8_t i;

    BQ76922_GetSnapshot(&g_bq76922, &s);
    BmsGauge_GetOutput(&g);
    Tlm_Begin(b, TLM_GROUP_BMS);
    Tlm_Put(b, TLM_BMS_PRESENT, (BQ76922_IsEnabled() && s.present) ? 1 : 0);
    Tlm_Put(b, TLM_BMS_CFG, (BQ76922_IsEnabled() && s.configured) ? 1 : 0);
    Tlm_Put(b, TLM_BMS_STATE, BQ76922_IsEnabled() ? (int32_t)s.state : 0);
    Tlm_Put(b, TLM_BMS_ALERT, (s.alert_latched || s.alert_pin) ? 1 : 0);
    Tlm_Put(b, TLM_BMS_FAULT, (int32_t)(s.fault_flags | BmsLowBatt_Flags(&s_lb)));
    Tlm_Put(b, TLM_BMS_ALARM, s.alarm_status);
    Tlm_Put(b, TLM_BMS_SA, s.safety_status_a);
    Tlm_Put(b, TLM_BMS_SB, s.safety_status_b);
    Tlm_Put(b, TLM_BMS_SC, s.safety_status_c);
    Tlm_Put(b, TLM_BMS_FET, s.fet_status);
    Tlm_Put(b, TLM_BMS_MANUF, s.manuf_status);
    Tlm_Put(b, TLM_BMS_INIT_STEP, s.init_step);
    Tlm_Put(b, TLM_BMS_CFG_FAIL, s.cfg_fail_count);
    Tlm_Put(b, TLM_BMS_VCELL_RB, s.vcell_mode_rb);
    Tlm_Put(b, TLM_BMS_BATT_STATUS, s.battery_status);
    Tlm_Put(b, TLM_BMS_CHG, s.chg_fet_on ? 1 : 0);
    Tlm_Put(b, TLM_BMS_DSG, s.dsg_fet_on ? 1 : 0);
    Tlm_Put(b, TLM_BMS_FETS, s.fets_enabled ? 1 : 0);
    Tlm_Put(b, TLM_BMS_SERIES, (int32_t)BMS_SERIES_COUNT);
    Tlm_Put(b, TLM_BMS_SAMPLE, s.sample_valid ? 1 : 0);
    if (s.sample_valid) {
        for (i = 0U; i < BQ76922_CELL_COUNT; i++) {
            Tlm_Put(b, (uint8_t)(TLM_BMS_C1_MV + i), s.cell_mv[i]);
        }
        Tlm_Put(b, TLM_BMS_MIN_MV, s.min_cell_mv);
        Tlm_Put(b, TLM_BMS_MAX_MV, s.max_cell_mv);
        Tlm_Put(b, TLM_BMS_DV_MV, (int32_t)s.max_cell_mv - (int32_t)s.min_cell_mv);
        Tlm_Put(b, TLM_BMS_SUM_MV, (uint16_t)s.cell_sum_mv);
        Tlm_Put(b, TLM_BMS_PACK_MV, (uint16_t)s.pack_mv);
        Tlm_Put(b, TLM_BMS_STACK_MV, (uint16_t)s.stack_mv);
        Tlm_Put(b, TLM_BMS_I_PACK_MA, s.cc2_ma);
        Tlm_Put(b, TLM_BMS_CC1_MA, s.cc1_ma);
        if (s.int_temp_dk != 0) {
            Tlm_Put(b, TLM_BMS_INT_TEMP_DK, s.int_temp_dk);
        }
        if (s.ts1_temp_dk != 0) {
            Tlm_Put(b, TLM_BMS_TS1_DK, s.ts1_temp_dk);
        }
        Tlm_Put(b, TLM_BMS_BALANCE, s.balance_mask);
    }
    if (s.passq_valid) {
        Tlm_Put(b, TLM_BMS_PASSQ_MAH, s.passq_mah);
    }
    if (g.soc_permille != BMS_GAUGE_INVALID_PERMILLE) {
        Tlm_Put(b, TLM_BMS_SOC, g.soc_permille);
    }
    Tlm_Put(b, TLM_BMS_SOC_FLAGS, g.flags);
    Tlm_Put(b, TLM_BMS_GOLDEN, (s.golden_ok ? 1 : 0) | (s.golden_otp ? 2 : 0));
    Tlm_Put(b, TLM_BMS_I2C_ERR, (int32_t)s.i2c_error_count);
    Tlm_Put(b, TLM_BMS_ALERTS, (int32_t)s.alert_count);
}

void BmsMgr_FillBatt(TlmBuilder_t *b)
{
    BQ76922_Snapshot_t s;
    BmsGauge_Output_t g;

    BQ76922_GetSnapshot(&g_bq76922, &s);
    BmsGauge_GetOutput(&g);
    Tlm_Begin(b, TLM_GROUP_BATT);
    Tlm_Put(b, TLM_BATT_CHEM, 1);            /* 1 = LiFePO4 */
    Tlm_Put(b, TLM_BATT_CELLS, (int32_t)BMS_SERIES_COUNT);
    Tlm_Put(b, TLM_BATT_DESIGN_MAH, BMS_DESIGN_CAPACITY_MAH);
    if (g.soc_permille != BMS_GAUGE_INVALID_PERMILLE) {
        Tlm_Put(b, TLM_BATT_SOC, g.soc_permille);
        Tlm_Put(b, TLM_BATT_REM_MAH, g.remaining_mah);
    }
    Tlm_Put(b, TLM_BATT_FLAGS, g.flags);
    Tlm_Put(b, TLM_BATT_FCC_MAH, g.fcc_mah);
    Tlm_Put(b, TLM_BATT_SOH, g.soh_permille);
    Tlm_Put(b, TLM_BATT_CYCLES, g.cycles_x100);
    Tlm_Put(b, TLM_BATT_RCELL, g.rcell_mohm_x10);
    Tlm_Put(b, TLM_BATT_OCV_MV, g.ocv_avg_mv);
    if (g.tte_min != 0xFFFFU) {
        Tlm_Put(b, TLM_BATT_TTE_MIN, g.tte_min);
    }
    if (g.ttf_min != 0xFFFFU) {
        Tlm_Put(b, TLM_BATT_TTF_MIN, g.ttf_min);
    }
    Tlm_Put(b, TLM_BATT_ANCHOR, g.last_anchor);
    Tlm_Put(b, TLM_BATT_LEARN_N, g.learn_count);
    Tlm_Put(b, TLM_BATT_LEARN_MAH, g.learn_delta_mah);
    Tlm_Put(b, TLM_BATT_CHG_TOTAL, (int32_t)g.chg_total_mah);
    Tlm_Put(b, TLM_BATT_DSG_TOTAL, (int32_t)g.dsg_total_mah);
    Tlm_Put(b, TLM_BATT_STORE_SEQ, (int32_t)BmsStore_LastSeq());
    Tlm_Put(b, TLM_BATT_STORE_OK, (s_last_save_ok ? 1 : 0) | (s_loaded ? 2 : 0) |
                                  (s_loaded_clean ? 4 : 0));
    Tlm_Put(b, TLM_BATT_UNSAVED, BmsGauge_UnsavedMah());
    if (s.ts1_temp_dk != 0) {
        Tlm_Put(b, TLM_BATT_TEMP_DC, dk_to_dc(s.ts1_temp_dk));
    }
}
