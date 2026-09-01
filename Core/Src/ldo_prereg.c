#include "ldo_prereg.h"

#include "app.h"
#include "board_rev.h"
#include "debug_uart.h"
#include "ldo_link.h"

#include <string.h>

#define LDO_G0_MODE_CC                       2U
#define LDO_TLM_STALE_MS                     500U
#define PREREG_SLEW_UP_V_PER_S               10.0f
#define PREREG_SLEW_DOWN_V_PER_S             0.3f
#define PREREG_REGULATION_BAND_V             0.50f
#define PREREG_PERMIT_SETTLE_MS              150U
#define PREREG_PERMIT_RELEASE_MS             50U

static LdoPrereg_Status_t s_status;
static float s_command_v;
static uint32_t s_last_task_ms;
static uint32_t s_regulation_since_ms;
static uint32_t s_out_of_reg_since_ms;
static bool s_force_disable;
static bool s_last_permit_granted;

static float Prereg_ClampV(float value)
{
    if (value < BOARD_VPRE_MIN_V) {
        return BOARD_VPRE_MIN_V;
    }
    if (value > BOARD_VPRE_MAX_V) {
        return BOARD_VPRE_MAX_V;
    }
    return value;
}

static bool Prereg_FaultIsNone(const char *fault)
{
    if ((fault == NULL) || (fault[0] == '\0')) {
        return true;
    }
    return (strcmp(fault, "NONE") == 0);
}

static float Prereg_ComputeRequestV(const LdoLink_Status_t *ldo)
{
    float request_v;

    if ((ldo == NULL) || (!ldo->output_on)) {
        return BOARD_VPRE_MIN_V;
    }

    if (ldo->vpre_present && (ldo->vpre_mv > 0U)) {
        request_v = (float)ldo->vpre_mv / 1000.0f;
    } else if ((ldo->cc_cv != 0U) || (ldo->mode == LDO_G0_MODE_CC)) {
        request_v = ((float)ldo->vout_mv / 1000.0f) + BOARD_VPRE_MARGIN_V;
    } else {
        request_v = ((float)ldo->vset_mv / 1000.0f) + BOARD_VPRE_MARGIN_V;
    }

    return Prereg_ClampV(request_v);
}

static void Prereg_UpdateSlew(float request_v, float dt_s)
{
    float delta;
    float max_up;
    float max_down;

    if (dt_s <= 0.0f) {
        return;
    }

    max_up = PREREG_SLEW_UP_V_PER_S * dt_s;
    max_down = PREREG_SLEW_DOWN_V_PER_S * dt_s;
    delta = request_v - s_command_v;

    if (delta > max_up) {
        delta = max_up;
    } else if (delta < -max_down) {
        delta = -max_down;
    }

    s_command_v = Prereg_ClampV(s_command_v + delta);
}

static bool Prereg_UpdateRegulation(float measured_v, bool dcdc_enabled, uint32_t now_ms)
{
    float error;

    if (!dcdc_enabled) {
        s_regulation_since_ms = 0U;
        s_out_of_reg_since_ms = 0U;
        return false;
    }

    error = s_command_v - measured_v;
    if (error < 0.0f) {
        error = -error;
    }

    if (error <= PREREG_REGULATION_BAND_V) {
        s_out_of_reg_since_ms = 0U;
        if (s_regulation_since_ms == 0U) {
            s_regulation_since_ms = now_ms;
        }
        return ((uint32_t)(now_ms - s_regulation_since_ms) >= PREREG_PERMIT_SETTLE_MS);
    }

    s_regulation_since_ms = 0U;
    if (s_out_of_reg_since_ms == 0U) {
        s_out_of_reg_since_ms = now_ms;
    }

    if ((uint32_t)(now_ms - s_out_of_reg_since_ms) >= PREREG_PERMIT_RELEASE_MS) {
        return false;
    }

    return s_status.regulation_ok;
}

void LdoPrereg_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_command_v = BOARD_VPRE_MIN_V;
    s_last_task_ms = HAL_GetTick();
    s_regulation_since_ms = 0U;
    s_out_of_reg_since_ms = 0U;
    s_last_permit_granted = false;
    s_force_disable = false;
    s_status.force_disable = false;
    LdoLink_SetDcdcPermitRequest(false);
}

void LdoPrereg_SetPermitOverrideOff(bool force_off)
{
    s_status.permit_override_off = force_off;
}

void LdoPrereg_SetForceDisable(bool force_disable)
{
    s_force_disable = force_disable;
    s_status.force_disable = force_disable;
}

bool LdoPrereg_IsG0Active(void)
{
    return s_status.g0_active;
}

bool LdoPrereg_ShouldEnableDcdc(void)
{
    return s_status.dcdc_enable_request;
}

bool LdoPrereg_IsPermitGranted(void)
{
    return s_status.permit_granted;
}

float LdoPrereg_GetCommandV(void)
{
    return s_command_v;
}

void LdoPrereg_GetStatus(LdoPrereg_Status_t *out)
{
    if (out != NULL) {
        *out = s_status;
    }
}

void LdoPrereg_Task(float dcdc_measured_v, bool dcdc_enabled)
{
    LdoLink_Status_t ldo;
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms;
    float dt_s;
    float request_v;
    bool g0_fresh;
    bool want_enable;
    bool want_permit;

    dt_ms = now_ms - s_last_task_ms;
    s_last_task_ms = now_ms;
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    dt_s = (float)dt_ms / 1000.0f;

    LdoLink_GetStatus(&ldo);
    g0_fresh = ldo.telemetry_valid &&
               ((uint32_t)(now_ms - ldo.last_tlm_ms) <= LDO_TLM_STALE_MS);

    s_status.g0_active = g0_fresh;
    s_status.dcdc_measured_v = dcdc_measured_v;
    s_status.g0_cccv = ldo.cc_cv;

    if (g0_fresh) {
        if (!ldo.output_on) {
            s_force_disable = false;
            s_status.force_disable = false;
        }
        request_v = Prereg_ComputeRequestV(&ldo);
        s_status.vpre_g0_v = ldo.vpre_present ?
                             ((float)ldo.vpre_mv / 1000.0f) : 0.0f;
        s_status.vpre_request_v = request_v;

        want_enable = ldo.output_on &&
                      Prereg_FaultIsNone(ldo.fault) &&
                      (ldo.pgood != 0U) &&
                      (!s_force_disable);

        Prereg_UpdateSlew(request_v, dt_s);
        s_status.vpre_command_v = s_command_v;

        s_status.regulation_ok =
            Prereg_UpdateRegulation(dcdc_measured_v, dcdc_enabled, now_ms);

        want_permit = want_enable &&
                      dcdc_enabled &&
                      s_status.regulation_ok &&
                      (!s_status.permit_override_off);

#if (BOARD_BRINGUP_LOCAL_CV != 0U)
        if ((!want_permit) &&
            (App_GetRequestedMode() == MODE_CV) &&
            (!s_force_disable) &&
            (!s_status.permit_override_off)) {
            want_permit = true;
        }
#endif

        s_status.dcdc_enable_request = want_enable;
        s_status.permit_granted = want_permit;
    } else {
        s_status.vpre_request_v = BOARD_VPRE_MIN_V;
        s_status.vpre_g0_v = 0.0f;
        s_status.regulation_ok = false;
        s_status.dcdc_enable_request = false;

#if (BOARD_BRINGUP_LOCAL_CV != 0U)
        if ((App_GetRequestedMode() == MODE_CV) &&
            (!s_force_disable) &&
            (!s_status.permit_override_off)) {
            s_status.permit_granted = true;
        } else {
            s_status.permit_granted = false;
        }
#else
        s_status.permit_granted = false;
#endif

        s_regulation_since_ms = 0U;
        s_out_of_reg_since_ms = 0U;

        if (s_command_v > BOARD_VPRE_MIN_V) {
            Prereg_UpdateSlew(BOARD_VPRE_MIN_V, dt_s);
        } else {
            s_command_v = BOARD_VPRE_MIN_V;
        }
        s_status.vpre_command_v = s_command_v;
    }

    LdoLink_SetDcdcPermitRequest(s_status.permit_granted);

    if (s_status.permit_granted != s_last_permit_granted) {
        s_last_permit_granted = s_status.permit_granted;
        Debug_Printf("[PRE] permit=%u req=%ld mV cmd=%ld mV meas=%ld mV g0_out=%u cccv=%u\r\n",
                     s_status.permit_granted ? 1U : 0U,
                     (long)(s_status.vpre_request_v * 1000.0f),
                     (long)(s_status.vpre_command_v * 1000.0f),
                     (long)(dcdc_measured_v * 1000.0f),
                     g0_fresh ? (unsigned int)ldo.output_on : 0U,
                     (unsigned int)s_status.g0_cccv);
    }
}
