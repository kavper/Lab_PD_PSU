#include "remote_sense.h"

#include "board_rev.h"
#include "debug_uart.h"
#include "sense_check.h"

#define SENSE_PERIOD_MS                  100U
#define SENSE_SPIN_LIMIT                 200000U
#define SENSE_VREF_MV                    ((uint32_t)(BOARD_VREF_V * 1000.0f + 0.5f))

static ADC_HandleTypeDef *s_hadc;
static bool s_configured;
static bool s_wanted;
static bool s_closed;
static uint8_t s_code = SENSE_NO_SAMPLE;
static uint8_t s_ok_streak;
static uint16_t s_local_mv = SENSE_MV_MISSING;
static uint16_t s_remote_p_mv = SENSE_MV_MISSING;
static uint16_t s_remote_n_mv = SENSE_MV_MISSING;
static uint32_t s_last_ms;
static bool s_task_started;

static void RemoteSense_Drive(bool closed)
{
    if (closed == s_closed) {
        return;
    }
    s_closed = closed;
    HAL_GPIO_WritePin(REMOTE_ON_GPIO_Port, REMOTE_ON_Pin,
                      closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Debug_Printf("[SENSE] relay %s code=%u\r\n",
                 closed ? "REMOTE" : "LOCAL",
                 (unsigned int)s_code);
}

static bool RemoteSense_ConfigInjected(ADC_HandleTypeDef *hadc)
{
    ADC_InjectionConfTypeDef cfg = {0};
    const uint32_t channels[3] = {
        ADC_CHANNEL_5,   /* PB14 ADC_LOCAL_VOUT */
        ADC_CHANNEL_15,  /* PB0  ADC_REMOTE_P */
        ADC_CHANNEL_12   /* PB1  ADC_REMOTE_N */
    };
    const uint32_t ranks[3] = {
        ADC_INJECTED_RANK_1,
        ADC_INJECTED_RANK_2,
        ADC_INJECTED_RANK_3
    };
    uint32_t i;

    cfg.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    cfg.InjectedSingleDiff = ADC_SINGLE_ENDED;
    cfg.InjectedOffsetNumber = ADC_OFFSET_NONE;
    cfg.InjectedOffset = 0U;
    cfg.InjectedOffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
    cfg.InjectedOffsetSaturation = DISABLE;
    cfg.InjectedNbrOfConversion = 3U;
    cfg.InjectedDiscontinuousConvMode = DISABLE;
    cfg.AutoInjectedConv = DISABLE;
    cfg.QueueInjectedContext = DISABLE;
    cfg.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    cfg.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    cfg.InjecOversamplingMode = DISABLE;

    for (i = 0U; i < 3U; i++) {
        cfg.InjectedChannel = channels[i];
        cfg.InjectedRank = ranks[i];
        if (HAL_ADCEx_InjectedConfigChannel(hadc, &cfg) != HAL_OK) {
            return false;
        }
    }
    return true;
}

static bool RemoteSense_Sample(uint32_t *local_mv, uint32_t *remote_p_mv,
                               uint32_t *remote_n_mv)
{
    uint32_t spins = 0U;
    uint16_t local_counts;
    uint16_t p_counts;
    uint16_t n_counts;

    if ((s_hadc == NULL) || (!s_configured)) {
        return false;
    }

    LL_ADC_ClearFlag_JEOS(s_hadc->Instance);
    LL_ADC_INJ_StartConversion(s_hadc->Instance);
    while (LL_ADC_IsActiveFlag_JEOS(s_hadc->Instance) == 0U) {
        if (++spins >= SENSE_SPIN_LIMIT) {
            return false;
        }
    }
    LL_ADC_ClearFlag_JEOS(s_hadc->Instance);

    local_counts = LL_ADC_INJ_ReadConversionData12(s_hadc->Instance,
                                                   LL_ADC_INJ_RANK_1);
    p_counts = LL_ADC_INJ_ReadConversionData12(s_hadc->Instance,
                                               LL_ADC_INJ_RANK_2);
    n_counts = LL_ADC_INJ_ReadConversionData12(s_hadc->Instance,
                                               LL_ADC_INJ_RANK_3);
    *local_mv = Sense_CountsToMv(local_counts, SENSE_VREF_MV);
    *remote_p_mv = Sense_CountsToMv(p_counts, SENSE_VREF_MV);
    *remote_n_mv = Sense_CountsToMv(n_counts, SENSE_VREF_MV);
    return true;
}

void RemoteSense_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    s_configured = false;
    s_wanted = false;
    s_closed = false;
    s_code = SENSE_NO_SAMPLE;
    s_ok_streak = 0U;
    s_local_mv = SENSE_MV_MISSING;
    s_remote_p_mv = SENSE_MV_MISSING;
    s_remote_n_mv = SENSE_MV_MISSING;
    s_last_ms = 0U;
    s_task_started = false;
    HAL_GPIO_WritePin(REMOTE_ON_GPIO_Port, REMOTE_ON_Pin, GPIO_PIN_RESET);

    if (hadc != NULL) {
        s_configured = RemoteSense_ConfigInjected(hadc);
    }
    Debug_Printf("[SENSE] self-test %s (divider x12, relay stays local)\r\n",
                 s_configured ? "ready" : "ADC config failed");
}

void RemoteSense_Request(bool enable)
{
    if (enable == s_wanted) {
        return;
    }
    s_wanted = enable;
    s_ok_streak = 0U;
    Debug_Printf("[SENSE] request %s\r\n", enable ? "REMOTE" : "LOCAL");
    if (!enable) {
        RemoteSense_Drive(false);
    }
}

bool RemoteSense_IsClosed(void)
{
    return s_closed;
}

bool RemoteSense_IsWanted(void)
{
    return s_wanted;
}

uint8_t RemoteSense_Code(void)
{
    return s_code;
}

uint16_t RemoteSense_LocalMv(void)
{
    return s_local_mv;
}

uint16_t RemoteSense_RemotePMv(void)
{
    return s_remote_p_mv;
}

uint16_t RemoteSense_RemoteNMv(void)
{
    return s_remote_n_mv;
}

void RemoteSense_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t local_mv = 0U;
    uint32_t remote_p_mv = 0U;
    uint32_t remote_n_mv = 0U;
    uint8_t code;

    if (s_task_started &&
        ((uint32_t)(now_ms - s_last_ms) < SENSE_PERIOD_MS)) {
        return;
    }
    s_task_started = true;
    s_last_ms = now_ms;

    if (!RemoteSense_Sample(&local_mv, &remote_p_mv, &remote_n_mv)) {
        s_code = SENSE_NO_SAMPLE;
        s_local_mv = SENSE_MV_MISSING;
        s_remote_p_mv = SENSE_MV_MISSING;
        s_remote_n_mv = SENSE_MV_MISSING;
        s_ok_streak = 0U;
        RemoteSense_Drive(false);
        return;
    }

    s_local_mv = Sense_MvToU16(local_mv);
    s_remote_p_mv = Sense_MvToU16(remote_p_mv);
    s_remote_n_mv = Sense_MvToU16(remote_n_mv);
    code = Sense_Classify(local_mv, remote_p_mv, remote_n_mv);
    if (code != s_code) {
        Debug_Printf("[SENSE] code %u -> %u local=%lu p=%lu n=%lu\r\n",
                     (unsigned int)s_code,
                     (unsigned int)code,
                     (unsigned long)local_mv,
                     (unsigned long)remote_p_mv,
                     (unsigned long)remote_n_mv);
        s_code = code;
    }

    if ((!s_wanted) || (code != SENSE_OK)) {
        s_ok_streak = 0U;
        RemoteSense_Drive(false);
        return;
    }

    if (s_ok_streak < SENSE_OK_STREAK) {
        s_ok_streak++;
    }
    if (s_ok_streak >= SENSE_OK_STREAK) {
        RemoteSense_Drive(true);
    }
}
