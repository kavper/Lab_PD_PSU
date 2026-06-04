#include "measurements.h"

#define MEAS_ADC_TIMEOUT_MS              5U
#define MEAS_ADC_VREF_V                  3.3f
#define MEAS_ADC_FULL_SCALE              4095.0f
#define MEAS_IIR_ALPHA                   0.20f
#define MEAS_DMA_DEPTH                   64U
#define MEAS_ADC1_CHANNELS               2U
#define MEAS_ADC1_DMA_LENGTH             (MEAS_DMA_DEPTH * MEAS_ADC1_CHANNELS)
#define MEAS_ADC2_DMA_LENGTH             MEAS_DMA_DEPTH
#define MEAS_DMA_SENTINEL                0xFFFFU

/*
 * TODO: ustaw zgodnie z realnym torem pomiarowym.
 * Tymczasowe 11:1 daje sensowny zakres bring-up ok. 0..36 V z ADC 3.3 V.
 * VOUT skorygowane po wydluzeniu sample time: FW 30.00 V -> miernik 28.85 V.
 */
#define MEAS_VIN_DIVIDER_RATIO           11.0f
#define MEAS_VOUT_DIVIDER_RATIO          10.807530f
/* Kalibracja 2-punktowa: 0.008 A -> 0.000 A, 2.764 A -> 2.741 A. */
#define MEAS_IOUT_CAL_GAIN               1.813604f
#define MEAS_IOUT_A_PER_V                (1.0f * MEAS_IOUT_CAL_GAIN)
#define MEAS_IOUT_OFFSET_A               -0.007956f

typedef struct {
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
    float vin_filt;
    float vout_filt;
    float iout_filt;
    float vbat_filt;
    HAL_StatusTypeDef last_hal_status;
    uint32_t dma_update_count;
    uint32_t last_adc1_slot;
    uint8_t last_error;
    bool filter_ready;
    bool dma_running;
    bool dma_seen_once;
    bool initialized;
} Measurements_Context_t;

static Measurements_Context_t g_meas_ctx;
static uint16_t adc1_dma_buffer[MEAS_ADC1_DMA_LENGTH];
static uint16_t adc2_dma_buffer[MEAS_ADC2_DMA_LENGTH];

enum {
    MEAS_ERR_NONE = 0U,
    MEAS_ERR_INIT_ADC1,
    MEAS_ERR_INIT_ADC2,
    MEAS_ERR_CAL_ADC1,
    MEAS_ERR_CAL_ADC2,
    MEAS_ERR_START_ADC1,
    MEAS_ERR_POLL_ADC1_R1,
    MEAS_ERR_POLL_ADC1_R2,
    MEAS_ERR_START_ADC2,
    MEAS_ERR_POLL_ADC2_R1,
    MEAS_ERR_DMA_START_ADC1,
    MEAS_ERR_DMA_START_ADC2,
    MEAS_ERR_DMA_NO_DATA
};

static float Meas_AdcToVoltage(uint16_t raw)
{
    return ((float)raw * MEAS_ADC_VREF_V) / MEAS_ADC_FULL_SCALE;
}

static float Meas_Iir(float prev, float input)
{
    return prev + MEAS_IIR_ALPHA * (input - prev);
}

static void Meas_FillDmaBuffers(void)
{
    uint32_t i;

    for (i = 0U; i < MEAS_ADC1_DMA_LENGTH; ++i) {
        adc1_dma_buffer[i] = MEAS_DMA_SENTINEL;
    }

    for (i = 0U; i < MEAS_ADC2_DMA_LENGTH; ++i) {
        adc2_dma_buffer[i] = MEAS_DMA_SENTINEL;
    }
}

static bool Meas_EnsureCircularDma(ADC_HandleTypeDef *hadc)
{
    if ((hadc == NULL) || (hadc->DMA_Handle == NULL)) {
        return false;
    }

    if (hadc->DMA_Handle->Init.Mode != DMA_CIRCULAR) {
        hadc->DMA_Handle->Init.Mode = DMA_CIRCULAR;
        if (HAL_DMA_Init(hadc->DMA_Handle) != HAL_OK) {
            return false;
        }
    }

    return true;
}

static bool Meas_LatestAdc1Slot(uint32_t *slot)
{
    uint32_t remaining;
    uint32_t written;

    if ((slot == NULL) ||
        (g_meas_ctx.hadc1 == NULL) ||
        (g_meas_ctx.hadc1->DMA_Handle == NULL)) {
        return false;
    }

    remaining = __HAL_DMA_GET_COUNTER(g_meas_ctx.hadc1->DMA_Handle);
    if (remaining > MEAS_ADC1_DMA_LENGTH) {
        return false;
    }

    written = MEAS_ADC1_DMA_LENGTH - remaining;
    if (written >= MEAS_ADC1_CHANNELS) {
        *slot = (written / MEAS_ADC1_CHANNELS) - 1U;
    } else {
        *slot = MEAS_DMA_DEPTH - 1U;
    }

    if (*slot >= MEAS_DMA_DEPTH) {
        *slot = MEAS_DMA_DEPTH - 1U;
    }

    return true;
}

static bool Meas_LatestAdc2Slot(uint32_t *slot)
{
    uint32_t remaining;
    uint32_t written;

    if ((slot == NULL) ||
        (g_meas_ctx.hadc2 == NULL) ||
        (g_meas_ctx.hadc2->DMA_Handle == NULL)) {
        return false;
    }

    remaining = __HAL_DMA_GET_COUNTER(g_meas_ctx.hadc2->DMA_Handle);
    if (remaining > MEAS_ADC2_DMA_LENGTH) {
        return false;
    }

    written = MEAS_ADC2_DMA_LENGTH - remaining;
    if (written >= 1U) {
        *slot = written - 1U;
    } else {
        *slot = MEAS_DMA_DEPTH - 1U;
    }

    if (*slot >= MEAS_DMA_DEPTH) {
        *slot = MEAS_DMA_DEPTH - 1U;
    }

    return true;
}

void Measurements_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
    g_meas_ctx.hadc1 = hadc1;
    g_meas_ctx.hadc2 = hadc2;
    g_meas_ctx.vin_filt = 0.0f;
    g_meas_ctx.vout_filt = 0.0f;
    g_meas_ctx.iout_filt = 0.0f;
    g_meas_ctx.vbat_filt = 0.0f;
    g_meas_ctx.last_hal_status = HAL_OK;
    g_meas_ctx.dma_update_count = 0U;
    g_meas_ctx.last_adc1_slot = MEAS_DMA_DEPTH;
    g_meas_ctx.last_error = MEAS_ERR_NONE;
    g_meas_ctx.filter_ready = false;
    g_meas_ctx.dma_running = false;
    g_meas_ctx.dma_seen_once = false;
    g_meas_ctx.initialized = false;

    if ((hadc1 == NULL) || (hadc2 == NULL)) {
        return;
    }

    Meas_FillDmaBuffers();

    hadc1->Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
    hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1->Init.DMAContinuousRequests = ENABLE;
    hadc1->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc2->Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
    hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2->Init.DMAContinuousRequests = ENABLE;
    hadc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    if (!Meas_EnsureCircularDma(hadc1)) {
        g_meas_ctx.last_error = MEAS_ERR_INIT_ADC1;
        return;
    }

    if (!Meas_EnsureCircularDma(hadc2)) {
        g_meas_ctx.last_error = MEAS_ERR_INIT_ADC2;
        return;
    }

    g_meas_ctx.last_hal_status = HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED);
    if (g_meas_ctx.last_hal_status != HAL_OK) {
        g_meas_ctx.last_error = MEAS_ERR_CAL_ADC1;
        return;
    }

    g_meas_ctx.last_hal_status = HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED);
    if (g_meas_ctx.last_hal_status != HAL_OK) {
        g_meas_ctx.last_error = MEAS_ERR_CAL_ADC2;
        return;
    }

    g_meas_ctx.last_hal_status = HAL_ADC_Start_DMA(hadc1,
                                                   (uint32_t *)adc1_dma_buffer,
                                                   MEAS_ADC1_DMA_LENGTH);
    if (g_meas_ctx.last_hal_status != HAL_OK) {
        g_meas_ctx.last_error = MEAS_ERR_DMA_START_ADC1;
        return;
    }

    g_meas_ctx.last_hal_status = HAL_ADC_Start_DMA(hadc2,
                                                   (uint32_t *)adc2_dma_buffer,
                                                   MEAS_ADC2_DMA_LENGTH);
    if (g_meas_ctx.last_hal_status != HAL_OK) {
        g_meas_ctx.last_error = MEAS_ERR_DMA_START_ADC2;
        return;
    }

    g_meas_ctx.dma_running = true;
    g_meas_ctx.initialized = true;
    g_meas_ctx.last_error = MEAS_ERR_NONE;
}

bool Measurements_Update(Measurements_t *meas)
{
    uint16_t raw_vout;
    uint16_t raw_vin;
    uint16_t raw_iout;
    uint32_t adc1_slot;
    uint32_t adc2_slot;
    float vin;
    float vout;
    float iout;
    float vbat;

    if ((meas == NULL) || (!g_meas_ctx.initialized)) {
        g_meas_ctx.last_error = MEAS_ERR_INIT_ADC1;
        return false;
    }

    if ((!g_meas_ctx.dma_running) ||
        (!Meas_LatestAdc1Slot(&adc1_slot)) ||
        (!Meas_LatestAdc2Slot(&adc2_slot))) {
        g_meas_ctx.last_error = MEAS_ERR_DMA_NO_DATA;
        return false;
    }

    /* ADC1 rank2: ADC_CHANNEL_1 (PA0, ADC_VBAT_Pin -> tymczasowo traktowane jako VIN). */
    raw_vout = adc1_dma_buffer[(adc1_slot * MEAS_ADC1_CHANNELS) + 0U];
    raw_vin = adc1_dma_buffer[(adc1_slot * MEAS_ADC1_CHANNELS) + 1U];
    raw_iout = adc2_dma_buffer[adc2_slot];

    if ((raw_vout == MEAS_DMA_SENTINEL) ||
        (raw_vin == MEAS_DMA_SENTINEL) ||
        (raw_iout == MEAS_DMA_SENTINEL) ||
        (raw_vout > (uint16_t)MEAS_ADC_FULL_SCALE) ||
        (raw_vin > (uint16_t)MEAS_ADC_FULL_SCALE) ||
        (raw_iout > (uint16_t)MEAS_ADC_FULL_SCALE)) {
        g_meas_ctx.last_error = MEAS_ERR_DMA_NO_DATA;
        return false;
    }

    if ((!g_meas_ctx.dma_seen_once) || (adc1_slot != g_meas_ctx.last_adc1_slot)) {
        g_meas_ctx.dma_update_count++;
        g_meas_ctx.last_adc1_slot = adc1_slot;
        g_meas_ctx.dma_seen_once = true;
    }

    vout = Meas_AdcToVoltage(raw_vout) * MEAS_VOUT_DIVIDER_RATIO;
    vin = Meas_AdcToVoltage(raw_vin) * MEAS_VIN_DIVIDER_RATIO;
    vbat = vin;
    iout = Meas_AdcToVoltage(raw_iout) * MEAS_IOUT_A_PER_V + MEAS_IOUT_OFFSET_A;

    if (iout < 0.0f) {
        iout = 0.0f;
    }

    if (!g_meas_ctx.filter_ready) {
        g_meas_ctx.vin_filt = vin;
        g_meas_ctx.vout_filt = vout;
        g_meas_ctx.iout_filt = iout;
        g_meas_ctx.vbat_filt = vbat;
        g_meas_ctx.filter_ready = true;
    } else {
        g_meas_ctx.vin_filt = Meas_Iir(g_meas_ctx.vin_filt, vin);
        g_meas_ctx.vout_filt = Meas_Iir(g_meas_ctx.vout_filt, vout);
        g_meas_ctx.iout_filt = Meas_Iir(g_meas_ctx.iout_filt, iout);
        g_meas_ctx.vbat_filt = Meas_Iir(g_meas_ctx.vbat_filt, vbat);
    }

    meas->vin = g_meas_ctx.vin_filt;
    meas->vout = g_meas_ctx.vout_filt;
    meas->iout = g_meas_ctx.iout_filt;
    meas->vbat = g_meas_ctx.vbat_filt;
    meas->raw_vin = raw_vin;
    meas->raw_vout = raw_vout;
    meas->raw_iout = raw_iout;
    meas->valid = true;
    g_meas_ctx.last_error = MEAS_ERR_NONE;
    g_meas_ctx.last_hal_status = HAL_OK;

    return true;
}

uint8_t Measurements_GetLastError(void)
{
    return g_meas_ctx.last_error;
}

HAL_StatusTypeDef Measurements_GetLastHalStatus(void)
{
    return g_meas_ctx.last_hal_status;
}

uint32_t Measurements_GetDmaUpdateCount(void)
{
    return g_meas_ctx.dma_update_count;
}

bool Measurements_IsDmaRunning(void)
{
    return g_meas_ctx.dma_running;
}
