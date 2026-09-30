/* Exercise the production power-stage code with RAM registers and a fake HAL.
 * No target hardware is accessed by this host executable. */
/* Unused ARM-only flash section declarations are not valid Mach-O sections. */
#define STM32G4xx_FLASH_RAMFUNC_H
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static GPIO_TypeDef test_gpio_a, test_gpio_c;
static EXTI_TypeDef test_exti;
#undef GPIOA
#undef GPIOC
#undef EXTI
#define GPIOA (&test_gpio_a)
#define GPIOC (&test_gpio_c)
#define EXTI (&test_exti)
#include "../Core/Src/power_stage.c"

static uint32_t tick_ms, outputs, en_falls, en_rises;
static bool inject_fault_on_start;
static HRTIM_TypeDef timer_regs;
static HRTIM_HandleTypeDef timer = { .Instance = &timer_regs };
uint32_t SystemCoreClock = 170000000U;
uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return SystemCoreClock; }
void LdoPrereg_SetPermitOverrideOff(bool off) { (void)off; }
bool LdoPrereg_IsPermitGranted(void) { return true; }
void HAL_GPIO_Init(GPIO_TypeDef *p, const GPIO_InitTypeDef *cfg) { (void)p; (void)cfg; }
GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *p, uint16_t pin)
{ return (p->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState state)
{
    bool was_on = (p->ODR & pin) != 0U;
    if (state == GPIO_PIN_SET) {
        p->ODR |= pin;
        en_rises += !was_on;
    } else {
        p->ODR &= ~pin;
        en_falls += was_on;
    }
    /* Simulate the EN/FLT node following its EN output. */
    GPIO_TypeDef *sense = pin == BUCK_TR_EN_Pin ? GPIOC : GPIOA;
    uint16_t sense_pin = pin == BUCK_TR_EN_Pin ? BUCK_TR_FLT_Pin : BOOST_TR_FLT_Pin;
    if (state == GPIO_PIN_SET) sense->IDR |= sense_pin;
    else {
        sense->IDR &= ~sense_pin;
        if (was_on) PowerStage_UccFaultExti(sense_pin);
    }
}
#define HAL_OK_2(name) \
HAL_StatusTypeDef name(HRTIM_HandleTypeDef *h, uint32_t v) \
{ (void)h; (void)v; return HAL_OK; }
HAL_OK_2(HAL_HRTIM_BurstModeCtl)
HAL_OK_2(HAL_HRTIM_WaveformCounterStart)
HAL_OK_2(HAL_HRTIM_WaveformCounterStop)
HAL_OK_2(HAL_HRTIM_SoftwareUpdate)
HAL_StatusTypeDef HAL_HRTIM_WaveformOutputStart(HRTIM_HandleTypeDef *h, uint32_t v)
{
    (void)h; outputs |= v;
    if (inject_fault_on_start) PowerStage_UccFaultExti(BOOST_TR_FLT_Pin);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_HRTIM_WaveformOutputStop(HRTIM_HandleTypeDef *h, uint32_t v)
{ (void)h; outputs &= ~v; return HAL_OK; }
HAL_StatusTypeDef HAL_HRTIM_WaveformOutputConfig(HRTIM_HandleTypeDef *h,
    uint32_t t, uint32_t o, const HRTIM_OutputCfgTypeDef *cfg)
{ (void)h; (void)t; (void)o; (void)cfg; return HAL_OK; }
HAL_StatusTypeDef HAL_HRTIM_WaveformSetOutputLevel(HRTIM_HandleTypeDef *h,
    uint32_t t, uint32_t o, uint32_t level)
{ (void)h; (void)t; (void)o; (void)level; return HAL_OK; }
HAL_StatusTypeDef HAL_HRTIM_BurstModeConfig(HRTIM_HandleTypeDef *h,
    const HRTIM_BurstModeCfgTypeDef *cfg)
{ (void)h; (void)cfg; return HAL_OK; }
HAL_StatusTypeDef HAL_HRTIM_BurstModeSoftwareTrigger(HRTIM_HandleTypeDef *h)
{ (void)h; return HAL_OK; }

static void reset_stage(void)
{
    memset(&timer_regs, 0, sizeof timer_regs);
    memset(&test_gpio_a, 0, sizeof test_gpio_a);
    memset(&test_gpio_c, 0, sizeof test_gpio_c);
    memset(&ps, 0, sizeof ps);
    tick_ms = outputs = en_falls = en_rises = 0;
    inject_fault_on_start = false;
    PowerStage_Init(&timer);
    assert(!PowerStage_IsFaultActive());
    assert(en_rises == 0U); /* Init must never pulse EN via SetDuty(0,0). */
}

int main(void)
{
    reset_stage();
    PowerStage_SetBuckDuty(0.5f);
    assert(!PowerStage_IsBuckTrEnActive() && PowerStage_IsBoostTrEnActive());
    assert(!ps.static_c && PowerStage_GetExpectedTc1Duty10k() <= 9000U);
    assert(PowerStage_Enable());
    assert(en_falls == 0U && en_rises == 1U); /* Enable preserves pre-start. */
    for (tick_ms = 1; tick_ms < DCDC_UCC_STARTUP_MS; tick_ms++) {
        PowerStage_SetBuckDuty(0.5f);
        assert(!ps.static_c && PowerStage_IsBoostTrEnActive());
        assert(PowerStage_GetExpectedTc1Duty10k() <= 9000U);
    }
    PowerStage_SetBuckDuty(0.5f);
    assert(ps.static_c && PowerStage_GetExpectedTc1Duty10k() == 10000U);
    assert(en_rises == 1U);

    /* A late-arriving buck request has its own startup timer. */
    PowerStage_SetBuckDuty(1.0f);
    assert(!ps.static_a && ps.static_c && ps.duty_a_10k == 9000U);
    tick_ms += DCDC_UCC_STARTUP_MS;
    PowerStage_SetBuckDuty(1.0f);
    assert(ps.static_a && ps.static_c);
    PowerStage_SetBuckDuty(0.93f);
    assert(ps.tr_en_a_active && !ps.static_a);
    PowerStage_SetBuckDuty(0.929f);
    assert(!ps.tr_en_a_active && !PowerStage_IsFaultActive());

    PowerStage_SuspendOutputsKeepDriverOn();
    assert(!ps.tr_en_a_active && !ps.tr_en_c_active && outputs == 0U);
    assert(!PowerStage_IsFaultActive());

    reset_stage();
    PowerStage_SetBoostDuty(0.4f);
    assert(ps.tr_en_a_active && !ps.tr_en_c_active && !ps.static_a);
    assert(PowerStage_Enable());
    tick_ms = DCDC_UCC_STARTUP_MS;
    PowerStage_SetBoostDuty(0.4f);
    assert(ps.static_a && !ps.static_c);

    /* A pulse that has ended before the next polling cycle stays latched. */
    PowerStage_UccFaultExti(BUCK_TR_FLT_Pin);
    assert(timer_regs.sCommonRegs.ODISR == POWER_STAGE_OUTPUTS);
    assert(PowerStage_IsFaultActive());
    PowerStage_Disable();
    assert(PowerStage_IsFaultActive());
    assert(!PowerStage_Enable());
    PowerStage_ClearDriverFault();
    assert(!PowerStage_IsFaultActive());

    reset_stage();
    PowerStage_SetBuckDuty(0.5f);
    PowerStage_UccFaultExti(BOOST_TR_FLT_Pin); /* Fault during startup. */
    assert(PowerStage_IsFaultActive());
    assert(!PowerStage_Enable());

    reset_stage();
    PowerStage_SetBuckDuty(0.5f);
    GPIOA->IDR &= ~BOOST_TR_FLT_Pin; /* EN/FLT never rose. */
    tick_ms = 1;
    assert(PowerStage_IsFaultActive());

    reset_stage();
    PowerStage_SetBuckDuty(0.5f);
    inject_fault_on_start = true;
    assert(!PowerStage_Enable() && outputs == 0U);

    /* Pending fault after OutputStart must also win in the discharge path. */
    reset_stage();
    PowerStage_SetBuckDischarge(100, 2);
    tick_ms = DCDC_UCC_STARTUP_MS;
    inject_fault_on_start = true;
    PowerStage_SetBuckDischarge(100, 2);
    assert(PowerStage_IsFaultActive() && outputs == 0U && !ps.discharge_active);

    /* Both legs in mixed operation warm up independently without EN chatter. */
    reset_stage();
    PowerStage_SetBuckBoostDuty(0.96f, 0.02f);
    assert(ps.tr_en_a_active && ps.tr_en_c_active && PowerStage_IsUccStarting());
    assert(PowerStage_Enable());
    tick_ms = DCDC_UCC_STARTUP_MS;
    PowerStage_SetBuckBoostDuty(0.96f, 0.02f);
    assert(!PowerStage_IsUccStarting() && !ps.static_a && !ps.static_c);
    assert(ps.duty_a_10k == 9600U && ps.duty_c_cmd_10k == 200U);
    assert(en_rises == 2U && en_falls == 0U);
    PowerStage_Disable();
    unsigned rises_before = en_rises;
    PowerStage_Disable();
    assert(en_rises == rises_before && !PowerStage_IsFaultActive());
    tick_ms += 100U;
    PowerStage_SetBuckDuty(0.5f);
    assert(PowerStage_IsUccStarting() && !ps.static_c);

    reset_stage();
    PowerStage_SetBuckDischarge(100, 2);
    assert(ps.tr_en_c_active && outputs == 0U && !ps.discharge_active);
    tick_ms = DCDC_UCC_STARTUP_MS;
    PowerStage_SetBuckDischarge(100, 2);
    assert(ps.discharge_active && outputs != 0U);
    PowerStage_Disable();
    assert(!PowerStage_IsFaultActive() && outputs == 0U);

    puts("test_power_stage_ucc: PASS");
    return 0;
}
