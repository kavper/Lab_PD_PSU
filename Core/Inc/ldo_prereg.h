#ifndef LDO_PREREG_H
#define LDO_PREREG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * G4 DCDC pre-regulator policy driven by G0 LDO telemetry.
 * G0 runs final CC/CV; G4 holds Vin_LDO ≈ vpre_request with headroom margin.
 */

typedef struct {
    bool g0_active;
    bool dcdc_enable_request;
    bool regulation_ok;
    bool permit_granted;
    bool permit_override_off;
    bool force_disable;
    float vpre_request_v;
    float vpre_command_v;
    float vpre_g0_v;
    float dcdc_measured_v;
    uint32_t g0_cccv;
} LdoPrereg_Status_t;

void LdoPrereg_Init(void);
void LdoPrereg_Task(float dcdc_measured_v, bool dcdc_enabled);
void LdoPrereg_SetPermitOverrideOff(bool force_off);
void LdoPrereg_SetForceDisable(bool force_disable);
bool LdoPrereg_IsG0Active(void);
bool LdoPrereg_ShouldEnableDcdc(void);
bool LdoPrereg_IsPermitGranted(void);
float LdoPrereg_GetCommandV(void);
void LdoPrereg_GetStatus(LdoPrereg_Status_t *out);

#endif /* LDO_PREREG_H */
