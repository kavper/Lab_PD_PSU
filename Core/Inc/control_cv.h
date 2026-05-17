#ifndef CONTROL_CV_H
#define CONTROL_CV_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float duty_min;
    float duty_max;
    float integrator;
    float target_setpoint;
    float ramped_setpoint;
    float softstart_slew_v_per_s;
    bool initialized;
} ControlCv_t;

void ControlCv_Init(ControlCv_t *ctrl,
                    float kp,
                    float ki,
                    float duty_min,
                    float duty_max,
                    float softstart_slew_v_per_s);

void ControlCv_Reset(ControlCv_t *ctrl, float duty_init);
void ControlCv_SetTarget(ControlCv_t *ctrl, float setpoint_v);
float ControlCv_GetRampedSetpoint(const ControlCv_t *ctrl);
float ControlCv_Run(ControlCv_t *ctrl, float vout_v, float dt_s);

#endif
