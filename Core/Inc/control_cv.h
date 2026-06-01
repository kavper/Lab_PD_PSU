#ifndef CONTROL_CV_H
#define CONTROL_CV_H

#include <stdbool.h>

#ifndef CONTROL_CV_USE_2P2Z
#define CONTROL_CV_USE_2P2Z 0
#endif

typedef struct {
    float kp;
    float ki;
    float duty_min;
    float duty_max;
    float integrator;
    float target_setpoint;
    float ramped_setpoint;
    float slew_up_v_per_s;
    float slew_down_v_per_s;
    float pending_integrator;
    float pending_error;
    bool initialized;
} ControlCv_t;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float s1;
    float s2;
    float pending_s1;
    float pending_s2;
    float last_input;
    bool initialized;
} Control2p2z_t;

void ControlCv_Init(ControlCv_t *ctrl,
                    float kp,
                    float ki,
                    float duty_min,
                    float duty_max,
                    float softstart_slew_v_per_s);

void ControlCv_SetSlewRates(ControlCv_t *ctrl,
                            float slew_up_v_per_s,
                            float slew_down_v_per_s);

void ControlCv_Reset(ControlCv_t *ctrl, float duty_init);
void ControlCv_SetTarget(ControlCv_t *ctrl, float setpoint_v);
float ControlCv_GetRampedSetpoint(const ControlCv_t *ctrl);
void ControlCv_UpdateRampedSetpoint(ControlCv_t *ctrl, float dt_s);
float ControlCv_RunPreClamp(ControlCv_t *ctrl, float vout_v, float dt_s);
void ControlCv_CommitIntegrator(ControlCv_t *ctrl,
                                bool saturated_high,
                                bool saturated_low);
float ControlCv_Run(ControlCv_t *ctrl, float vout_v, float dt_s);

void Control2p2z_Init(Control2p2z_t *ctrl,
                      float b0,
                      float b1,
                      float b2,
                      float a1,
                      float a2);
void Control2p2z_Reset(Control2p2z_t *ctrl, float state);
float Control2p2z_RunImmediate(Control2p2z_t *ctrl, float input);
void Control2p2z_Commit(Control2p2z_t *ctrl,
                        bool saturated_high,
                        bool saturated_low);

#endif
