#include "control_cv.h"

static float ControlCv_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void ControlCv_Init(ControlCv_t *ctrl,
                    float kp,
                    float ki,
                    float duty_min,
                    float duty_max,
                    float softstart_slew_v_per_s)
{
    if (ctrl == 0) {
        return;
    }

    ctrl->kp = kp;
    ctrl->ki = ki;
    ctrl->duty_min = duty_min;
    ctrl->duty_max = duty_max;
    ctrl->integrator = duty_min;
    ctrl->target_setpoint = 0.0f;
    ctrl->ramped_setpoint = 0.0f;
    ctrl->softstart_slew_v_per_s = softstart_slew_v_per_s;
    ctrl->initialized = true;
}

void ControlCv_Reset(ControlCv_t *ctrl, float duty_init)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    duty_init = ControlCv_Clamp(duty_init, ctrl->duty_min, ctrl->duty_max);
    ctrl->integrator = duty_init;
}

void ControlCv_SetTarget(ControlCv_t *ctrl, float setpoint_v)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    if (setpoint_v < 0.0f) {
        setpoint_v = 0.0f;
    }

    ctrl->target_setpoint = setpoint_v;
}

float ControlCv_GetRampedSetpoint(const ControlCv_t *ctrl)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return 0.0f;
    }

    return ctrl->ramped_setpoint;
}

float ControlCv_Run(ControlCv_t *ctrl, float vout_v, float dt_s)
{
    float slope_step;
    float error;
    float proportional;
    float integrator_candidate;
    float output;

    if ((ctrl == 0) || (!ctrl->initialized) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    slope_step = ctrl->softstart_slew_v_per_s * dt_s;
    if (ctrl->ramped_setpoint < ctrl->target_setpoint) {
        ctrl->ramped_setpoint += slope_step;
        if (ctrl->ramped_setpoint > ctrl->target_setpoint) {
            ctrl->ramped_setpoint = ctrl->target_setpoint;
        }
    } else if (ctrl->ramped_setpoint > ctrl->target_setpoint) {
        ctrl->ramped_setpoint = ctrl->target_setpoint;
    }

    error = ctrl->ramped_setpoint - vout_v;
    proportional = ctrl->kp * error;
    integrator_candidate = ctrl->integrator + (ctrl->ki * error * dt_s);
    output = proportional + integrator_candidate;

    if (output > ctrl->duty_max) {
        output = ctrl->duty_max;
        if (error < 0.0f) {
            ctrl->integrator = integrator_candidate;
        }
    } else if (output < ctrl->duty_min) {
        output = ctrl->duty_min;
        if (error > 0.0f) {
            ctrl->integrator = integrator_candidate;
        }
    } else {
        ctrl->integrator = integrator_candidate;
    }

    ctrl->integrator = ControlCv_Clamp(ctrl->integrator,
                                       ctrl->duty_min,
                                       ctrl->duty_max);

    return output;
}
