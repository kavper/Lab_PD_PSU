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

    if (duty_max < duty_min) {
        float swap = duty_max;
        duty_max = duty_min;
        duty_min = swap;
    }

    ctrl->kp = kp;
    ctrl->ki = ki;
    ctrl->duty_min = duty_min;
    ctrl->duty_max = duty_max;
    ctrl->integrator = duty_min;
    ctrl->target_setpoint = 0.0f;
    ctrl->ramped_setpoint = 0.0f;
    ctrl->slew_up_v_per_s = (softstart_slew_v_per_s > 0.0f) ? softstart_slew_v_per_s : 0.0f;
    ctrl->slew_down_v_per_s = ctrl->slew_up_v_per_s;
    ctrl->pending_integrator = ctrl->integrator;
    ctrl->pending_error = 0.0f;
    ctrl->initialized = true;
}

void ControlCv_SetSlewRates(ControlCv_t *ctrl,
                            float slew_up_v_per_s,
                            float slew_down_v_per_s)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    ctrl->slew_up_v_per_s = (slew_up_v_per_s > 0.0f) ? slew_up_v_per_s : 0.0f;
    ctrl->slew_down_v_per_s = (slew_down_v_per_s > 0.0f) ? slew_down_v_per_s : 0.0f;
}

void ControlCv_Reset(ControlCv_t *ctrl, float duty_init)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    duty_init = ControlCv_Clamp(duty_init, ctrl->duty_min, ctrl->duty_max);
    ctrl->integrator = duty_init;
    ctrl->pending_integrator = duty_init;
    ctrl->pending_error = 0.0f;
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

void ControlCv_UpdateRampedSetpoint(ControlCv_t *ctrl, float dt_s)
{
    float up_step;
    float down_step;

    if ((ctrl == 0) || (!ctrl->initialized) || (dt_s <= 0.0f)) {
        return;
    }

    up_step = ctrl->slew_up_v_per_s * dt_s;
    down_step = ctrl->slew_down_v_per_s * dt_s;

    if (ctrl->ramped_setpoint < ctrl->target_setpoint) {
        ctrl->ramped_setpoint += up_step;
        if (ctrl->ramped_setpoint > ctrl->target_setpoint) {
            ctrl->ramped_setpoint = ctrl->target_setpoint;
        }
    } else if (ctrl->ramped_setpoint > ctrl->target_setpoint) {
        ctrl->ramped_setpoint -= down_step;
        if (ctrl->ramped_setpoint < ctrl->target_setpoint) {
            ctrl->ramped_setpoint = ctrl->target_setpoint;
        }
    }
}

float ControlCv_RunPreClamp(ControlCv_t *ctrl, float vout_v, float dt_s)
{
    float error;
    float proportional;

    if ((ctrl == 0) || (!ctrl->initialized) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    error = ctrl->ramped_setpoint - vout_v;
    proportional = ctrl->kp * error;
    ctrl->pending_integrator = ctrl->integrator + (ctrl->ki * error * dt_s);
    ctrl->pending_error = error;

    return proportional + ctrl->pending_integrator;
}

void ControlCv_CommitIntegrator(ControlCv_t *ctrl,
                                bool saturated_high,
                                bool saturated_low)
{
    bool pushing_high;
    bool pushing_low;

    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    pushing_high = saturated_high && (ctrl->pending_error > 0.0f);
    pushing_low = saturated_low && (ctrl->pending_error < 0.0f);

    if ((!pushing_high) && (!pushing_low)) {
        ctrl->integrator = ctrl->pending_integrator;
    }

    ctrl->integrator = ControlCv_Clamp(ctrl->integrator,
                                       ctrl->duty_min,
                                       ctrl->duty_max);
}

float ControlCv_Run(ControlCv_t *ctrl, float vout_v, float dt_s)
{
    float duty;
    bool sat_hi = false;
    bool sat_lo = false;

    if ((ctrl == 0) || (!ctrl->initialized) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    ControlCv_UpdateRampedSetpoint(ctrl, dt_s);
    duty = ControlCv_RunPreClamp(ctrl, vout_v, dt_s);

    if (duty >= ctrl->duty_max) {
        duty = ctrl->duty_max;
        sat_hi = true;
    } else if (duty <= ctrl->duty_min) {
        duty = ctrl->duty_min;
        sat_lo = true;
    }

    ControlCv_CommitIntegrator(ctrl, sat_hi, sat_lo);

    return duty;
}

void Control2p2z_Init(Control2p2z_t *ctrl,
                      float b0,
                      float b1,
                      float b2,
                      float a1,
                      float a2)
{
    if (ctrl == 0) {
        return;
    }

    ctrl->b0 = b0;
    ctrl->b1 = b1;
    ctrl->b2 = b2;
    ctrl->a1 = a1;
    ctrl->a2 = a2;
    ctrl->s1 = 0.0f;
    ctrl->s2 = 0.0f;
    ctrl->pending_s1 = 0.0f;
    ctrl->pending_s2 = 0.0f;
    ctrl->last_input = 0.0f;
    ctrl->initialized = true;
}

void Control2p2z_Reset(Control2p2z_t *ctrl, float state)
{
    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    ctrl->s1 = state;
    ctrl->s2 = state;
    ctrl->pending_s1 = state;
    ctrl->pending_s2 = state;
    ctrl->last_input = 0.0f;
}

float Control2p2z_RunImmediate(Control2p2z_t *ctrl, float input)
{
    float output;

    if ((ctrl == 0) || (!ctrl->initialized)) {
        return 0.0f;
    }

    ctrl->last_input = input;

    /* Direct Form II (transposed) z odroczonym commit. */
    output = (ctrl->b0 * input) + ctrl->s1;
    ctrl->pending_s1 = (ctrl->b1 * input) - (ctrl->a1 * output) + ctrl->s2;
    ctrl->pending_s2 = (ctrl->b2 * input) - (ctrl->a2 * output);

    return output;
}

void Control2p2z_Commit(Control2p2z_t *ctrl,
                        bool saturated_high,
                        bool saturated_low)
{
    bool pushing_high;
    bool pushing_low;

    if ((ctrl == 0) || (!ctrl->initialized)) {
        return;
    }

    pushing_high = saturated_high && (ctrl->last_input > 0.0f);
    pushing_low = saturated_low && (ctrl->last_input < 0.0f);

    if ((!pushing_high) && (!pushing_low)) {
        ctrl->s1 = ctrl->pending_s1;
        ctrl->s2 = ctrl->pending_s2;
    }
}
