#include "pid.h"

void pid_init(PID_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_reset(PID_t *pid)
{
    pid->integral = 0.0f;
}

float pid_update(PID_t *pid, float error)
{
    /* Proportional */
    float output = pid->kp * error;

    /* Integral with anti-windup clamping */
    pid->integral += pid->ki * error;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    output += pid->integral;

    /* Output clamp */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
