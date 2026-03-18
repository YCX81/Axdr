#ifndef __PID_H
#define __PID_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_max;
    float out_min;
} PID_t;

/* Initialize PI controller */
void pid_init(PID_t *pid, float kp, float ki, float out_min, float out_max);

/* Reset integral term */
void pid_reset(PID_t *pid);

/* Compute PI output. Call at fixed rate (dt baked into ki). */
float pid_update(PID_t *pid, float error);

#endif /* __PID_H */
