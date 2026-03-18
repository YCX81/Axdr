#ifndef __OPEN_LOOP_H
#define __OPEN_LOOP_H

#include <stdint.h>

typedef struct {
    float theta;            /* Current electrical angle [0, 2*PI) */
    float omega;            /* Angular velocity (rad/s) */
    float v_amplitude;      /* Output voltage amplitude ratio (0~1, fraction of Vbus) */
    float target_freq_hz;   /* Target electrical frequency (Hz) */
    uint8_t enabled;        /* Enable flag */
} OpenLoopCtrl_t;

/* Initialize open-loop controller with frequency and amplitude */
void open_loop_init(OpenLoopCtrl_t *ctrl, float freq_hz, float amplitude);

/* Set new frequency (can be called at runtime) */
void open_loop_set_freq(OpenLoopCtrl_t *ctrl, float freq_hz);

/* Set new amplitude (can be called at runtime) */
void open_loop_set_amplitude(OpenLoopCtrl_t *ctrl, float amplitude);

/*
 * Called every PWM cycle (in TIM1 update ISR).
 * Advances angle, computes SVPWM, and applies to TIM1.
 *   v_bus: measured or default bus voltage
 *   dt: time step (1/PWM_FREQ_HZ)
 */
void open_loop_update(OpenLoopCtrl_t *ctrl, float v_bus, float dt);

/* Start/stop PWM output */
void open_loop_start(void);
void open_loop_stop(void);

/* Global instance (declared in open_loop.c) */
extern OpenLoopCtrl_t g_open_loop;

#endif /* __OPEN_LOOP_H */
