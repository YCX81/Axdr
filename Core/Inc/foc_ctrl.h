#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H

#include <stdint.h>
#include "pid.h"

typedef enum {
    FOC_MODE_IDLE = 0,
    FOC_MODE_OPEN_LOOP,     /* Open-loop V/F (encoder not used) */
    FOC_MODE_VOLTAGE,       /* Encoder angle + fixed Vd/Vq (no current loop) */
    FOC_MODE_CURRENT,       /* Full FOC: encoder + current PI loops */
} FocMode_t;

typedef struct {
    FocMode_t mode;

    /* Encoder angle (updated from main loop cache) */
    float theta_elec;       /* Electrical angle from encoder [rad] */

    /* Open-loop state */
    float ol_theta;         /* Self-incrementing angle for open-loop */
    float ol_freq_hz;       /* Open-loop frequency */
    float ol_amplitude;     /* Open-loop voltage amplitude (0~1) */

    /* Current feedback */
    float ia, ib, ic;       /* Phase currents [A] */
    float i_alpha, i_beta;  /* Clarke output */
    float id, iq;           /* Park output */

    /* Current reference */
    float id_ref;           /* D-axis current ref (normally 0) */
    float iq_ref;           /* Q-axis current ref (torque command) */

    /* Voltage output */
    float vd, vq;           /* PI output or manual voltage */
    float v_alpha, v_beta;  /* Inverse Park output */

    /* PI controllers */
    PID_t pid_id;
    PID_t pid_iq;

    /* ADC zero-current offsets (raw ADC counts at 0A) */
    uint16_t adc_offset_a;
    uint16_t adc_offset_b;
    uint16_t adc_offset_c;

    /* Bus voltage */
    float v_bus;

    uint8_t enabled;
} FocCtrl_t;

/* Initialize FOC controller */
void foc_ctrl_init(FocCtrl_t *foc);

/* Calibrate ADC zero-current offsets (call before enabling PWM) */
void foc_ctrl_calibrate_offsets(FocCtrl_t *foc);

/* Set control mode */
void foc_ctrl_set_mode(FocCtrl_t *foc, FocMode_t mode);

/* Set torque reference (for CURRENT mode) */
void foc_ctrl_set_iq_ref(FocCtrl_t *foc, float iq_ref);

/* Set open-loop parameters */
void foc_ctrl_set_open_loop(FocCtrl_t *foc, float freq_hz, float amplitude);

/* Main control loop - call from TIM1 update ISR at 20kHz */
void foc_ctrl_update(FocCtrl_t *foc);

/* Start/stop PWM outputs */
void foc_ctrl_start(FocCtrl_t *foc);
void foc_ctrl_stop(FocCtrl_t *foc);

/* Global instance */
extern FocCtrl_t g_foc;

#endif /* __FOC_CTRL_H */
