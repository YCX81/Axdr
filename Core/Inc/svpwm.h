#ifndef __SVPWM_H
#define __SVPWM_H

#include <stdint.h>

typedef struct {
    uint16_t ccr_a;     /* TIM1->CCR1 (Phase A) */
    uint16_t ccr_b;     /* TIM1->CCR2 (Phase B) */
    uint16_t ccr_c;     /* TIM1->CCR3 (Phase C) */
    uint8_t  sector;    /* Current sector 1~6 */
} SvpwmOutput_t;

/*
 * Calculate SVPWM duty cycles from alpha-beta voltage reference.
 *   v_alpha, v_beta: voltage in alpha-beta frame (volts)
 *   v_bus: DC bus voltage (volts)
 * Returns CCR values for TIM1 CH1/CH2/CH3.
 */
SvpwmOutput_t svpwm_calculate(float v_alpha, float v_beta, float v_bus);

/* Write CCR values to TIM1 registers */
void svpwm_apply(const SvpwmOutput_t *output);

#endif /* __SVPWM_H */
