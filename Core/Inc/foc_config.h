#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

/* ---- Timer / PWM ---- */
#define PWM_PERIOD          4000U       /* TIM1 ARR (center-aligned) */
#define PWM_FREQ_HZ         20000U      /* 160MHz / (2*4000) = 20kHz */
#define DEAD_TIME_REG       16U         /* 160MHz * 100ns = 16 counts */
#define SYSTEM_CLOCK_HZ     160000000U

/* ---- ADC ---- */
#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4096
#define ADC_MID_VALUE       2048        /* VREF bias = VCC/2 */

/* ---- Current Sensing ---- */
#define SHUNT_RESISTANCE    0.001f      /* 1mOhm */
#define OPAMP_GAIN          20.0f       /* Rf=20k, R=1k */

/* ---- Voltage Sensing ---- */
#define VBUS_DIVIDER_RATIO  21.0f       /* (10k+10k+1k)/1k */
#define VBUS_DEFAULT        29.1f       /* Default bus voltage for open-loop */

/* ---- Math Constants ---- */
#define FOC_PI              3.14159265f
#define FOC_2PI             6.28318530f
#define FOC_SQRT3           1.73205080f
#define FOC_SQRT3_INV       0.57735026f /* 1/sqrt(3) */
#define FOC_2_BY_SQRT3      1.15470053f /* 2/sqrt(3) */

/* ---- Open Loop Defaults ---- */
#define OPEN_LOOP_DEFAULT_FREQ_HZ   10.0f
#define OPEN_LOOP_DEFAULT_AMPLITUDE 0.10f   /* 10% of Vbus */
#define OPEN_LOOP_DT                (1.0f / (float)PWM_FREQ_HZ)

#endif /* __FOC_CONFIG_H */
