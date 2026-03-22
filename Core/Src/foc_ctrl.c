#include "foc_ctrl.h"
#include "foc_config.h"
#include "foc_math.h"
#include "svpwm.h"
#include "mt6835.h"
#include "tim.h"
#include "adc.h"
#include "main.h"

/* Place FOC state in CCMRAM: CPU-exclusive, zero-wait-state, no DMA bus contention */
__attribute__((section(".ccmram")))
FocCtrl_t g_foc = { 0 };

/* PI gains — start very conservative, raise slowly while observing Iq on VOFA.
 * Rule of thumb: Kp ≈ L / Ts, Ki ≈ R / Ts  (Ts = 50µs @ 20kHz)
 * Without knowing exact L/R, use safe low values to avoid oscillation. */
#define PID_ID_KP   0.020f
#define PID_ID_KI   0.0004f
#define PID_IQ_KP   0.020f
#define PID_IQ_KI   0.0004f
#define PID_OUT_MAX 5.0f    /* Limit voltage output to ~17% of 29V bus */
#define CURRENT_FILTER_ALPHA 0.20f

static inline float lpf1(float prev, float sample)
{
    return prev + CURRENT_FILTER_ALPHA * (sample - prev);
}

void foc_ctrl_init(FocCtrl_t *foc)
{
    foc->mode = FOC_MODE_IDLE;
    foc->enabled = 0;
    foc->theta_elec = 0.0f;
    foc->ol_theta = 0.0f;
    foc->ol_freq_hz = OPEN_LOOP_DEFAULT_FREQ_HZ;
    foc->ol_amplitude = OPEN_LOOP_DEFAULT_AMPLITUDE;

    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->v_bus = VBUS_DEFAULT;

    foc->adc_offset_a = ADC_MID_VALUE;
    foc->adc_offset_b = ADC_MID_VALUE;
    foc->adc_offset_c = ADC_MID_VALUE;
    foc->adc_offset_bus = ADC_MID_VALUE;
    foc->raw_adc_a = ADC_MID_VALUE;
    foc->raw_adc_b = ADC_MID_VALUE;
    foc->raw_adc_c = ADC_MID_VALUE;
    foc->raw_adc_bus = ADC_MID_VALUE;
    foc->current_filter_ready = 0;

    pid_init(&foc->pid_id, PID_ID_KP, PID_ID_KI, -PID_OUT_MAX, PID_OUT_MAX);
    pid_init(&foc->pid_iq, PID_IQ_KP, PID_IQ_KI, -PID_OUT_MAX, PID_OUT_MAX);
}

void foc_ctrl_calibrate_offsets(FocCtrl_t *foc)
{
    /* Sample ADC injected channels with PWM off to get zero-current offset.
     *
     * JSQR is configured for TIM1_CH4 external trigger, but TIM1 is not
     * running yet during calibration.  Temporarily clear JEXTEN bits so
     * JADSTART triggers the conversion via software instead. */

    uint32_t sum_a = 0, sum_b = 0, sum_c = 0, sum_bus = 0;
    const uint32_t n_samples = 64;

    /* Ensure ADC is enabled */
    if (!(ADC1->CR & ADC_CR_ADEN)) {
        ADC1->ISR = ADC_ISR_ADRDY;
        ADC1->CR |= ADC_CR_ADEN;
        while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }
    }

    /* Save JSQR and switch to software trigger */
    uint32_t saved_jsqr = ADC1->JSQR;
    ADC1->JSQR = saved_jsqr & ~ADC_JSQR_JEXTEN_Msk;

    for (uint32_t i = 0; i < n_samples; i++) {
        ADC1->CR |= ADC_CR_JADSTART;
        while (!(ADC1->ISR & ADC_ISR_JEOS)) { }
        ADC1->ISR = ADC_ISR_JEOS;

        /* Schematic mapping:
         *   PA2 / ADC1_IN3 -> Ia
         *   PA1 / ADC1_IN2 -> Ib
         *   PA0 / ADC1_IN1 -> Ic
         * Injected ranks follow channel order 1,2,3 => JDR1=Ic, JDR2=Ib, JDR3=Ia.
         */
        sum_a += (ADC1->JDR3 & 0xFFFFU);
#if MOTOR_ADC_BC_SWAP
        sum_b += (ADC1->JDR1 & 0xFFFFU);
        sum_c += (ADC1->JDR2 & 0xFFFFU);
#else
        sum_b += (ADC1->JDR2 & 0xFFFFU);
        sum_c += (ADC1->JDR1 & 0xFFFFU);
#endif
        sum_bus += (ADC1->JDR4 & 0xFFFFU);
    }

    /* Restore hardware trigger */
    ADC1->JSQR = saved_jsqr;

    foc->adc_offset_a = (uint16_t)(sum_a / n_samples);
    foc->adc_offset_b = (uint16_t)(sum_b / n_samples);
    foc->adc_offset_c = (uint16_t)(sum_c / n_samples);
    foc->adc_offset_bus = (uint16_t)(sum_bus / n_samples);
}

void foc_ctrl_set_mode(FocCtrl_t *foc, FocMode_t mode)
{
    if (foc->mode != mode) {
        pid_reset(&foc->pid_id);
        pid_reset(&foc->pid_iq);
        foc->ol_theta = 0.0f;
        foc->current_filter_ready = 0;
        foc->mode = mode;
    }
}

void foc_ctrl_set_iq_ref(FocCtrl_t *foc, float iq_ref)
{
    foc->iq_ref = iq_ref;
}

void foc_ctrl_set_open_loop(FocCtrl_t *foc, float freq_hz, float amplitude)
{
    foc->ol_freq_hz = freq_hz;
    foc->ol_amplitude = amplitude;
}

/* Pre-compute the ADC-to-Amps scale factor (constant, avoids division in ISR) */
#define ADC_SCALE  (ADC_VREF / (float)ADC_RESOLUTION / (SHUNT_RESISTANCE * OPAMP_GAIN))

static inline void foc_read_currents(FocCtrl_t *foc)
{
    /* Wait for injected conversion to complete (JEOS = End Of Sequence).
     * With RCR=1 the ISR fires at valley (~25µs after ADC trigger at peak),
     * so JEOS should already be set.  The while loop is a safety net. */
    while (!(ADC1->ISR & ADC_ISR_JEOS)) {}
    ADC1->ISR = ADC_ISR_JEOS;  /* Clear flag (write-1-to-clear) */

    /* Direct register access — avoids HAL function call overhead in ISR.
     * JDRx registers hold injected conversion results. */
    /* Schematic mapping:
     *   PA2 / ADC1_IN3 -> Ia
     *   PA1 / ADC1_IN2 -> Ib
     *   PA0 / ADC1_IN1 -> Ic
     * Injected ranks follow channel order 1,2,3 => JDR1=Ic, JDR2=Ib, JDR3=Ia.
     */
    uint16_t raw_a = (uint16_t)(ADC1->JDR3 & 0xFFFFU);
#if MOTOR_ADC_BC_SWAP
    uint16_t raw_b = (uint16_t)(ADC1->JDR1 & 0xFFFFU);
    uint16_t raw_c = (uint16_t)(ADC1->JDR2 & 0xFFFFU);
#else
    uint16_t raw_b = (uint16_t)(ADC1->JDR2 & 0xFFFFU);
    uint16_t raw_c = (uint16_t)(ADC1->JDR1 & 0xFFFFU);
#endif
    uint16_t raw_bus = (uint16_t)(ADC1->JDR4 & 0xFFFFU);

    foc->raw_adc_a = raw_a;
    foc->raw_adc_b = raw_b;
    foc->raw_adc_c = raw_c;
    foc->raw_adc_bus = raw_bus;

    float ia_sample = ((float)raw_a - (float)foc->adc_offset_a) * ADC_SCALE;
    float ib_sample = ((float)raw_b - (float)foc->adc_offset_b) * ADC_SCALE;
    float ic_sample = ((float)raw_c - (float)foc->adc_offset_c) * ADC_SCALE;
    float ibus_sample = ((float)raw_bus - (float)foc->adc_offset_bus) * ADC_SCALE;

    if (!foc->current_filter_ready) {
        foc->ia = ia_sample;
        foc->ib = ib_sample;
        foc->ic = ic_sample;
        foc->ibus = ibus_sample;
        foc->current_filter_ready = 1;
    } else {
        foc->ia = lpf1(foc->ia, ia_sample);
        foc->ib = lpf1(foc->ib, ib_sample);
        foc->ic = lpf1(foc->ic, ic_sample);
        foc->ibus = lpf1(foc->ibus, ibus_sample);
    }
}

void foc_ctrl_update(FocCtrl_t *foc)
{
    if (!foc->enabled) {
        return;
    }

    float theta;

    switch (foc->mode) {

    case FOC_MODE_OPEN_LOOP:
        /* Self-incrementing angle, no encoder, no current loop */
        foc->ol_theta += FOC_2PI * foc->ol_freq_hz * OPEN_LOOP_DT;
        if (foc->ol_theta >= FOC_2PI) foc->ol_theta -= FOC_2PI;
        theta = foc->ol_theta;
        foc->theta_elec = theta;
        foc->vd = 0.0f;
        foc->vq = foc->ol_amplitude * foc->v_bus;
        /* Sample currents for monitoring (not used for control) */
        foc_read_currents(foc);
        {
            AlphaBeta_t i_ab = clarke_transform(foc->ia, foc->ib, foc->ic);
            foc->i_alpha = i_ab.alpha;
            foc->i_beta  = i_ab.beta;
            DQ_t i_dq = park_transform(foc->i_alpha, foc->i_beta, theta);
            foc->id = lpf1(foc->id, i_dq.d);
            foc->iq = lpf1(foc->iq, i_dq.q);
        }
        break;

    case FOC_MODE_VOLTAGE:
        /* Encoder angle, fixed voltage, no current loop */
        theta = mt6835_get_electrical_angle(&g_encoder);
        foc->theta_elec = theta;
        /* Also sample currents for debug (not used for control) */
        foc_read_currents(foc);
        {
            AlphaBeta_t i_ab = clarke_transform(foc->ia, foc->ib, foc->ic);
            foc->i_alpha = i_ab.alpha;
            foc->i_beta  = i_ab.beta;
            DQ_t i_dq = park_transform(foc->i_alpha, foc->i_beta, theta);
            foc->id = lpf1(foc->id, i_dq.d);
            foc->iq = lpf1(foc->iq, i_dq.q);
        }
        foc->vd = 0.0f;
        foc->vq = foc->ol_amplitude * foc->v_bus;
        break;

    case FOC_MODE_CURRENT: {
        /* Full FOC: encoder + current PI */
        theta = mt6835_get_electrical_angle(&g_encoder);
        foc->theta_elec = theta;

        /* Read phase currents from ADC */
        foc_read_currents(foc);

        /* Clarke: Ia, Ib -> Ialpha, Ibeta */
        foc->i_alpha = foc->ia;
        foc->i_beta  = (foc->ia + 2.0f * foc->ib) * FOC_SQRT3_INV;

        /* Compute sin/cos once, reuse for Park + inverse Park */
        float sin_t, cos_t;
        foc_sincos(theta, &sin_t, &cos_t);

        /* Park: Ialpha, Ibeta -> Id, Iq */
        foc->id = lpf1(foc->id,  foc->i_alpha * cos_t + foc->i_beta * sin_t);
        foc->iq = lpf1(foc->iq, -foc->i_alpha * sin_t + foc->i_beta * cos_t);

        /* PI controllers */
        foc->vd = pid_update(&foc->pid_id, foc->id_ref - foc->id);
        foc->vq = pid_update(&foc->pid_iq, foc->iq_ref - foc->iq);

        /* Inverse Park: Vd, Vq -> Valpha, Vbeta (reuse sin/cos) */
        foc->v_alpha = foc->vd * cos_t - foc->vq * sin_t;
        foc->v_beta  = foc->vd * sin_t + foc->vq * cos_t;

        /* SVPWM */
        SvpwmOutput_t pwm = svpwm_calculate(foc->v_alpha, foc->v_beta, foc->v_bus);
        svpwm_apply(&pwm);
        foc->duty_a = (float)pwm.ccr_a / (float)PWM_PERIOD;
        foc->duty_b = (float)pwm.ccr_b / (float)PWM_PERIOD;
        foc->duty_c = (float)pwm.ccr_c / (float)PWM_PERIOD;
        return;  /* skip common path below */
    }

    default:
        return;
    }

    /* Inverse Park: Vd, Vq -> Valpha, Vbeta (for non-CURRENT modes) */
    AlphaBeta_t v_ab = inv_park_transform(foc->vd, foc->vq, theta);
    foc->v_alpha = v_ab.alpha;
    foc->v_beta  = v_ab.beta;

    /* SVPWM */
    SvpwmOutput_t pwm = svpwm_calculate(foc->v_alpha, foc->v_beta, foc->v_bus);
    svpwm_apply(&pwm);
    foc->duty_a = (float)pwm.ccr_a / (float)PWM_PERIOD;
    foc->duty_b = (float)pwm.ccr_b / (float)PWM_PERIOD;
    foc->duty_c = (float)pwm.ccr_c / (float)PWM_PERIOD;
}

void foc_ctrl_start(FocCtrl_t *foc)
{
    /* Start TIM1 PWM on channels 1, 2, 3 + complementary */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* CH4 for ADC trigger — set compare value near counter peak so ADC samples
     * at PWM center (all low-side FETs ON, best current measurement point).
     * Period = 4000, so CCR4 = Period - 1 triggers on the falling edge at peak. */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, htim1.Init.Period - 325);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Start ADC injected conversion (triggered by TIM1_CH4) */
    HAL_ADCEx_InjectedStart(&hadc1);

    /* Start TIM1 update interrupt */
    HAL_TIM_Base_Start_IT(&htim1);

    /* Enable relay */
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);

    foc->enabled = 1;
}

void foc_ctrl_stop(FocCtrl_t *foc)
{
    foc->enabled = 0;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);

    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim1);

    pid_reset(&foc->pid_id);
    pid_reset(&foc->pid_iq);
}
