#include "open_loop.h"
#include "foc_config.h"
#include "foc_math.h"
#include "svpwm.h"
#include "tim.h"
#include "main.h"

/* Global open-loop controller instance */
OpenLoopCtrl_t g_open_loop = { 0 };

void open_loop_init(OpenLoopCtrl_t *ctrl, float freq_hz, float amplitude)
{
    ctrl->theta = 0.0f;
    ctrl->target_freq_hz = freq_hz;
    ctrl->omega = FOC_2PI * freq_hz;
    ctrl->v_amplitude = amplitude;
    ctrl->enabled = 0;
}

void open_loop_set_freq(OpenLoopCtrl_t *ctrl, float freq_hz)
{
    ctrl->target_freq_hz = freq_hz;
    ctrl->omega = FOC_2PI * freq_hz;
}

void open_loop_set_amplitude(OpenLoopCtrl_t *ctrl, float amplitude)
{
    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 0.95f) amplitude = 0.95f;
    ctrl->v_amplitude = amplitude;
}

void open_loop_update(OpenLoopCtrl_t *ctrl, float v_bus, float dt)
{
    if (!ctrl->enabled) {
        return;
    }

    /* Advance electrical angle */
    ctrl->theta += ctrl->omega * dt;
    if (ctrl->theta >= FOC_2PI) {
        ctrl->theta -= FOC_2PI;
    }

    /* Open-loop: set Vd=0, Vq=amplitude*Vbus, then inverse Park */
    float vq = ctrl->v_amplitude * v_bus;
    AlphaBeta_t v_ab = inv_park_transform(0.0f, vq, ctrl->theta);

    /* SVPWM */
    SvpwmOutput_t pwm = svpwm_calculate(v_ab.alpha, v_ab.beta, v_bus);
    svpwm_apply(&pwm);
}

void open_loop_start(void)
{
    /* Start TIM1 PWM on channels 1, 2, 3 (main outputs) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* Start complementary outputs (CH1N, CH2N, CH3N) */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* Start CH4 for ADC trigger (no GPIO output) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Start TIM1 update interrupt */
    HAL_TIM_Base_Start_IT(&htim1);

    /* Enable relay */
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);

    g_open_loop.enabled = 1;
}

void open_loop_stop(void)
{
    g_open_loop.enabled = 0;

    /* Set duty to 50% (all low side off in complementary mode) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* Disable relay */
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET);

    /* Stop PWM outputs */
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Stop_IT(&htim1);
}
