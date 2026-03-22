#include "svpwm.h"
#include "foc_config.h"
#include "tim.h"
#include <stdint.h>

/*
 * SVPWM implementation using min-max zero-sequence injection.
 *
 * This method is mathematically equivalent to the classic 7-segment
 * sector-based SVPWM, but simpler and more robust to implement.
 *
 * Steps:
 *   1. Inverse Clarke: Valpha, Vbeta -> Va, Vb, Vc
 *   2. Zero-sequence injection: Vn = -(Vmax + Vmin) / 2
 *   3. Va += Vn, Vb += Vn, Vc += Vn
 *   4. Normalize to [0, PWM_PERIOD] -> CCR values
 */

static float min3f(float a, float b, float c)
{
    float m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    return m;
}

static float max3f(float a, float b, float c)
{
    float m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    return m;
}

static uint16_t clamp_ccr(float val)
{
    if (val < 0.0f) return 0U;
    if (val > (float)PWM_PERIOD) return (uint16_t)PWM_PERIOD;
    return (uint16_t)val;
}

SvpwmOutput_t svpwm_calculate(float v_alpha, float v_beta, float v_bus)
{
    SvpwmOutput_t out;

    if (v_bus < 1.0f) {
        out.ccr_a = PWM_PERIOD / 2;
        out.ccr_b = PWM_PERIOD / 2;
        out.ccr_c = PWM_PERIOD / 2;
        out.sector = 0;
        return out;
    }

    /* Inverse Clarke transform: alpha-beta -> abc */
    float va = v_alpha;
    float vb = -0.5f * v_alpha + FOC_SQRT3 * 0.5f * v_beta;
    float vc = -0.5f * v_alpha - FOC_SQRT3 * 0.5f * v_beta;

    /* Zero-sequence injection (SVPWM equivalent) */
    float vmin = min3f(va, vb, vc);
    float vmax = max3f(va, vb, vc);
    float vneutral = -0.5f * (vmax + vmin);

    va += vneutral;
    vb += vneutral;
    vc += vneutral;

    /* Determine sector for diagnostics */
    uint8_t n = 0;
    if (v_beta > 0.0f) n += 1;
    if ((FOC_SQRT3 * v_alpha - v_beta) > 0.0f) n += 2;
    if ((-FOC_SQRT3 * v_alpha - v_beta) > 0.0f) n += 4;
    {
        static const uint8_t smap[8] = { 0, 2, 6, 1, 4, 3, 5, 0 };
        out.sector = smap[n];
    }

    /* Normalize to duty cycle [0, PWM_PERIOD]
     * va is in range [-Vbus/2, Vbus/2] after injection
     * duty = (va / Vbus + 0.5) * PWM_PERIOD
     */
    float scale = (float)PWM_PERIOD / v_bus;
    out.ccr_a = clamp_ccr((va * scale) + (float)PWM_PERIOD * 0.5f);
    out.ccr_b = clamp_ccr((vb * scale) + (float)PWM_PERIOD * 0.5f);
    out.ccr_c = clamp_ccr((vc * scale) + (float)PWM_PERIOD * 0.5f);

    return out;
}

void svpwm_apply(const SvpwmOutput_t *output)
{
#if MOTOR_PWM_BC_SWAP
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output->ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, output->ccr_c);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, output->ccr_b);
#else
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output->ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, output->ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, output->ccr_c);
#endif
}
