#include "foc_math.h"
#include "foc_config.h"

/* 256-point sine lookup table, placed in CCMRAM for zero-wait-state access.
 * CPU-exclusive memory, no DMA bus contention. */
__attribute__((section(".ccmram")))
static const float sin_lut[256] = {
     0.000000f,  0.024541f,  0.049068f,  0.073565f,  0.098017f,  0.122411f,  0.146730f,  0.170962f,
     0.195090f,  0.219101f,  0.242980f,  0.266713f,  0.290285f,  0.313682f,  0.336890f,  0.359895f,
     0.382683f,  0.405241f,  0.427555f,  0.449611f,  0.471397f,  0.492898f,  0.514103f,  0.534998f,
     0.555570f,  0.575808f,  0.595699f,  0.615232f,  0.634393f,  0.653173f,  0.671559f,  0.689541f,
     0.707107f,  0.724247f,  0.740951f,  0.757209f,  0.773010f,  0.788346f,  0.803208f,  0.817585f,
     0.831470f,  0.844854f,  0.857729f,  0.870087f,  0.881921f,  0.893224f,  0.903989f,  0.914210f,
     0.923880f,  0.932993f,  0.941544f,  0.949528f,  0.956940f,  0.963776f,  0.970031f,  0.975702f,
     0.980785f,  0.985278f,  0.989177f,  0.992480f,  0.995185f,  0.997290f,  0.998795f,  0.999699f,
     1.000000f,  0.999699f,  0.998795f,  0.997290f,  0.995185f,  0.992480f,  0.989177f,  0.985278f,
     0.980785f,  0.975702f,  0.970031f,  0.963776f,  0.956940f,  0.949528f,  0.941544f,  0.932993f,
     0.923880f,  0.914210f,  0.903989f,  0.893224f,  0.881921f,  0.870087f,  0.857729f,  0.844854f,
     0.831470f,  0.817585f,  0.803208f,  0.788346f,  0.773010f,  0.757209f,  0.740951f,  0.724247f,
     0.707107f,  0.689541f,  0.671559f,  0.653173f,  0.634393f,  0.615232f,  0.595699f,  0.575808f,
     0.555570f,  0.534998f,  0.514103f,  0.492898f,  0.471397f,  0.449611f,  0.427555f,  0.405241f,
     0.382683f,  0.359895f,  0.336890f,  0.313682f,  0.290285f,  0.266713f,  0.242980f,  0.219101f,
     0.195090f,  0.170962f,  0.146730f,  0.122411f,  0.098017f,  0.073565f,  0.049068f,  0.024541f,
     0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
    -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
    -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
    -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914210f,
    -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
    -0.980785f, -0.985278f, -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f, -0.989177f, -0.985278f,
    -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
    -0.923880f, -0.914210f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
    -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615232f, -0.595699f, -0.575808f,
    -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
    -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f,
};

#define LUT_SIZE      256
#define LUT_SIZE_F    256.0f
#define LUT_MASK      0xFFU
#define RAD_TO_IDX    (LUT_SIZE_F / FOC_2PI)   /* 256 / 2PI ≈ 40.74 */
#define COS_OFFSET    64U                       /* 256/4 = 90 degrees */

/* Internal: single LUT lookup with linear interpolation, no angle wrapping needed.
 * idx_f must be in [0, 256) range. */
static inline float lut_interp(float idx_f)
{
    uint32_t idx = (uint32_t)idx_f;
    float frac = idx_f - (float)idx;
    uint32_t idx_next = (idx + 1U) & LUT_MASK;
    return sin_lut[idx] + frac * (sin_lut[idx_next] - sin_lut[idx]);
}

/*
 * Compute sin and cos simultaneously from a single angle.
 * Avoids redundant angle wrapping and uses index offset for cos.
 * Input: angle in radians [any range]
 */
void foc_sincos(float angle_rad, float *sin_out, float *cos_out)
{
    /* Fast angle wrap using multiply + floor, no while loops.
     * Works for any input range including negative angles. */
    float normalized = angle_rad * (1.0f / FOC_2PI);
    normalized -= (float)(int32_t)normalized;
    if (normalized < 0.0f) normalized += 1.0f;

    float idx_f = normalized * LUT_SIZE_F;    /* [0, 256) */

    *sin_out = lut_interp(idx_f);

    /* cos = sin(angle + 90°) = LUT offset by 64 entries */
    float idx_cos = idx_f + (float)COS_OFFSET;
    if (idx_cos >= LUT_SIZE_F) idx_cos -= LUT_SIZE_F;
    *cos_out = lut_interp(idx_cos);
}

/* Legacy API — still available but prefer foc_sincos for paired calls */
float foc_sin(float angle_rad)
{
    float s, c;
    foc_sincos(angle_rad, &s, &c);
    return s;
}

float foc_cos(float angle_rad)
{
    float s, c;
    foc_sincos(angle_rad, &s, &c);
    return c;
}

/*
 * Clarke transform (equal amplitude, 3-phase to alpha-beta)
 *   Ialpha = Ia
 *   Ibeta  = (Ia + 2*Ib) / sqrt(3)
 */
inline AlphaBeta_t clarke_transform(float ia, float ib, float ic)
{
    (void)ic;
    AlphaBeta_t result;
    result.alpha = ia;
    result.beta  = (ia + 2.0f * ib) * FOC_SQRT3_INV;
    return result;
}

/*
 * Park transform (alpha-beta to dq)
 *   Id =  Ialpha * cos(theta) + Ibeta * sin(theta)
 *   Iq = -Ialpha * sin(theta) + Ibeta * cos(theta)
 */
inline DQ_t park_transform(float i_alpha, float i_beta, float theta)
{
    float sin_t, cos_t;
    foc_sincos(theta, &sin_t, &cos_t);
    DQ_t result;
    result.d =  i_alpha * cos_t + i_beta * sin_t;
    result.q = -i_alpha * sin_t + i_beta * cos_t;
    return result;
}

/*
 * Inverse Park transform (dq to alpha-beta)
 *   Valpha = Vd * cos(theta) - Vq * sin(theta)
 *   Vbeta  = Vd * sin(theta) + Vq * cos(theta)
 */
inline AlphaBeta_t inv_park_transform(float vd, float vq, float theta)
{
    float sin_t, cos_t;
    foc_sincos(theta, &sin_t, &cos_t);
    AlphaBeta_t result;
    result.alpha = vd * cos_t - vq * sin_t;
    result.beta  = vd * sin_t + vq * cos_t;
    return result;
}
