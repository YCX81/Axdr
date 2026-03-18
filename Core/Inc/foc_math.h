#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include <stdint.h>

typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

typedef struct {
    float d;
    float q;
} DQ_t;

/* Fast sin/cos using 256-point LUT with linear interpolation */
float foc_sin(float angle_rad);
float foc_cos(float angle_rad);

/* Compute sin and cos simultaneously (faster than calling separately) */
void foc_sincos(float angle_rad, float *sin_out, float *cos_out);

/* Clarke transform: Ia, Ib, Ic -> Ialpha, Ibeta (equal amplitude) */
AlphaBeta_t clarke_transform(float ia, float ib, float ic);

/* Park transform: Ialpha, Ibeta, theta -> Id, Iq */
DQ_t park_transform(float i_alpha, float i_beta, float theta);

/* Inverse Park transform: Vd, Vq, theta -> Valpha, Vbeta */
AlphaBeta_t inv_park_transform(float vd, float vq, float theta);

#endif /* __FOC_MATH_H */
