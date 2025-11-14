/**
  ******************************************************************************
  * @file    cordic_math.c
  * @brief   Hardware CORDIC accelerated math functions implementation
  ******************************************************************************
  */

#include "cordic_math.h"

/* CORDIC Q1.31 fixed-point constants */
#define Q31_MAX         2147483647.0f     /* 2^31 - 1 */
#define Q31_MIN        -2147483648.0f     /* -2^31 */
#define Q31_SCALE       2147483648.0f     /* 2^31 for normalization */
#define PI_FLOAT        3.14159265358979323846f

/* Inline conversion helpers */
static inline int32_t float_to_q31(float x) {
    if (x >= 1.0f) return (int32_t)Q31_MAX;
    if (x <= -1.0f) return (int32_t)Q31_MIN;
    return (int32_t)(x * Q31_SCALE);
}

static inline float q31_to_float(int32_t q) {
    return (float)q / Q31_SCALE;
}

/**
 * @brief Initialize CORDIC math module
 */
void CORDIC_Math_Init(void) {
    /* CORDIC peripheral is already initialized by MX_CORDIC_Init() */
    /* This function reserved for future initialization if needed */
}

/**
 * @brief Compute atan2(y, x) using CORDIC PHASE function
 * @note Returns angle in radians [-π, π]
 * @note CORDIC PHASE also outputs modulus (magnitude), but we only use angle
 */
float CORDIC_Atan2(float y, float x) {
    /* Handle special cases */
    if (x == 0.0f && y == 0.0f) return 0.0f;

    /* Normalize to [-1, 1] range for CORDIC */
    float magnitude = sqrtf(x*x + y*y);
    if (magnitude == 0.0f) return 0.0f;

    float x_norm = x / magnitude;
    float y_norm = y / magnitude;

    /* Convert to Q1.31 format */
    int32_t x_q31 = float_to_q31(x_norm);
    int32_t y_q31 = float_to_q31(y_norm);

    /* Configure CORDIC for PHASE function (atan2 + modulus) */
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_PHASE,        /* Phase = atan2 */
                     CORDIC_PRECISION_DEFAULT,        /* 15 cycles for max precision */
                     LL_CORDIC_SCALE_0,               /* No scaling needed */
                     LL_CORDIC_NBWRITE_2,             /* 2 writes (x, y) */
                     LL_CORDIC_NBREAD_2,              /* 2 reads (modulus, phase) */
                     LL_CORDIC_INSIZE_32BITS,         /* 32-bit input */
                     LL_CORDIC_OUTSIZE_32BITS);       /* 32-bit output */

    /* Write input data: X then Y */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)y_q31);

    /* Read results: first is modulus (discard), second is phase */
    (void)LL_CORDIC_ReadData(CORDIC);  /* Discard modulus */
    int32_t phase_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    /* Convert from Q1.31 to radians
     * CORDIC returns angle/π in Q1.31, so multiply by π */
    float angle = q31_to_float(phase_q31) * PI_FLOAT;

    return angle;
}

/**
 * @brief Compute square root using CORDIC
 * @note Input must be normalized to [0, 1] range
 */
float CORDIC_Sqrt(float x) {
    /* Handle edge cases */
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return sqrtf(x);  /* Fall back to standard sqrt for x > 1 */

    /* For CORDIC sqrt, input must be in range [0, 1] */
    /* We handle this by scaling if needed */
    int scale_factor = 0;
    float x_scaled = x;

    /* Scale input to [0.25, 1] range for best precision */
    while (x_scaled < 0.25f && scale_factor < 16) {
        x_scaled *= 4.0f;  /* Multiply by 2^2 */
        scale_factor += 2;
    }

    /* Convert to Q1.31 */
    int32_t x_q31 = float_to_q31(x_scaled);

    /* Configure CORDIC for SQRT function */
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_SQUAREROOT,
                     CORDIC_PRECISION_DEFAULT,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_1,             /* 1 write (x) */
                     LL_CORDIC_NBREAD_1,              /* 1 read (sqrt) */
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);

    /* Write input data */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);

    /* Read result */
    int32_t sqrt_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    /* Convert back to float */
    float result = q31_to_float(sqrt_q31);

    /* Unscale result: if we scaled input by 2^(2n), scale output by 2^n */
    if (scale_factor > 0) {
        result /= (float)(1 << (scale_factor / 2));
    }

    return result;
}

/**
 * @brief Compute sine and cosine simultaneously using CORDIC
 * @note Angle in radians, converted internally to angle/π for CORDIC
 */
void CORDIC_SinCos(float angle, float *sin_out, float *cos_out) {
    /* Normalize angle to [-π, π] */
    while (angle > PI_FLOAT) angle -= 2.0f * PI_FLOAT;
    while (angle < -PI_FLOAT) angle += 2.0f * PI_FLOAT;

    /* Convert angle to angle/π for CORDIC (range [-1, 1]) */
    float angle_norm = angle / PI_FLOAT;

    /* Convert to Q1.31 */
    int32_t angle_q31 = float_to_q31(angle_norm);

    /* Configure CORDIC for SINE function (also computes cosine) */
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_SINE,
                     CORDIC_PRECISION_DEFAULT,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2,             /* 2 writes: angle + modulus */
                     LL_CORDIC_NBREAD_2,              /* 2 reads: sine, cosine */
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);

    /* Write angle and modulus (1.0) */
    LL_CORDIC_WriteData(CORDIC, (uint32_t)angle_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)0x7FFFFFFF);  /* Modulus = 1.0 in Q1.31 */

    /* Read sine and cosine */
    int32_t sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
    int32_t cos_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    /* Convert to float */
    *sin_out = q31_to_float(sin_q31);
    *cos_out = q31_to_float(cos_q31);
}

/**
 * @brief Compute arcsine using CORDIC (via atan2 and sqrt)
 * @note Implements asin(x) = atan2(x, sqrt(1-x²))
 * @note This requires 2 CORDIC operations but provides deterministic timing
 */
float CORDIC_Asin(float x) {
    /* Handle edge cases */
    if (x >= 1.0f) return PI_FLOAT / 2.0f;
    if (x <= -1.0f) return -PI_FLOAT / 2.0f;

    /* Compute sqrt(1 - x²) using CORDIC */
    float x2 = x * x;
    float sqrt_term = CORDIC_Sqrt(1.0f - x2);

    /* Compute atan2(x, sqrt_term) using CORDIC */
    return CORDIC_Atan2(x, sqrt_term);
}

/**
 * @brief Compute arccosine using CORDIC (via atan2 and sqrt)
 * @note Implements acos(x) = atan2(sqrt(1-x²), x)
 * @note This requires 2 CORDIC operations but provides deterministic timing
 */
float CORDIC_Acos(float x) {
    /* Handle edge cases */
    if (x >= 1.0f) return 0.0f;
    if (x <= -1.0f) return PI_FLOAT;

    /* Compute sqrt(1 - x²) using CORDIC */
    float x2 = x * x;
    float sqrt_term = CORDIC_Sqrt(1.0f - x2);

    /* Compute atan2(sqrt_term, x) using CORDIC */
    return CORDIC_Atan2(sqrt_term, x);
}
