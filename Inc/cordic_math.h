/**
  ******************************************************************************
  * @file    cordic_math.h
  * @brief   Hardware CORDIC accelerated math functions for STM32G4
  * @note    These functions directly replace standard math.h functions with
  *          hardware-accelerated CORDIC equivalents. No runtime switching.
  ******************************************************************************
  */

#ifndef CORDIC_MATH_H_
#define CORDIC_MATH_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"
#include <math.h>

/* CORDIC handle - must be initialized by cordic.c */
extern CORDIC_HandleTypeDef hcordic;

/* Configuration */
#define CORDIC_PRECISION_DEFAULT    LL_CORDIC_PRECISION_15CYCLES  /* Maximum precision (~20-24 bits) */

/**
 * @brief Initialize CORDIC math module
 * @note Must be called after MX_CORDIC_Init()
 */
void CORDIC_Math_Init(void);

/**
 * @brief Compute atan2(y, x) using CORDIC PHASE function
 * @param y: Y coordinate (numerator)
 * @param x: X coordinate (denominator)
 * @return Angle in radians [-π, π]
 */
float CORDIC_Atan2(float y, float x);

/**
 * @brief Compute square root using CORDIC
 * @param x: Input value (must be >= 0)
 * @return Square root of x
 */
float CORDIC_Sqrt(float x);

/**
 * @brief Compute sine and cosine simultaneously using CORDIC
 * @param angle: Angle in radians
 * @param sin_out: Pointer to store sine result
 * @param cos_out: Pointer to store cosine result
 */
void CORDIC_SinCos(float angle, float *sin_out, float *cos_out);

/**
 * @brief Compute arcsine using CORDIC (via atan2 and sqrt)
 * @param x: Input value [-1, 1]
 * @return Angle in radians [-π/2, π/2]
 * @note Implements asin(x) = atan2(x, sqrt(1-x²))
 */
float CORDIC_Asin(float x);

#endif /* CORDIC_MATH_H_ */
