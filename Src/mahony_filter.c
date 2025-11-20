/**
  ******************************************************************************
  * @file    mahony_filter.c
  * @brief   Mahony AHRS Filter - CORDIC HARDWARE ACCELERATED VERSION
  * @note    Key optimizations:
  *          - Uses STM32G4 CORDIC for atan2, sqrt, and asin calculations
  *          - Deterministic timing for real-time control
  *          - ~3-10x faster than software math functions
  ******************************************************************************
  */

#include "mahony_filter.h"

/* Input validation macros */
#define VALIDATE_PTR_VOID(ptr) do { if ((ptr) == NULL) return; } while(0)

/* Magnetometer alignment compensation matrix - IDENTITY (no compensation for MMC5983MA) */
static const RotationMatrix_t MAG_ALIGNMENT_COMPENSATION = {
    .m = {
        { 1.0f,  0.0f,  0.0f},
        { 0.0f,  1.0f,  0.0f},
        { 0.0f,  0.0f,  1.0f}
    }
};

/**
 * @brief Fast inverse square root implementation
 * @note Kept as-is - still competitive for normalization operations
 */
float Mahony_InvSqrt(float number) {
    union {
        float f;
        uint32_t i;
    } conv = {.f = number};

    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (number * 0.5f * conv.f * conv.f);
    conv.f *= 1.5f - (number * 0.5f * conv.f * conv.f);

    return conv.f;
}

/**
 * @brief Normalize a quaternion (unchanged)
 */
void Mahony_QuaternionNormalize(Quaternion_t *q) {
    VALIDATE_PTR_VOID(q);

    float norm = Mahony_InvSqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 *= norm;
    q->q1 *= norm;
    q->q2 *= norm;
    q->q3 *= norm;
}

/**
 * @brief Convert quaternion to Euler angles (CORDIC-ACCELERATED)
 * @note Uses hardware CORDIC for atan2 and asin calculations
 * @note For rocket with X-forward (nose), Y-left, Z-right:
 *       - Roll: rotation about X (nose) axis
 *       - Pitch: rotation about Y axis (nose up/down)
 *       - Yaw: rotation about Z axis (heading)
 */
void Mahony_QuaternionToEuler(const Quaternion_t *q, EulerAngles_t *euler) {
    VALIDATE_PTR_VOID(q);
    VALIDATE_PTR_VOID(euler);

    /* Roll (X-axis rotation) - CORDIC ATAN2 */
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    euler->roll = CORDIC_Atan2(sinr_cosp, cosr_cosp);

    /* Pitch (Y-axis rotation) - CORDIC ASIN via atan2(x, sqrt(1-x²)) */
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f)
        euler->pitch = copysignf(M_PI / 2.0f, sinp);  /* Gimbal lock case */
    else
        euler->pitch = CORDIC_Asin(sinp);  /* CORDIC: 2 operations (sqrt + atan2) */

    /* Yaw (Z-axis rotation) - CORDIC ATAN2 */
    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    euler->yaw = CORDIC_Atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Convert quaternion to rotation matrix (unchanged)
 */
void Mahony_QuaternionToDCM(const Quaternion_t *q, RotationMatrix_t *dcm) {
    VALIDATE_PTR_VOID(q);
    VALIDATE_PTR_VOID(dcm);

    float q0q0 = q->q0 * q->q0;
    float q0q1 = q->q0 * q->q1;
    float q0q2 = q->q0 * q->q2;
    float q0q3 = q->q0 * q->q3;
    float q1q1 = q->q1 * q->q1;
    float q1q2 = q->q1 * q->q2;
    float q1q3 = q->q1 * q->q3;
    float q2q2 = q->q2 * q->q2;
    float q2q3 = q->q2 * q->q3;
    float q3q3 = q->q3 * q->q3;

    dcm->m[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcm->m[0][1] = 2.0f * (q1q2 - q0q3);
    dcm->m[0][2] = 2.0f * (q1q3 + q0q2);

    dcm->m[1][0] = 2.0f * (q1q2 + q0q3);
    dcm->m[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    dcm->m[1][2] = 2.0f * (q2q3 - q0q1);

    dcm->m[2][0] = 2.0f * (q1q3 - q0q2);
    dcm->m[2][1] = 2.0f * (q2q3 + q0q1);
    dcm->m[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**
 * @brief Apply magnetometer alignment compensation (sensor calibration)
 * @note Currently set to IDENTITY matrix (no compensation) for MMC5983MA
 * @note This is NOT a frame transform - it's correcting sensor misalignment
 */
static void Mahony_ApplyMagCalibration(float mag_x, float mag_y, float mag_z,
                                       float *cal_x, float *cal_y, float *cal_z) {
    *cal_x = MAG_ALIGNMENT_COMPENSATION.m[0][0] * mag_x +
             MAG_ALIGNMENT_COMPENSATION.m[0][1] * mag_y +
             MAG_ALIGNMENT_COMPENSATION.m[0][2] * mag_z;

    *cal_y = MAG_ALIGNMENT_COMPENSATION.m[1][0] * mag_x +
             MAG_ALIGNMENT_COMPENSATION.m[1][1] * mag_y +
             MAG_ALIGNMENT_COMPENSATION.m[1][2] * mag_z;

    *cal_z = MAG_ALIGNMENT_COMPENSATION.m[2][0] * mag_x +
             MAG_ALIGNMENT_COMPENSATION.m[2][1] * mag_y +
             MAG_ALIGNMENT_COMPENSATION.m[2][2] * mag_z;
}

/**
 * @brief Initialize the Mahony filter (standard - level orientation)
 * @note Also initializes CORDIC math module
 */
HAL_StatusTypeDef Mahony_Init(MahonyFilter_t *filter, float kp, float ki, float sample_freq) {
    if (filter == NULL) {
        return HAL_ERROR;
    }

    /* Initialize CORDIC math module */
    CORDIC_Math_Init();

    /* Set filter parameters */
    filter->kp = (kp > 0.0f) ? kp : MAHONY_KP_DEFAULT;
    filter->ki = (ki >= 0.0f) ? ki : MAHONY_KI_DEFAULT;
    filter->sample_freq = (sample_freq > 0.0f) ? sample_freq : MAHONY_SAMPLE_FREQ_DEFAULT;
    filter->dt = 1.0f / filter->sample_freq;

    /* Initialize quaternion to identity (level orientation, X forward) */
    filter->q.q0 = 1.0f;
    filter->q.q1 = 0.0f;
    filter->q.q2 = 0.0f;
    filter->q.q3 = 0.0f;

    /* Initialize integral error terms */
    filter->integral_error_x = 0.0f;
    filter->integral_error_y = 0.0f;
    filter->integral_error_z = 0.0f;

    /* Initialize status */
    filter->initialized = true;
    filter->update_count = 0;
    filter->last_update_time = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief Initialize for rocket on launch pad (X pointing up/nose direction)
 * @note Also initializes CORDIC math module
 */
HAL_StatusTypeDef Mahony_InitLaunchPad(MahonyFilter_t *filter, float kp, float ki, float sample_freq) {
    if (filter == NULL) {
        return HAL_ERROR;
    }

    /* Initialize CORDIC math module */
    CORDIC_Math_Init();

    /* Set filter parameters */
    filter->kp = (kp > 0.0f) ? kp : MAHONY_KP_DEFAULT;
    filter->ki = (ki >= 0.0f) ? ki : MAHONY_KI_DEFAULT;
    filter->sample_freq = (sample_freq > 0.0f) ? sample_freq : MAHONY_SAMPLE_FREQ_DEFAULT;
    filter->dt = 1.0f / filter->sample_freq;

    /* Initialize quaternion for nose-up orientation
     * This represents a -90° rotation about Y-axis (pitch up)
     * q = [cos(-45°), 0, sin(-45°), 0] = [0.707, 0, -0.707, 0]
     */
    filter->q.q0 = 0.7071068f;   // cos(-pi/4)
    filter->q.q1 = 0.0f;
    filter->q.q2 = -0.7071068f;   // sin(-pi/4)
    filter->q.q3 = 0.0f;

    /* Initialize integral error terms */
    filter->integral_error_x = 0.0f;
    filter->integral_error_y = 0.0f;
    filter->integral_error_z = 0.0f;

    /* Initialize status */
    filter->initialized = true;
    filter->update_count = 0;
    filter->last_update_time = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief Update the Mahony filter with new sensor data (9-DOF) - CORDIC ACCELERATED
 * @note NOW PROCESSES IN BODY/SENSOR FRAME - NO INPUT TRANSFORM
 * @note Uses CORDIC for sqrt calculation (bx computation)
 */
HAL_StatusTypeDef Mahony_Update(MahonyFilter_t *filter,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz) {
    if (filter == NULL || !filter->initialized) {
        return HAL_ERROR;
    }

    /* Apply magnetometer calibration (sensor alignment, not frame transform) */
    float mx_cal, my_cal, mz_cal;
    Mahony_ApplyMagCalibration(mx, my, mz, &mx_cal, &my_cal, &mz_cal);

    /* Calculate dynamic dt */
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - filter->last_update_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = filter->dt;
    }
    filter->last_update_time = current_time;

    /* Normalize accelerometer measurement */
    float acc_norm = Mahony_InvSqrt(ax * ax + ay * ay + az * az);
    if (acc_norm == 0.0f) return HAL_ERROR;

    ax *= acc_norm;
    ay *= acc_norm;
    az *= acc_norm;

    /* Normalize magnetometer measurement */
    float mag_norm = Mahony_InvSqrt(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal);
    if (mag_norm == 0.0f) return HAL_ERROR;

    mx_cal *= mag_norm;
    my_cal *= mag_norm;
    mz_cal *= mag_norm;

    /* Reference direction of Earth's magnetic field */
    float hx = 2.0f * (mx_cal * (0.5f - filter->q.q2 * filter->q.q2 - filter->q.q3 * filter->q.q3) +
                       my_cal * (filter->q.q1 * filter->q.q2 - filter->q.q0 * filter->q.q3) +
                       mz_cal * (filter->q.q1 * filter->q.q3 + filter->q.q0 * filter->q.q2));
    float hy = 2.0f * (mx_cal * (filter->q.q1 * filter->q.q2 + filter->q.q0 * filter->q.q3) +
                       my_cal * (0.5f - filter->q.q1 * filter->q.q1 - filter->q.q3 * filter->q.q3) +
                       mz_cal * (filter->q.q2 * filter->q.q3 - filter->q.q0 * filter->q.q1));

    /* CORDIC-ACCELERATED: Compute bx using hardware sqrt */
    float bx = CORDIC_Sqrt(hx * hx + hy * hy);

    float bz = 2.0f * (mx_cal * (filter->q.q1 * filter->q.q3 - filter->q.q0 * filter->q.q2) +
                       my_cal * (filter->q.q2 * filter->q.q3 + filter->q.q0 * filter->q.q1) +
                       mz_cal * (0.5f - filter->q.q1 * filter->q.q1 - filter->q.q2 * filter->q.q2));

    /* Estimated direction of gravity and magnetic field */
    float vx = 2.0f * (filter->q.q1 * filter->q.q3 - filter->q.q0 * filter->q.q2);
    float vy = 2.0f * (filter->q.q0 * filter->q.q1 + filter->q.q2 * filter->q.q3);
    float vz = filter->q.q0 * filter->q.q0 - filter->q.q1 * filter->q.q1 -
               filter->q.q2 * filter->q.q2 + filter->q.q3 * filter->q.q3;

    float wx = 2.0f * (bx * (0.5f - filter->q.q2 * filter->q.q2 - filter->q.q3 * filter->q.q3) +
                       bz * (filter->q.q1 * filter->q.q3 - filter->q.q0 * filter->q.q2));
    float wy = 2.0f * (bx * (filter->q.q1 * filter->q.q2 - filter->q.q0 * filter->q.q3) +
                       bz * (filter->q.q0 * filter->q.q1 + filter->q.q2 * filter->q.q3));
    float wz = 2.0f * (bx * (filter->q.q0 * filter->q.q2 + filter->q.q1 * filter->q.q3) +
                       bz * (0.5f - filter->q.q1 * filter->q.q1 - filter->q.q2 * filter->q.q2));

    /* Error is cross product between estimated and measured directions */
    float ex = (ay * vz - az * vy) + (my_cal * wz - mz_cal * wy);
    float ey = (az * vx - ax * vz) + (mz_cal * wx - mx_cal * wz);
    float ez = (ax * vy - ay * vx) + (mx_cal * wy - my_cal * wx);

    /* Compute and apply integral feedback if enabled */
    if (filter->ki > 0.0f) {
        filter->integral_error_x += ex * dt;
        filter->integral_error_y += ey * dt;
        filter->integral_error_z += ez * dt;

        gx += filter->ki * filter->integral_error_x;
        gy += filter->ki * filter->integral_error_y;
        gz += filter->ki * filter->integral_error_z;
    }

    /* Apply proportional feedback */
    gx += filter->kp * ex;
    gy += filter->kp * ey;
    gz += filter->kp * ez;

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = filter->q.q0;
    float qb = filter->q.q1;
    float qc = filter->q.q2;

    filter->q.q0 += (-qb * gx - qc * gy - filter->q.q3 * gz);
    filter->q.q1 += (qa * gx + qc * gz - filter->q.q3 * gy);
    filter->q.q2 += (qa * gy - qb * gz + filter->q.q3 * gx);
    filter->q.q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalize quaternion */
    Mahony_QuaternionNormalize(&filter->q);

    filter->update_count++;
    return HAL_OK;
}

/**
 * @brief Update filter with IMU data only (6-DOF, no magnetometer)
 * @note NOW PROCESSES IN BODY/SENSOR FRAME - NO INPUT TRANSFORM
 */
HAL_StatusTypeDef Mahony_UpdateIMU(MahonyFilter_t *filter,
                                   float gx, float gy, float gz,
                                   float ax, float ay, float az) {
    if (filter == NULL || !filter->initialized) {
        return HAL_ERROR;
    }

    /* Calculate dynamic dt */
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - filter->last_update_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = filter->dt;
    }
    filter->last_update_time = current_time;

    /* Normalize accelerometer measurement */
    float acc_norm = Mahony_InvSqrt(ax * ax + ay * ay + az * az);
    if (acc_norm == 0.0f) return HAL_ERROR;

    ax *= acc_norm;
    ay *= acc_norm;
    az *= acc_norm;

    /* Estimated direction of gravity */
    float vx = 2.0f * (filter->q.q1 * filter->q.q3 - filter->q.q0 * filter->q.q2);
    float vy = 2.0f * (filter->q.q0 * filter->q.q1 + filter->q.q2 * filter->q.q3);
    float vz = filter->q.q0 * filter->q.q0 - filter->q.q1 * filter->q.q1 -
               filter->q.q2 * filter->q.q2 + filter->q.q3 * filter->q.q3;

    /* Error is cross product between estimated and measured direction of gravity */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* Compute and apply integral feedback if enabled */
    if (filter->ki > 0.0f) {
        filter->integral_error_x += ex * dt;
        filter->integral_error_y += ey * dt;
        filter->integral_error_z += ez * dt;

        gx += filter->ki * filter->integral_error_x;
        gy += filter->ki * filter->integral_error_y;
        gz += filter->ki * filter->integral_error_z;
    }

    /* Apply proportional feedback */
    gx += filter->kp * ex;
    gy += filter->kp * ey;
    gz += filter->kp * ez;

    /* Integrate rate of change of quaternion */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = filter->q.q0;
    float qb = filter->q.q1;
    float qc = filter->q.q2;

    filter->q.q0 += (-qb * gx - qc * gy - filter->q.q3 * gz);
    filter->q.q1 += (qa * gx + qc * gz - filter->q.q3 * gy);
    filter->q.q2 += (qa * gy - qb * gz + filter->q.q3 * gx);
    filter->q.q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalize quaternion */
    Mahony_QuaternionNormalize(&filter->q);

    filter->update_count++;
    return HAL_OK;
}

/* All other functions remain unchanged */

HAL_StatusTypeDef Mahony_GetQuaternion(MahonyFilter_t *filter, Quaternion_t *q) {
    if (filter == NULL || q == NULL || !filter->initialized) {
        return HAL_ERROR;
    }
    *q = filter->q;
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_GetEulerAngles(MahonyFilter_t *filter, EulerAngles_t *euler) {
    if (filter == NULL || euler == NULL || !filter->initialized) {
        return HAL_ERROR;
    }
    Mahony_QuaternionToEuler(&filter->q, euler);
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_GetRotationMatrix(MahonyFilter_t *filter, RotationMatrix_t *dcm) {
    if (filter == NULL || dcm == NULL || !filter->initialized) {
        return HAL_ERROR;
    }
    Mahony_QuaternionToDCM(&filter->q, dcm);
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_SetInitialOrientation(MahonyFilter_t *filter, const Quaternion_t *initial_q) {
    if (filter == NULL || initial_q == NULL) {
        return HAL_ERROR;
    }
    filter->q = *initial_q;
    Mahony_QuaternionNormalize(&filter->q);
    filter->integral_error_x = 0.0f;
    filter->integral_error_y = 0.0f;
    filter->integral_error_z = 0.0f;
    filter->update_count = 0;
    filter->last_update_time = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_Reset(MahonyFilter_t *filter) {
    if (filter == NULL) {
        return HAL_ERROR;
    }
    filter->q.q0 = 1.0f;
    filter->q.q1 = 0.0f;
    filter->q.q2 = 0.0f;
    filter->q.q3 = 0.0f;
    filter->integral_error_x = 0.0f;
    filter->integral_error_y = 0.0f;
    filter->integral_error_z = 0.0f;
    filter->update_count = 0;
    filter->last_update_time = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_SetGains(MahonyFilter_t *filter, float kp, float ki) {
    if (filter == NULL || kp < 0.0f || ki < 0.0f) {
        return HAL_ERROR;
    }
    filter->kp = kp;
    filter->ki = ki;
    return HAL_OK;
}

HAL_StatusTypeDef Mahony_GetStats(MahonyFilter_t *filter, float *update_rate, uint32_t *update_count) {
    if (filter == NULL) {
        return HAL_ERROR;
    }
    if (update_count != NULL) {
        *update_count = filter->update_count;
    }
    if (update_rate != NULL) {
        uint32_t elapsed_time = HAL_GetTick() -
            (filter->last_update_time - (filter->update_count > 0 ?
             filter->update_count * (1000.0f / filter->sample_freq) : 0));
        if (elapsed_time > 0) {
            *update_rate = (float)filter->update_count * 1000.0f / elapsed_time;
        } else {
            *update_rate = 0.0f;
        }
    }
    return HAL_OK;
}

/* Helper functions for frame conversions (if needed) */

void Mahony_ConvertEulerBodyToNED(const EulerAngles_t *body_euler, EulerAngles_t *ned_euler) {
    ned_euler->roll = body_euler->roll;
    ned_euler->pitch = body_euler->pitch;
    ned_euler->yaw = body_euler->yaw;
}

void Mahony_TransformBodyToNED(MahonyFilter_t *filter,
                               float body_x, float body_y, float body_z,
                               float *ned_n, float *ned_e, float *ned_d) {
    RotationMatrix_t dcm;
    Mahony_QuaternionToDCM(&filter->q, &dcm);

    *ned_n = dcm.m[0][0] * body_x + dcm.m[0][1] * body_y + dcm.m[0][2] * body_z;
    *ned_e = dcm.m[1][0] * body_x + dcm.m[1][1] * body_y + dcm.m[1][2] * body_z;
    *ned_d = dcm.m[2][0] * body_x + dcm.m[2][1] * body_y + dcm.m[2][2] * body_z;
}
