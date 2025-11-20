/**
  ******************************************************************************
  * @file    mahony_filter.h
  * @brief   Mahony AHRS Filter for NED Frame Orientation Estimation
  * @author  Generated for Thrust Vector Control Application
  * @note    Enhanced with STM32G4 CORDIC hardware acceleration
  ******************************************************************************
  */

#ifndef MAHONY_FILTER_H_
#define MAHONY_FILTER_H_

#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include "cordic_math.h"  /* Hardware CORDIC acceleration */

/* Mahony filter parameters */
#define MAHONY_KP_DEFAULT           2.0f    /* Proportional gain */
#define MAHONY_KI_DEFAULT           0.01f   /* Integral gain */
#define MAHONY_SAMPLE_FREQ_DEFAULT  500.0f  /* Default sample frequency (Hz) */

/* Mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/* Quaternion structure */
typedef struct {
    float q0, q1, q2, q3;  /* w, x, y, z components */
} Quaternion_t;

/* Euler angles structure (in radians) */
typedef struct {
    float roll;    /* Rotation about X-axis (NED frame) */
    float pitch;   /* Rotation about Y-axis (NED frame) */
    float yaw;     /* Rotation about Z-axis (NED frame) */
} EulerAngles_t;

/* Mahony filter state structure */
typedef struct {
    /* Filter parameters */
    float kp;                    /* Proportional gain */
    float ki;                    /* Integral gain */
    float sample_freq;           /* Sample frequency (Hz) */
    float dt;                    /* Sample period (s) */

    /* State variables */
    Quaternion_t q;              /* Orientation quaternion (NED frame) */
    float integral_error_x;      /* Integral error X */
    float integral_error_y;      /* Integral error Y */
    float integral_error_z;      /* Integral error Z */

    /* Status flags */
    bool initialized;            /* Filter initialization status */
    uint32_t update_count;       /* Number of updates performed */

    /* Last update timestamp */
    uint32_t last_update_time;   /* Last update time (HAL_GetTick()) */

} MahonyFilter_t;

/* Coordinate transformation matrix from sensor to NED frame */
typedef struct {
    float m[3][3];  /* 3x3 rotation matrix */
} RotationMatrix_t;

/* Function prototypes */

/**
 * @brief Initialize the Mahony filter
 * @param filter: Pointer to MahonyFilter_t structure
 * @param kp: Proportional gain (default: 2.0)
 * @param ki: Integral gain (default: 0.01)
 * @param sample_freq: Sample frequency in Hz (default: 500.0)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_Init(MahonyFilter_t *filter, float kp, float ki, float sample_freq);

/**
 * @brief Initialize the Mahony filter with launch pad orientation (X pointing up)
 * @param filter: Pointer to MahonyFilter_t structure
 * @param kp: Proportional gain (default: 2.0)
 * @param ki: Integral gain (default: 0.01)
 * @param sample_freq: Sample frequency in Hz (default: 500.0)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_InitLaunchPad(MahonyFilter_t *filter, float kp, float ki, float sample_freq);

/**
 * @brief Set initial orientation manually
 * @param filter: Pointer to MahonyFilter_t structure
 * @param initial_q: Pointer to desired initial quaternion
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_SetInitialOrientation(MahonyFilter_t *filter, const Quaternion_t *initial_q);

/**
 * @brief Update the Mahony filter with new sensor data
 * @param filter: Pointer to MahonyFilter_t structure
 * @param gx, gy, gz: Gyroscope data in rad/s (sensor frame)
 * @param ax, ay, az: Accelerometer data in m/s² (sensor frame)
 * @param mx, my, mz: Magnetometer data in µT (sensor frame)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_Update(MahonyFilter_t *filter,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz);

/**
 * @brief Update filter with IMU data only (no magnetometer)
 * @param filter: Pointer to MahonyFilter_t structure
 * @param gx, gy, gz: Gyroscope data in rad/s (sensor frame)
 * @param ax, ay, az: Accelerometer data in m/s² (sensor frame)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_UpdateIMU(MahonyFilter_t *filter,
                                   float gx, float gy, float gz,
                                   float ax, float ay, float az);

/**
 * @brief Get current orientation as quaternion (NED frame)
 * @param filter: Pointer to MahonyFilter_t structure
 * @param q: Pointer to Quaternion_t structure to store result
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_GetQuaternion(MahonyFilter_t *filter, Quaternion_t *q);

/**
 * @brief Get current orientation as Euler angles (NED frame)
 * @param filter: Pointer to MahonyFilter_t structure
 * @param euler: Pointer to EulerAngles_t structure to store result
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_GetEulerAngles(MahonyFilter_t *filter, EulerAngles_t *euler);

/**
 * @brief Get current orientation as rotation matrix (NED frame)
 * @param filter: Pointer to MahonyFilter_t structure
 * @param dcm: Pointer to RotationMatrix_t structure to store result
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_GetRotationMatrix(MahonyFilter_t *filter, RotationMatrix_t *dcm);

/**
 * @brief Transform IMU sensor frame data (accel/gyro) to NED frame
 * @param sensor_x, sensor_y, sensor_z: IMU sensor frame data
 * @param ned_x, ned_y, ned_z: Pointers to store NED frame data
 */
void Mahony_TransformIMUToNED(float sensor_x, float sensor_y, float sensor_z,
                              float *ned_x, float *ned_y, float *ned_z);

/**
 * @brief Apply magnetometer alignment compensation then transform to NED frame
 * @param mag_x, mag_y, mag_z: Magnetometer sensor frame data
 * @param ned_x, ned_y, ned_z: Pointers to store NED frame data
 */
void Mahony_TransformMagToNED(float mag_x, float mag_y, float mag_z,
                              float *ned_x, float *ned_y, float *ned_z);

/**
 * @brief Reset the filter to initial state
 * @param filter: Pointer to MahonyFilter_t structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_Reset(MahonyFilter_t *filter);

/**
 * @brief Set filter gains
 * @param filter: Pointer to MahonyFilter_t structure
 * @param kp: Proportional gain
 * @param ki: Integral gain
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_SetGains(MahonyFilter_t *filter, float kp, float ki);

/**
 * @brief Get filter performance statistics
 * @param filter: Pointer to MahonyFilter_t structure
 * @param update_rate: Pointer to store actual update rate (Hz)
 * @param update_count: Pointer to store total update count
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Mahony_GetStats(MahonyFilter_t *filter, float *update_rate, uint32_t *update_count);

/* Utility functions */

/**
 * @brief Normalize a quaternion
 * @param q: Pointer to Quaternion_t structure
 */
void Mahony_QuaternionNormalize(Quaternion_t *q);

/**
 * @brief Convert quaternion to Euler angles (CORDIC-accelerated)
 * @param q: Pointer to Quaternion_t structure
 * @param euler: Pointer to EulerAngles_t structure to store result
 */
void Mahony_QuaternionToEuler(const Quaternion_t *q, EulerAngles_t *euler);

/**
 * @brief Convert quaternion to rotation matrix
 * @param q: Pointer to Quaternion_t structure
 * @param dcm: Pointer to RotationMatrix_t structure to store result
 */
void Mahony_QuaternionToDCM(const Quaternion_t *q, RotationMatrix_t *dcm);

/**
 * @brief Fast inverse square root (Quake algorithm)
 * @param number: Input value
 * @retval float: 1/sqrt(number)
 */
float Mahony_InvSqrt(float number);

#endif /* MAHONY_FILTER_H_ */
