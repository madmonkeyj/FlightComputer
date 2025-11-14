/**
  ******************************************************************************
  * @file    sensor_manager.h
  * @brief   Unified sensor management - DMA ONLY version
  *          Optimized for 500Hz+ Mahony filter integration
  ******************************************************************************
  */

#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_MANAGER_TIMESTAMP        1

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
    int16_t high_g_x;
    int16_t high_g_y;
    int16_t high_g_z;
#if SENSOR_MANAGER_TIMESTAMP
    uint32_t timestamp_us;
#endif
    uint8_t imu_valid    : 1;
    uint8_t mag_valid    : 1;
    uint8_t high_g_valid : 1;
    uint8_t reserved     : 5;
} SensorManager_RawData_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
    float temperature_c;
    uint32_t timestamp_us;
} SensorManager_ScaledData_t;

typedef struct {
    uint16_t imu_odr_hz;
    uint16_t mag_odr_hz;
    uint8_t  imu_accel_fs;
    uint8_t  imu_gyro_fs;
    bool     use_mag;
    bool     use_high_g;
} SensorManager_Config_t;

typedef struct {
    uint32_t imu_read_count;
    uint32_t mag_read_count;
    uint32_t high_g_read_count;
    uint32_t imu_error_count;
    uint32_t mag_error_count;
    uint32_t high_g_error_count;
    uint32_t last_update_us;
    float    actual_rate_hz;
} SensorManager_Status_t;

typedef struct {
    float accel_scale;
    float gyro_scale;
    float mag_scale;
    float temp_scale;
    float temp_offset;
} SensorManager_Scales_t;

/* Function prototypes - REMOVED TestAllSensors(), StartContinuous(), StopContinuous(), IsDataReady() */
HAL_StatusTypeDef SensorManager_Init(const SensorManager_Config_t *config);
HAL_StatusTypeDef SensorManager_ReadRaw(SensorManager_RawData_t *data);
HAL_StatusTypeDef SensorManager_ReadScaled(SensorManager_ScaledData_t *data);
void SensorManager_GetScales(SensorManager_Scales_t *scales);
void SensorManager_GetStatus(SensorManager_Status_t *status);
void SensorManager_ConvertToScaled(const SensorManager_RawData_t *raw,
                                     SensorManager_ScaledData_t *scaled);

/* Interrupt callbacks */
void SensorManager_IMU_DataReady_Callback(void);
void SensorManager_MAG_DataReady_Callback(void);
void SensorManager_HighG_DataReady_Callback(void);

/* DMA completion callbacks */
void SensorManager_IMU_DMA_Complete_Callback(void);
void SensorManager_MAG_DMA_Complete_Callback(void);

#endif /* SENSOR_MANAGER_H_ */
