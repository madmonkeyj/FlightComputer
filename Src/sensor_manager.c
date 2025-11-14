/**
  ******************************************************************************
  * @file    sensor_manager.c
  * @brief   Unified sensor management with barometer and I2C DMA arbiter
  ******************************************************************************
  */

#include "sensor_manager.h"
#include "icm42688.h"
#include "mmc5983ma.h"
#include "h3lis331dl.h"
#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include <string.h>
#include <math.h>

/* Private variables */
static SensorManager_Config_t config;
static SensorManager_Status_t sensor_status;
static SensorManager_Scales_t scales;
static SensorManager_RawData_t latest_data;
static uint32_t last_read_time_us = 0;

/* Decimation counters for low-priority sensors */
static uint8_t baro_decimation_counter = 0;
static uint8_t highg_decimation_counter = 0;

/* Altitude calculation */
#define STANDARD_SEA_LEVEL_PA   101325.0f

/* Default configuration */
static const SensorManager_Config_t default_config = {
    .imu_odr_hz = 1000,
    .mag_odr_hz = 1000,
    .baro_odr_hz = 100,
    .imu_accel_fs = 0,
    .imu_gyro_fs = 0,
    .use_mag = true,
    .use_high_g = false,
    .use_baro = false
};

/* Private function prototypes */
static inline uint32_t GetMicros(void);
static void CalculateScalingFactors(void);
static float CalculateAltitude(float pressure_pa, float sea_level_pa);

/* Microsecond timer */
static inline uint32_t GetMicros(void) {
    uint32_t m = HAL_GetTick();
    uint32_t u = SysTick->LOAD - SysTick->VAL;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        m++;
        u = SysTick->LOAD - SysTick->VAL;
    }

    return (m * 1000) + (u * 1000 / SysTick->LOAD);
}

static void CalculateScalingFactors(void) {
    /* ICM42688 at ±16g: 2048 LSB/g */
    scales.accel_scale = 1.0f / 2048.0f;

    /* ICM42688 at ±2000dps: 16.4 LSB/(deg/s) */
    scales.gyro_scale = 1.0f / 16.4f;

    /* MMC5983MA: 16384 LSB/Gauss */
    scales.mag_scale = 1.0f / 16384.0f;

    /* ICM42688 Temperature */
    scales.temp_scale = 1.0f / 132.48f;
    scales.temp_offset = 25.0f;

    /* BMP581: 64 LSB/Pa */
    scales.pressure_scale = 1.0f / 64.0f;
}

static float CalculateAltitude(float pressure_pa, float sea_level_pa) {
    /* Standard barometric formula: h = 44330 * (1 - (P/P0)^0.1903) */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

HAL_StatusTypeDef SensorManager_Init(const SensorManager_Config_t *user_config) {
    HAL_StatusTypeDef status;

    /* Use provided config or defaults */
    if (user_config != NULL) {
        memcpy(&config, user_config, sizeof(SensorManager_Config_t));
    } else {
        memcpy(&config, &default_config, sizeof(SensorManager_Config_t));
    }

    /* Initialize I2C DMA arbiter */
    I2C_DMA_Arbiter_Init();

    /* Calculate scaling factors */
    CalculateScalingFactors();

    /* Initialize statistics */
    memset(&sensor_status, 0, sizeof(SensorManager_Status_t));
    memset(&latest_data, 0, sizeof(SensorManager_RawData_t));

    /* Initialize IMU (SPI - no arbiter needed) */
    status = ICM42688_Init();
    if (status != HAL_OK) {
        return status;
    }

    /* Initialize Magnetometer (highest priority on I2C) */
    if (config.use_mag) {
        status = MMC5983MA_Init();
        if (status != HAL_OK) {
            return status;
        }
    }

    /* Initialize Barometer (medium priority on I2C) */
    if (config.use_baro) {
        status = BMP581_Init();
        if (status != HAL_OK) {
            /* Non-critical - continue */
        }
    }

    /* Initialize High-G Accelerometer (lowest priority on I2C) */
    if (config.use_high_g) {
        status = H3LIS331DL_Init();
        if (status != HAL_OK) {
            /* Non-critical - continue */
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef SensorManager_ReadRaw(SensorManager_RawData_t *data) {
    HAL_StatusTypeDef hal_status = HAL_OK;
    ICM42688_Data_t imu_data;

    /* ALWAYS read IMU (critical for Mahony filter at 500Hz) */
    hal_status = ICM42688_ReadSensorData(&imu_data);
    if (hal_status == HAL_OK) {
        data->accel_x = imu_data.accel_x;
        data->accel_y = imu_data.accel_y;
        data->accel_z = imu_data.accel_z;
        data->gyro_x = imu_data.gyro_x;
        data->gyro_y = imu_data.gyro_y;
        data->gyro_z = imu_data.gyro_z;
        data->imu_valid = 1;
        sensor_status.imu_read_count++;
    } else {
        data->imu_valid = 0;
        sensor_status.imu_error_count++;
    }

    /* ALWAYS try MAG (highest priority, needed for Mahony heading) */
    if (config.use_mag) {
        MMC5983MA_Data_t mag_data;
        HAL_StatusTypeDef mag_status = MMC5983MA_ReadSensorData(&mag_data);
        if (mag_status == HAL_OK) {
            data->mag_x = mag_data.mag_x;
            data->mag_y = mag_data.mag_y;
            data->mag_z = mag_data.mag_z;
            data->mag_valid = 1;
            sensor_status.mag_read_count++;
        } else {
            data->mag_valid = 0;
            sensor_status.mag_error_count++;
        }
    } else {
        data->mag_valid = 0;
    }

    /* DECIMATE BARO to target rate (e.g., 100Hz when running at 500Hz loop) */
    /* Calculate decimation: loop_rate / baro_rate */
    /* For 500Hz loop with 100Hz baro: read every 5th call */
    uint8_t baro_decimation = 5;  // TODO: Calculate based on config
    if (config.use_baro) {
        if (baro_decimation_counter++ >= baro_decimation) {
            baro_decimation_counter = 0;

            /* Try to read, but skip if arbiter busy (fail-fast) */
            if (!I2C_DMA_Arbiter_IsBusy(&hi2c1)) {
                BMP581_Data_t baro_data;
                HAL_StatusTypeDef baro_status = BMP581_ReadSensorData(&baro_data);
                if (baro_status == HAL_OK) {
                    data->pressure_raw = baro_data.pressure_raw;
                    data->temperature_raw = baro_data.temperature_raw;
                    data->baro_valid = 1;
                    sensor_status.baro_read_count++;
                } else {
                    data->baro_valid = 0;
                    if (baro_status != HAL_BUSY) {
                        sensor_status.baro_error_count++;
                    }
                }
            } else {
                data->baro_valid = 0;
                /* Arbiter busy - not an error, just skip this read */
            }
        } else {
            data->baro_valid = 0;
        }
    } else {
        data->baro_valid = 0;
    }

    /* DECIMATE High-G (lowest priority, shock detection only) */
    if (config.use_high_g) {
        uint8_t highg_decimation = 10;  // Read every 10th call
        if (highg_decimation_counter++ >= highg_decimation) {
            highg_decimation_counter = 0;

            /* Skip if arbiter busy */
            if (!I2C_DMA_Arbiter_IsBusy(&hi2c1)) {
                H3LIS331DL_Data_t high_g_data;
                HAL_StatusTypeDef high_g_status = H3LIS331DL_ReadSensorData(&high_g_data);
                if (high_g_status == HAL_OK) {
                    data->high_g_x = high_g_data.accel_x;
                    data->high_g_y = high_g_data.accel_y;
                    data->high_g_z = high_g_data.accel_z;
                    data->high_g_valid = 1;
                    sensor_status.high_g_read_count++;
                } else {
                    data->high_g_valid = 0;
                    if (high_g_status != HAL_BUSY) {
                        sensor_status.high_g_error_count++;
                    }
                }
            } else {
                data->high_g_valid = 0;
            }
        } else {
            data->high_g_valid = 0;
        }
    } else {
        data->high_g_valid = 0;
    }

#if SENSOR_MANAGER_TIMESTAMP
    data->timestamp_us = GetMicros();
#endif

    /* Update rate calculation */
    uint32_t current_time = GetMicros();
    if (last_read_time_us > 0) {
        uint32_t dt = current_time - last_read_time_us;
        if (dt > 0) {
            sensor_status.actual_rate_hz = 1000000.0f / (float)dt;
        }
    }
    last_read_time_us = current_time;
    sensor_status.last_update_us = current_time;

    /* Store latest data */
    memcpy(&latest_data, data, sizeof(SensorManager_RawData_t));

    return hal_status;
}

HAL_StatusTypeDef SensorManager_ReadScaled(SensorManager_ScaledData_t *data) {
    SensorManager_RawData_t raw;
    HAL_StatusTypeDef hal_status;

    hal_status = SensorManager_ReadRaw(&raw);

    if (hal_status == HAL_OK && raw.imu_valid) {
        SensorManager_ConvertToScaled(&raw, data);
    }

    return hal_status;
}

void SensorManager_ConvertToScaled(const SensorManager_RawData_t *raw,
                                    SensorManager_ScaledData_t *scaled) {
    /* Accelerometer conversion */
    scaled->accel_x_g = (float)raw->accel_x * scales.accel_scale;
    scaled->accel_y_g = (float)raw->accel_y * scales.accel_scale;
    scaled->accel_z_g = (float)raw->accel_z * scales.accel_scale;

    /* Gyroscope conversion */
    scaled->gyro_x_dps = (float)raw->gyro_x * scales.gyro_scale;
    scaled->gyro_y_dps = (float)raw->gyro_y * scales.gyro_scale;
    scaled->gyro_z_dps = (float)raw->gyro_z * scales.gyro_scale;

    /* Magnetometer conversion */
    scaled->mag_x_gauss = (float)raw->mag_x * scales.mag_scale;
    scaled->mag_y_gauss = (float)raw->mag_y * scales.mag_scale;
    scaled->mag_z_gauss = (float)raw->mag_z * scales.mag_scale;

    /* Barometer conversion */
    if (raw->baro_valid) {
        scaled->pressure_pa = (float)raw->pressure_raw * scales.pressure_scale;
        scaled->temperature_c = (float)raw->temperature_raw * (1.0f / 65536.0f);  // BMP581 temp scale
        scaled->altitude_m = CalculateAltitude(scaled->pressure_pa, STANDARD_SEA_LEVEL_PA);
    } else {
        scaled->pressure_pa = 0.0f;
        scaled->temperature_c = 25.0f;
        scaled->altitude_m = 0.0f;
    }

#if SENSOR_MANAGER_TIMESTAMP
    scaled->timestamp_us = raw->timestamp_us;
#endif
}

void SensorManager_GetScales(SensorManager_Scales_t *out_scales) {
    if (out_scales != NULL) {
        memcpy(out_scales, &scales, sizeof(SensorManager_Scales_t));
    }
}

void SensorManager_GetStatus(SensorManager_Status_t *out_status) {
    if (out_status != NULL) {
        memcpy(out_status, &sensor_status, sizeof(SensorManager_Status_t));
    }
}

/* Interrupt callbacks */
void SensorManager_IMU_DataReady_Callback(void) {
    /* Can be used for event-driven architecture */
}

void SensorManager_MAG_DataReady_Callback(void) {
    /* Can be used for event-driven architecture */
}

void SensorManager_HighG_DataReady_Callback(void) {
    /* Can be used for shock detection */
}

void SensorManager_BARO_DataReady_Callback(void) {
    /* Can be used for barometer data ready */
}

/* DMA completion callbacks */
void SensorManager_IMU_DMA_Complete_Callback(void) {
    ICM42688_DMA_Complete_Callback();
}

void SensorManager_MAG_DMA_Complete_Callback(void) {
    MMC5983MA_DMA_Complete_Callback();
    I2C_DMA_Arbiter_HandleComplete(&hi2c1);
}

void SensorManager_BARO_DMA_Complete_Callback(void) {
    BMP581_DMA_Complete_Callback();
    I2C_DMA_Arbiter_HandleComplete(&hi2c1);
}
