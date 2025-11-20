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
#include "mahony_filter.h"

/* Input validation macros */
#define VALIDATE_PTR(ptr) do { if ((ptr) == NULL) return HAL_ERROR; } while(0)
#define VALIDATE_PTR_VOID(ptr) do { if ((ptr) == NULL) return; } while(0)

/* Private variables */
static SensorManager_Config_t config;
static SensorManager_Status_t sensor_status;
static SensorManager_Scales_t scales;
static SensorManager_RawData_t latest_data;
static uint32_t last_read_time_us = 0;

/* Decimation counters and factors for low-priority sensors */
static uint8_t baro_decimation_counter = 0;
static uint8_t highg_decimation_counter = 0;
static uint8_t baro_decimation_factor = 5;   /* Calculated from config in Init() */
static uint8_t highg_decimation_factor = 10; /* Calculated from config in Init() */

/* Sensor scaling constants */
/* ICM42688 IMU scaling factors */
#define ICM42688_GYRO_SCALE_2000DPS     16.4f       /* LSB per deg/s at ±2000dps */
#define ICM42688_ACCEL_SCALE_16G        2048.0f     /* LSB per g at ±16g */
#define ICM42688_TEMP_SCALE             132.48f     /* LSB per °C */
#define ICM42688_TEMP_OFFSET            25.0f       /* Temperature offset (°C) */

/* MMC5983MA magnetometer scaling */
#define MMC5983MA_MAG_SCALE             16384.0f    /* LSB per Gauss */
#define GAUSS_TO_MICROTESLA             100.0f      /* 1 Gauss = 100 µT */

/* BMP581 barometer scaling */
#define BMP581_PRESSURE_SCALE           64.0f       /* LSB per Pascal */
#define BMP581_TEMP_SCALE               65536.0f    /* LSB per °C */

/* Physics constants */
#define GRAVITY_MSS                     9.81f       /* Standard gravity (m/s²) */
#define BAROMETRIC_CONSTANT_M           44330.0f    /* Barometric altitude formula constant (m) */
#define BAROMETRIC_EXPONENT             0.1903f     /* Barometric altitude formula exponent */
#define STANDARD_SEA_LEVEL_PA           101325.0f   /* Standard sea level pressure (Pa) */

/* Conversion constants */
#define DEG_TO_RAD                      (M_PI / 180.0f)
#ifndef M_PI
#define M_PI                            3.14159265358979323846f
#endif

/* Default configuration */
static const SensorManager_Config_t default_config = {
    .imu_odr_hz = 1000,
    .mag_odr_hz = 1000,
    .baro_odr_hz = 100,
    .highg_odr_hz = 50,      /* High-G accel decimated to 50 Hz */
    .main_loop_hz = 500,     /* Assume 500 Hz main loop */
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

/* Microsecond timer using DWT cycle counter (race-free, higher precision) */
static inline uint32_t GetMicros(void) {
    /* DWT cycle counter is incremented on every CPU cycle */
    /* At 170 MHz: 1 µs = 170 cycles */
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static void CalculateScalingFactors(void) {
    /* ICM42688 accelerometer */
    scales.accel_scale = 1.0f / ICM42688_ACCEL_SCALE_16G;

    /* ICM42688 gyroscope */
    scales.gyro_scale = 1.0f / ICM42688_GYRO_SCALE_2000DPS;

    /* MMC5983MA magnetometer */
    scales.mag_scale = 1.0f / MMC5983MA_MAG_SCALE;

    /* ICM42688 temperature */
    scales.temp_scale = 1.0f / ICM42688_TEMP_SCALE;
    scales.temp_offset = ICM42688_TEMP_OFFSET;

    /* BMP581 barometer */
    scales.pressure_scale = 1.0f / BMP581_PRESSURE_SCALE;
}

static float CalculateAltitude(float pressure_pa, float sea_level_pa) {
    /* Standard barometric formula: h = 44330 * (1 - (P/P0)^0.1903) */
    return BAROMETRIC_CONSTANT_M * (1.0f - powf(pressure_pa / sea_level_pa, BAROMETRIC_EXPONENT));
}

HAL_StatusTypeDef SensorManager_Init(const SensorManager_Config_t *user_config) {
    HAL_StatusTypeDef status;

    /* Use provided config or defaults */
    if (user_config != NULL) {
        memcpy(&config, user_config, sizeof(SensorManager_Config_t));
    } else {
        memcpy(&config, &default_config, sizeof(SensorManager_Config_t));
    }

    /* Initialize DWT cycle counter for microsecond timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* Enable trace */
    DWT->CYCCNT = 0;                                  /* Reset counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             /* Enable counter */

    /* Initialize I2C DMA arbiter */
    I2C_DMA_Arbiter_Init();

    /* Calculate scaling factors */
    CalculateScalingFactors();

    /* Calculate decimation factors from configuration */
    if (config.baro_odr_hz > 0 && config.main_loop_hz > 0) {
        baro_decimation_factor = config.main_loop_hz / config.baro_odr_hz;
        if (baro_decimation_factor == 0) baro_decimation_factor = 1;
    } else {
        baro_decimation_factor = 5;  /* Default fallback */
    }

    if (config.highg_odr_hz > 0 && config.main_loop_hz > 0) {
        highg_decimation_factor = config.main_loop_hz / config.highg_odr_hz;
        if (highg_decimation_factor == 0) highg_decimation_factor = 1;
    } else {
        highg_decimation_factor = 10;  /* Default fallback */
    }

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
    VALIDATE_PTR(data);

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

    /* DECIMATE BARO to target rate */
    /* Decimation factor calculated from config in SensorManager_Init() */
    static BMP581_Data_t last_valid_baro_data = {0};

    if (config.use_baro) {
        if (baro_decimation_counter++ >= baro_decimation_factor) {
            baro_decimation_counter = 0;

            /* Try to read, but skip if arbiter busy (fail-fast) */
            if (!I2C_DMA_Arbiter_IsBusy(&hi2c1)) {
                BMP581_Data_t baro_data;
                HAL_StatusTypeDef baro_status = BMP581_ReadSensorData(&baro_data);

                if (baro_status == HAL_OK) {
                    /* New valid data */
                    data->pressure_raw = baro_data.pressure_raw;
                    data->temperature_raw = baro_data.temperature_raw;
                    data->baro_valid = 1;
                    sensor_status.baro_read_count++;

                    /* Save for next time */
                    last_valid_baro_data = baro_data;
                } else if (baro_status == HAL_BUSY) {
                    /* Data not ready yet - use last valid data */
                    data->pressure_raw = last_valid_baro_data.pressure_raw;
                    data->temperature_raw = last_valid_baro_data.temperature_raw;
                    data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
                    /* Don't increment read count - this is cached data */
                } else {
                    /* Real error */
                    data->baro_valid = 0;
                    sensor_status.baro_error_count++;
                }
            } else {
                /* Arbiter busy - use last valid data */
                data->pressure_raw = last_valid_baro_data.pressure_raw;
                data->temperature_raw = last_valid_baro_data.temperature_raw;
                data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
            }
        } else {
            /* Use last valid data during decimation */
            data->pressure_raw = last_valid_baro_data.pressure_raw;
            data->temperature_raw = last_valid_baro_data.temperature_raw;
            data->baro_valid = (last_valid_baro_data.pressure_raw != 0) ? 1 : 0;
        }
    } else {
        data->baro_valid = 0;
    }

    /* DECIMATE High-G (lowest priority, shock detection only) */
    if (config.use_high_g) {
        if (highg_decimation_counter++ >= highg_decimation_factor) {
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
    VALIDATE_PTR(data);

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
    VALIDATE_PTR_VOID(raw);
    VALIDATE_PTR_VOID(scaled);

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
        scaled->temperature_c = (float)raw->temperature_raw * (1.0f / BMP581_TEMP_SCALE);
        scaled->altitude_m = CalculateAltitude(scaled->pressure_pa, STANDARD_SEA_LEVEL_PA);
    } else {
        scaled->pressure_pa = 0.0f;
        scaled->temperature_c = ICM42688_TEMP_OFFSET;
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

/**
 * @brief Convert raw sensor data to Mahony filter units
 * @param raw: Raw sensor data
 * @param gx, gy, gz: Gyro in rad/s (output)
 * @param ax, ay, az: Accel in m/s² (output)
 * @param mx, my, mz: Mag in µT (output)
 */
void SensorManager_GetMahonyData(const SensorManager_RawData_t *raw,
                                  float *gx, float *gy, float *gz,
                                  float *ax, float *ay, float *az,
                                  float *mx, float *my, float *mz) {
    VALIDATE_PTR_VOID(raw);
    VALIDATE_PTR_VOID(gx);
    VALIDATE_PTR_VOID(gy);
    VALIDATE_PTR_VOID(gz);
    VALIDATE_PTR_VOID(ax);
    VALIDATE_PTR_VOID(ay);
    VALIDATE_PTR_VOID(az);
    VALIDATE_PTR_VOID(mx);
    VALIDATE_PTR_VOID(my);
    VALIDATE_PTR_VOID(mz);

    // Gyro: Convert from LSB to rad/s
    *gx = (raw->gyro_x / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;
    *gy = (raw->gyro_y / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;
    *gz = (raw->gyro_z / ICM42688_GYRO_SCALE_2000DPS) * DEG_TO_RAD;

    // Accel: Convert from LSB to m/s²
    *ax = (raw->accel_x / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;
    *ay = (raw->accel_y / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;
    *az = (raw->accel_z / ICM42688_ACCEL_SCALE_16G) * GRAVITY_MSS;

    // Mag: Convert from LSB to µT (microTesla)
    *mx = (raw->mag_x / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;
    *my = (raw->mag_y / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;
    *mz = (raw->mag_z / MMC5983MA_MAG_SCALE) * GAUSS_TO_MICROTESLA;
}
