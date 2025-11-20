/**
  ******************************************************************************
  * @file    bmp581.c
  * @brief   BMP581 high-precision barometer - DMA with arbiter at 100Hz
  ******************************************************************************
  */

#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include "sensor_driver_common.h"
#include <math.h>
#include "debug_utils.h"

/* Private variables */
static volatile uint32_t baro_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) baro_rx_buffer[8];

/**
 * @brief DMA busy flag for barometer reads
 * @note Volatile is sufficient for single-core Cortex-M4
 * @note DMA callbacks run in interrupt context but don't preempt themselves
 * @note Flag is set before DMA request and cleared in completion callback
 */
static volatile bool baro_dma_busy = false;

/* Constants */
#define BMP581_I2C_TIMEOUT      100

/* Conversion constants (from datasheet) */
#define BMP581_PRESS_SCALE      (1.0f / 64.0f)     // Pa per LSB
#define BMP581_TEMP_SCALE       (1.0f / 65536.0f)  // °C per LSB

/* Common driver configuration for DMA reads */
static const I2C_Sensor_Driver_t baro_driver = {
    .hi2c = &hi2c1,
    .dev_address = BMP581_I2C_ADDR,
    .arbiter_device = I2C_DMA_DEVICE_BARO,
    .dma_busy_flag = &baro_dma_busy,
    .dma_buffer = baro_rx_buffer,
    .dma_callback = BMP581_DMA_Complete_Callback,
    .timeout_ms = BMP581_I2C_TIMEOUT,
    .max_retries = 0,  /* Fail fast to maintain loop rate */
    .retry_delay_ms = 0
};

/* Private function prototypes */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
static float BMP581_ConvertPressure(int32_t raw_press);
static float BMP581_ConvertTemperature(int32_t raw_temp);
static float BMP581_CalculateAltitude(float pressure_pa, float sea_level_pa);

/* Private functions */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, BMP581_I2C_TIMEOUT);
}

HAL_StatusTypeDef BMP581_ReadRegister(uint8_t reg, uint8_t *value) {  // Remove "static" here too
    // Blocking reads for single bytes (used in init) are also fine
    return HAL_I2C_Mem_Read(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, BMP581_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    return I2C_Sensor_ReadRegisters_DMA(&baro_driver, reg, buffer, len);
}

static float BMP581_ConvertPressure(int32_t raw_press) {
    /* Convert 24-bit raw pressure to Pascals */
    return (float)raw_press * BMP581_PRESS_SCALE;
}

static float BMP581_ConvertTemperature(int32_t raw_temp) {
    /* Convert 24-bit raw temperature to Celsius */
    return (float)raw_temp * BMP581_TEMP_SCALE;
}

static float BMP581_CalculateAltitude(float pressure_pa, float sea_level_pa) {
    /* Standard barometric formula */
    /* h = 44330 * (1 - (P/P0)^0.1903) */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

/* Public functions */
void BMP581_DMA_Complete_Callback(void) {
    baro_dma_busy = false;
    baro_dma_callback_count++;
}

uint32_t BMP581_GetDMACallbackCount(void) {
    return baro_dma_callback_count;
}

HAL_StatusTypeDef BMP581_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t chip_id;
    uint8_t reg_data;

    // Check if device is present
    status = HAL_I2C_IsDeviceReady(&hi2c1, BMP581_I2C_ADDR, 10, 100);
    if (status != HAL_OK) return status;

    HAL_Delay(2);

    // Read and verify chip ID
    status = BMP581_ReadChipID(&chip_id);
    if (status != HAL_OK || chip_id != BMP581_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }

    // Soft reset
    status = BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    // Wait for power-up (NVM ready)
    uint8_t status_reg;
    uint32_t timeout = HAL_GetTick() + 100;
    while (HAL_GetTick() < timeout) {
        status = BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg);
        if (status == HAL_OK && (status_reg & BMP581_STATUS_NVM_RDY)) {
            break;
        }
        HAL_Delay(5);
    }

    // Configure OSR with PRESSURE ENABLE bit set
    reg_data = (BMP581_OSR_T_2X << BMP581_OSR_T_SHIFT) |
               BMP581_OSR_P_8X |
               (1 << BMP581_PRESS_EN_POS);
    status = BMP581_WriteRegister(BMP581_REG_OSR_CONFIG, reg_data);
    if (status != HAL_OK) return status;

    // Configure IIR filter
    uint8_t iir_config = BMP581_IIR_COEFF_3 | (BMP581_IIR_COEFF_3 << 4);
    status = BMP581_WriteRegister(BMP581_REG_DSP_IIR, iir_config);
    if (status != HAL_OK) return status;

    // Enable data ready interrupt source
    status = BMP581_WriteRegister(BMP581_REG_INT_SOURCE, 0x01);
    if (status != HAL_OK) return status;

    // Configure ODR with proper bit positions
    uint8_t odr_config = (BMP581_DEEP_DISABLED << BMP581_DEEP_DISABLE_POS) |
                         (BMP581_ODR_50_HZ << 2) |
                         BMP581_MODE_NORMAL;
    status = BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, odr_config);
    if (status != HAL_OK) return status;

    // Wait for first valid data
    status = BMP581_WaitForDataReady(100);

    return HAL_OK;
}

HAL_StatusTypeDef BMP581_WaitForDataReady(uint32_t timeout_ms) {
    uint8_t status_reg;
    uint32_t timeout = HAL_GetTick() + timeout_ms;

    while (HAL_GetTick() < timeout) {
        if (BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg) == HAL_OK) {
            if ((status_reg & (BMP581_STATUS_DRDY_PRESS | BMP581_STATUS_DRDY_TEMP)) ==
                (BMP581_STATUS_DRDY_PRESS | BMP581_STATUS_DRDY_TEMP)) {
                return HAL_OK;
            }
        }
        HAL_Delay(10);
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef BMP581_ReadChipID(uint8_t *chip_id) {
    return BMP581_ReadRegister(BMP581_REG_CHIP_ID, chip_id);
}

HAL_StatusTypeDef BMP581_ReadSensorData(BMP581_Data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    /* Read temperature (3 bytes) + pressure (3 bytes) starting from TEMP_DATA_XLSB */
    status = BMP581_ReadRegisters(BMP581_REG_TEMP_DATA_XLSB, raw_data, 6);

    if (status == HAL_OK) {
        /* Combine 24-bit temperature (XLSB, LSB, MSB) - signed */
        data->temperature_raw = ((int32_t)raw_data[2] << 16) |
                                ((int32_t)raw_data[1] << 8) |
                                ((int32_t)raw_data[0]);

        /* Sign extend from 24-bit to 32-bit */
        if (data->temperature_raw & 0x00800000) {
            data->temperature_raw |= 0xFF000000;
        }

        /* Combine 24-bit pressure (XLSB, LSB, MSB) - signed */
        data->pressure_raw = ((int32_t)raw_data[5] << 16) |
                             ((int32_t)raw_data[4] << 8) |
                             ((int32_t)raw_data[3]);

        /* Sign extend from 24-bit to 32-bit */
        if (data->pressure_raw & 0x00800000) {
            data->pressure_raw |= 0xFF000000;
        }

        /* Check if pressure data is invalid (0x7F7F7F) */
        if ((data->pressure_raw & 0x00FFFFFF) == 0x007F7F7F) {
            return HAL_BUSY;
        }

        /* Convert to engineering units */
        data->temperature_c = BMP581_ConvertTemperature(data->temperature_raw);
        data->pressure_pa = BMP581_ConvertPressure(data->pressure_raw);

        /* Calculate altitude (assumes standard sea level pressure) */
        data->altitude_m = BMP581_CalculateAltitude(data->pressure_pa, 101325.0f);
    }

    return status;
}

HAL_StatusTypeDef BMP581_GetPressure(float *pressure_pa) {
    BMP581_Data_t data;
    HAL_StatusTypeDef status;

    status = BMP581_ReadSensorData(&data);
    if (status == HAL_OK) {
        *pressure_pa = data.pressure_pa;
    }

    return status;
}

HAL_StatusTypeDef BMP581_GetTemperature(float *temperature_c) {
    BMP581_Data_t data;
    HAL_StatusTypeDef status;

    status = BMP581_ReadSensorData(&data);
    if (status == HAL_OK) {
        *temperature_c = data.temperature_c;
    }

    return status;
}

HAL_StatusTypeDef BMP581_GetAltitude(float *altitude_m, float sea_level_pa) {
    float pressure_pa;
    HAL_StatusTypeDef status;

    status = BMP581_GetPressure(&pressure_pa);
    if (status == HAL_OK) {
        *altitude_m = BMP581_CalculateAltitude(pressure_pa, sea_level_pa);
    }

    return status;
}

HAL_StatusTypeDef BMP581_CheckDataReady(bool *ready) {
    HAL_StatusTypeDef status;
    uint8_t status_reg;

    status = BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg);
    if (status == HAL_OK) {
        /* Check if both pressure and temperature data are ready */
        *ready = ((status_reg & (BMP581_STATUS_DRDY_PRESS | BMP581_STATUS_DRDY_TEMP)) ==
                  (BMP581_STATUS_DRDY_PRESS | BMP581_STATUS_DRDY_TEMP));
    }

    return status;
}

HAL_StatusTypeDef BMP581_SetODR(BMP581_ODR_t odr) {
    HAL_StatusTypeDef status;
    uint8_t odr_config;

    /* Read current ODR_CONFIG to preserve mode bits */
    status = BMP581_ReadRegister(BMP581_REG_ODR_CONFIG, &odr_config);
    if (status != HAL_OK) return status;

    /* Clear ODR bits [4:0] and set new ODR */
    odr_config = (odr_config & 0xE0) | (odr & 0x1F);

    return BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, odr_config);
}

HAL_StatusTypeDef BMP581_SetOversampling(BMP581_OSR_t press_osr, BMP581_OSR_t temp_osr) {
    uint8_t osr_config = ((temp_osr & 0x07) << BMP581_OSR_T_SHIFT) | (press_osr & 0x07);
    return BMP581_WriteRegister(BMP581_REG_OSR_CONFIG, osr_config);
}

HAL_StatusTypeDef BMP581_SetIIRFilter(BMP581_IIR_t iir_coeff) {
    /* Set same IIR coefficient for both pressure and temperature */
    uint8_t iir_config = (iir_coeff & 0x07) | ((iir_coeff & 0x07) << 4);
    return BMP581_WriteRegister(BMP581_REG_DSP_IIR, iir_config);
}

HAL_StatusTypeDef BMP581_SoftReset(void) {
    HAL_StatusTypeDef status;

    status = BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    if (status == HAL_OK) {
        HAL_Delay(10);  // Wait for reset to complete
    }

    return status;
}
