/**
  ******************************************************************************
  * @file    bmp581.c
  * @brief   BMP581 high-precision barometer - DMA with arbiter at 100Hz
  ******************************************************************************
  */

#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include <math.h>

/* Private variables */
static volatile uint32_t baro_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) baro_rx_buffer[8];
static volatile bool baro_dma_busy = false;

/* Constants */
#define BMP581_I2C_TIMEOUT      100

/* Conversion constants (from datasheet) */
#define BMP581_PRESS_SCALE      (1.0f / 64.0f)     // Pa per LSB
#define BMP581_TEMP_SCALE       (1.0f / 65536.0f)  // °C per LSB

/* Private function prototypes */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BMP581_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
static float BMP581_ConvertPressure(int32_t raw_press);
static float BMP581_ConvertTemperature(int32_t raw_temp);
static float BMP581_CalculateAltitude(float pressure_pa, float sea_level_pa);

/* Private functions */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, BMP581_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP581_ReadRegister(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, BMP581_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;

    /* Wait if our previous DMA is still busy */
    wait_start = HAL_GetTick();
    while (baro_dma_busy) {
        if (HAL_GetTick() - wait_start > BMP581_I2C_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    /* Mark our device as busy */
    baro_dma_busy = true;

    /* Request DMA transfer through arbiter */
    status = I2C_DMA_Arbiter_RequestTransfer(
        &hi2c1,
        I2C_DMA_DEVICE_BARO,
        BMP581_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        baro_rx_buffer,
        len,
        BMP581_DMA_Complete_Callback
    );

    if (status == HAL_BUSY) {
        /* Arbiter is busy - fail fast to maintain loop rate */
        /* BARO has medium priority, so it's OK to skip reads */
        baro_dma_busy = false;
        return HAL_BUSY;
    }

    if (status != HAL_OK) {
        baro_dma_busy = false;
        return status;
    }

    /* Wait for completion */
    uint32_t timeout = HAL_GetTick() + BMP581_I2C_TIMEOUT;
    while (baro_dma_busy && HAL_GetTick() < timeout) {
        __NOP();
    }

    if (baro_dma_busy) {
        HAL_I2C_Master_Abort_IT(&hi2c1, BMP581_I2C_ADDR);
        baro_dma_busy = false;
        return HAL_TIMEOUT;
    }

    /* Copy from DMA buffer */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = baro_rx_buffer[i];
    }

    return HAL_OK;
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

    HAL_Delay(2);

    /* Verify chip ID */
    status = BMP581_ReadChipID(&chip_id);
    if (status != HAL_OK || chip_id != BMP581_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }

    /* Soft reset */
    BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    HAL_Delay(10);

    /* Configure oversampling: 8x pressure, 2x temperature */
    /* OSR_CONFIG = [5:3] temp_osr, [2:0] press_osr */
    uint8_t osr_config = (BMP581_OSR_T_2X << BMP581_OSR_T_SHIFT) | BMP581_OSR_P_8X;
    status = BMP581_WriteRegister(BMP581_REG_OSR_CONFIG, osr_config);
    if (status != HAL_OK) return status;

    /* Configure IIR filter: coefficient 4 (good for dynamic applications) */
    /* Set pressure IIR in bits [2:0], temp IIR in bits [6:4] */
    uint8_t iir_config = BMP581_IIR_COEFF_3 | (BMP581_IIR_COEFF_3 << 4);
    status = BMP581_WriteRegister(BMP581_REG_DSP_IIR, iir_config);
    if (status != HAL_OK) return status;

    /* Enable IIR filter in DSP_CONFIG */
    status = BMP581_WriteRegister(BMP581_REG_DSP_CONFIG, 0x07);  // Enable IIR for both P and T
    if (status != HAL_OK) return status;

    /* Configure ODR: 100 Hz in normal (continuous) mode - increased for EKF */
    /* ODR_CONFIG = [6:5] mode, [4:0] odr */
    uint8_t odr_config = (BMP581_MODE_NORMAL << BMP581_MODE_SHIFT) | BMP581_ODR_100_HZ;
    status = BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, odr_config);
    if (status != HAL_OK) return status;

    HAL_Delay(50);  // Wait for first measurement

    return HAL_OK;
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
