/**
  ******************************************************************************
  * @file    mmc5983ma.c
  * @brief   MMC5983MA driver - Production version (DMA only)
  ******************************************************************************
  */

#include "mmc5983ma.h"

/* Private variables */
static volatile uint32_t mag_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) mag_rx_buffer[9];
static volatile bool mag_dma_busy = false;

/* Constants */
#define MMC5983MA_I2C_TIMEOUT    100

/* Private function prototypes */
static HAL_StatusTypeDef MMC5983MA_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef MMC5983MA_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef MMC5983MA_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

/* Private functions */
static HAL_StatusTypeDef MMC5983MA_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, MMC5983MA_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, MMC5983MA_I2C_TIMEOUT);
}

static HAL_StatusTypeDef MMC5983MA_ReadRegister(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, MMC5983MA_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, MMC5983MA_I2C_TIMEOUT);
}

static HAL_StatusTypeDef MMC5983MA_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;

    /* Wait if previous DMA busy */
    wait_start = HAL_GetTick();
    while (mag_dma_busy) {
        if (HAL_GetTick() - wait_start > MMC5983MA_I2C_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    mag_dma_busy = true;

    /* Start DMA transfer */
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, MMC5983MA_I2C_ADDR, reg,
                                   I2C_MEMADD_SIZE_8BIT, mag_rx_buffer, len);

    if (status != HAL_OK) {
        mag_dma_busy = false;
        return status;
    }

    /* Wait for completion */
    uint32_t timeout = HAL_GetTick() + MMC5983MA_I2C_TIMEOUT;
    while (mag_dma_busy && HAL_GetTick() < timeout) {
        __NOP();
    }

    if (mag_dma_busy) {
        HAL_I2C_Master_Abort_IT(&hi2c1, MMC5983MA_I2C_ADDR);
        mag_dma_busy = false;
        return HAL_TIMEOUT;
    }

    /* Copy from DMA buffer */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = mag_rx_buffer[i];
    }

    return HAL_OK;
}

/* Public functions */
void MMC5983MA_DMA_Complete_Callback(void) {
    mag_dma_busy = false;
    mag_dma_callback_count++;
}

uint32_t MMC5983MA_GetDMACallbackCount(void) {
    return mag_dma_callback_count;
}

HAL_StatusTypeDef MMC5983MA_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t product_id;

    HAL_Delay(2);

    /* Verify product ID */
    status = MMC5983MA_ReadProductID(&product_id);
    if (status != HAL_OK || product_id != MMC5983MA_PRODUCT_ID) {
        return HAL_ERROR;
    }

    /* Software reset */
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_1, MMC5983MA_CTRL1_SW_RST);
    HAL_Delay(10);

    /* Configure for 1kHz continuous mode */
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_0, MMC5983MA_CTRL0_AUTO_SR_EN);
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_1, MMC5983MA_CTRL1_BW_800HZ);

    uint8_t ctrl2 = MMC5983MA_CTRL2_CM_FREQ_1000HZ |
                    MMC5983MA_CTRL2_CMM_EN |
                    MMC5983MA_CTRL2_PRD_SET_100 |
                    MMC5983MA_CTRL2_EN_PRD_SET;
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_2, ctrl2);

    /* Initial SET operation */
    MMC5983MA_PerformSET();
    HAL_Delay(1);

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983MA_ReadProductID(uint8_t *product_id) {
    return MMC5983MA_ReadRegister(MMC5983MA_REG_PRODUCT_ID, product_id);
}

HAL_StatusTypeDef MMC5983MA_PerformSET(void) {
    return MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_0,
                                    MMC5983MA_CTRL0_SET | MMC5983MA_CTRL0_AUTO_SR_EN);
}

HAL_StatusTypeDef MMC5983MA_PerformRESET(void) {
    return MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_0,
                                    MMC5983MA_CTRL0_RESET | MMC5983MA_CTRL0_AUTO_SR_EN);
}

HAL_StatusTypeDef MMC5983MA_TriggerMeasurement(void) {
    return MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_0, 
                                    MMC5983MA_CTRL0_TM_M | MMC5983MA_CTRL0_AUTO_SR_EN);
}

HAL_StatusTypeDef MMC5983MA_CheckMeasurementDone(bool *done) {
    HAL_StatusTypeDef status;
    uint8_t status_reg;

    status = MMC5983MA_ReadRegister(MMC5983MA_REG_STATUS, &status_reg);
    if (status == HAL_OK) {
        *done = (status_reg & MMC5983MA_STATUS_MEAS_M_DONE) ? true : false;
    }

    return status;
}

HAL_StatusTypeDef MMC5983MA_ReadSensorData(MMC5983MA_Data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[8];

    status = MMC5983MA_ReadRegisters(MMC5983MA_REG_XOUT_0, raw_data, 8);

    if (status == HAL_OK) {
        /* Combine 18-bit values */
        data->mag_x = ((uint32_t)raw_data[0] << 10) | 
                      ((uint32_t)raw_data[1] << 2) | 
                      ((uint32_t)(raw_data[6] >> 6) & 0x03);

        data->mag_y = ((uint32_t)raw_data[2] << 10) | 
                      ((uint32_t)raw_data[3] << 2) | 
                      ((uint32_t)(raw_data[6] >> 4) & 0x03);

        data->mag_z = ((uint32_t)raw_data[4] << 10) | 
                      ((uint32_t)raw_data[5] << 2) | 
                      ((uint32_t)(raw_data[6] >> 2) & 0x03);

        /* Remove null field offset */
        data->mag_x -= MMC5983MA_NULL_FIELD;
        data->mag_y -= MMC5983MA_NULL_FIELD;
        data->mag_z -= MMC5983MA_NULL_FIELD;

        data->temperature = raw_data[7];
    }

    return status;
}

HAL_StatusTypeDef MMC5983MA_GetMagData(int32_t *x, int32_t *y, int32_t *z) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[7];

    status = MMC5983MA_ReadRegisters(MMC5983MA_REG_XOUT_0, raw_data, 7);

    if (status == HAL_OK) {
        *x = ((uint32_t)raw_data[0] << 10) | 
             ((uint32_t)raw_data[1] << 2) | 
             ((uint32_t)(raw_data[6] >> 6) & 0x03);

        *y = ((uint32_t)raw_data[2] << 10) | 
             ((uint32_t)raw_data[3] << 2) | 
             ((uint32_t)(raw_data[6] >> 4) & 0x03);

        *z = ((uint32_t)raw_data[4] << 10) | 
             ((uint32_t)raw_data[5] << 2) | 
             ((uint32_t)(raw_data[6] >> 2) & 0x03);

        *x -= MMC5983MA_NULL_FIELD;
        *y -= MMC5983MA_NULL_FIELD;
        *z -= MMC5983MA_NULL_FIELD;
    }

    return status;
}

HAL_StatusTypeDef MMC5983MA_GetTemperature(uint8_t *temp) {
    return MMC5983MA_ReadRegister(MMC5983MA_REG_TEMP, temp);
}

HAL_StatusTypeDef MMC5983MA_SelfTest(MMC5983MA_SelfTest_t *result) {
    HAL_StatusTypeDef status;
    int32_t x_normal, y_normal, z_normal;
    int32_t x_pos, y_pos, z_pos;
    int32_t x_neg, y_neg, z_neg;

    /* Normal measurement */
    MMC5983MA_TriggerMeasurement();
    HAL_Delay(10);
    status = MMC5983MA_GetMagData(&x_normal, &y_normal, &z_normal);
    if (status != HAL_OK) return status;

    /* Positive self-test */
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_3, MMC5983MA_CTRL3_ST_ENP);
    HAL_Delay(10);
    MMC5983MA_TriggerMeasurement();
    HAL_Delay(10);
    status = MMC5983MA_GetMagData(&x_pos, &y_pos, &z_pos);
    if (status != HAL_OK) return status;

    /* Negative self-test */
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_3, MMC5983MA_CTRL3_ST_ENM);
    HAL_Delay(10);
    MMC5983MA_TriggerMeasurement();
    HAL_Delay(10);
    status = MMC5983MA_GetMagData(&x_neg, &y_neg, &z_neg);
    if (status != HAL_OK) return status;

    /* Disable self-test */
    MMC5983MA_WriteRegister(MMC5983MA_REG_CONTROL_3, 0x00);

    /* Calculate responses */
    result->x_response = x_pos - x_neg;
    result->y_response = y_pos - y_neg;
    result->z_response = z_pos - z_neg;

    /* Check responses */
    result->x_pass = (result->x_response > 1000 && result->x_response < 50000);
    result->y_pass = (result->y_response > 1000 && result->y_response < 50000);
    result->z_pass = (result->z_response > 1000 && result->z_response < 50000);

    return HAL_OK;
}
