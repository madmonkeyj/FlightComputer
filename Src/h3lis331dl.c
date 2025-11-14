/**
  ******************************************************************************
  * @file    h3lis331dl.c
  * @brief   H3LIS331DL high-g accelerometer - DMA with arbiter (lowest priority)
  ******************************************************************************
  */

#include "h3lis331dl.h"
#include "i2c_dma_arbiter.h"
#include <string.h>

/* Private variables */
static H3LIS331DL_FullScale_t current_full_scale = H3LIS331DL_FS_100G;
static volatile uint32_t highg_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) highg_rx_buffer[8];
static volatile bool highg_dma_busy = false;

/* Conversion factors (left-justified 12-bit format) */
#define H3LIS331DL_CONV_100G    3.0625f    // 49 mg/digit / 16
#define H3LIS331DL_CONV_200G    6.125f     // 98 mg/digit / 16
#define H3LIS331DL_CONV_400G    12.25f     // 195 mg/digit / 16
#define H3LIS331DL_I2C_TIMEOUT  100

/* Private function prototypes */
static HAL_StatusTypeDef H3LIS331DL_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef H3LIS331DL_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef H3LIS331DL_ReadMultipleRegisters(uint8_t reg, uint8_t *buffer, uint16_t length);
static float H3LIS331DL_ConvertToG(int16_t raw_value);

/* Private functions */
static HAL_StatusTypeDef H3LIS331DL_WriteRegister(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t data[2] = {reg, value};

    status = HAL_I2C_Master_Transmit(&hi2c1, H3LIS331DL_I2C_ADDR, data, 2, H3LIS331DL_I2C_TIMEOUT);
    return status;
}

static HAL_StatusTypeDef H3LIS331DL_ReadRegister(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, H3LIS331DL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, H3LIS331DL_I2C_TIMEOUT);
}

static HAL_StatusTypeDef H3LIS331DL_ReadMultipleRegisters(uint8_t reg, uint8_t *buffer, uint16_t length) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;
    uint8_t reg_addr = reg | H3LIS331DL_I2C_AUTO_INCREMENT;

    /* Wait if previous DMA busy */
    wait_start = HAL_GetTick();
    while (highg_dma_busy) {
        if (HAL_GetTick() - wait_start > H3LIS331DL_I2C_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    highg_dma_busy = true;

    /* Request DMA transfer through arbiter (LOWEST PRIORITY) */
    status = I2C_DMA_Arbiter_RequestTransfer(
        &hi2c1,
        I2C_DMA_DEVICE_HIGHG,
        H3LIS331DL_I2C_ADDR,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        highg_rx_buffer,
        length,
        H3LIS331DL_DMA_Complete_Callback
    );

    if (status == HAL_BUSY) {
        /* Arbiter is busy - fail fast (shock detection is not time-critical) */
        highg_dma_busy = false;
        return HAL_BUSY;
    }

    if (status != HAL_OK) {
        highg_dma_busy = false;
        return status;
    }

    /* Wait for completion */
    uint32_t timeout = HAL_GetTick() + H3LIS331DL_I2C_TIMEOUT;
    while (highg_dma_busy && HAL_GetTick() < timeout) {
        __NOP();
    }

    if (highg_dma_busy) {
        HAL_I2C_Master_Abort_IT(&hi2c1, H3LIS331DL_I2C_ADDR);
        highg_dma_busy = false;
        return HAL_TIMEOUT;
    }

    /* Copy from DMA buffer */
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = highg_rx_buffer[i];
    }

    return HAL_OK;
}

static float H3LIS331DL_ConvertToG(int16_t raw_value)
{
    float accel_mg;

    switch (current_full_scale) {
        case H3LIS331DL_FS_100G:
            accel_mg = (float)raw_value * H3LIS331DL_CONV_100G;
            break;
        case H3LIS331DL_FS_200G:
            accel_mg = (float)raw_value * H3LIS331DL_CONV_200G;
            break;
        case H3LIS331DL_FS_400G:
            accel_mg = (float)raw_value * H3LIS331DL_CONV_400G;
            break;
        default:
            accel_mg = (float)raw_value * H3LIS331DL_CONV_100G;
            break;
    }

    return accel_mg / 1000.0f;
}

/* Public functions */
void H3LIS331DL_DMA_Complete_Callback(void) {
    highg_dma_busy = false;
    highg_dma_callback_count++;
}

uint32_t H3LIS331DL_GetDMACallbackCount(void) {
    return highg_dma_callback_count;
}

HAL_StatusTypeDef H3LIS331DL_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i;
    uint8_t reg_value;

    /* Verify WHO_AM_I */
    status = H3LIS331DL_ReadWhoAmI(&who_am_i);
    if (status != HAL_OK || who_am_i != H3LIS331DL_WHO_AM_I_VALUE) {
        return HAL_ERROR;
    }

    /* Configure CTRL_REG1: Normal mode, 400 Hz ODR, all axes enabled */
    reg_value = H3LIS331DL_CTRL1_PM_NORMAL |
                H3LIS331DL_CTRL1_DR_400HZ |
                H3LIS331DL_CTRL1_XEN |
                H3LIS331DL_CTRL1_YEN |
                H3LIS331DL_CTRL1_ZEN;

    status = H3LIS331DL_WriteRegister(H3LIS331DL_REG_CTRL_REG1, reg_value);
    if (status != HAL_OK) {
        return status;
    }

    /* Configure CTRL_REG4: Block data update, ±100g full scale */
    reg_value = H3LIS331DL_CTRL4_BDU | H3LIS331DL_CTRL4_FS_100G;
    status = H3LIS331DL_WriteRegister(H3LIS331DL_REG_CTRL_REG4, reg_value);
    if (status != HAL_OK) {
        return status;
    }

    current_full_scale = H3LIS331DL_FS_100G;
    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef H3LIS331DL_ReadWhoAmI(uint8_t *who_am_i)
{
    return H3LIS331DL_ReadRegister(H3LIS331DL_REG_WHO_AM_I, who_am_i);
}

HAL_StatusTypeDef H3LIS331DL_ReadSensorData(H3LIS331DL_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];

    status = H3LIS331DL_ReadMultipleRegisters(H3LIS331DL_REG_OUT_X_L, buffer, 6);

    if (status == HAL_OK) {
        data->accel_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        data->accel_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        data->accel_z = (int16_t)((buffer[5] << 8) | buffer[4]);

        data->accel_x_g = H3LIS331DL_ConvertToG(data->accel_x);
        data->accel_y_g = H3LIS331DL_ConvertToG(data->accel_y);
        data->accel_z_g = H3LIS331DL_ConvertToG(data->accel_z);
    }

    return status;
}

HAL_StatusTypeDef H3LIS331DL_GetAccelData(int16_t *x, int16_t *y, int16_t *z)
{
    H3LIS331DL_Data_t data;
    HAL_StatusTypeDef status;

    status = H3LIS331DL_ReadSensorData(&data);

    if (status == HAL_OK) {
        *x = data.accel_x;
        *y = data.accel_y;
        *z = data.accel_z;
    }

    return status;
}

HAL_StatusTypeDef H3LIS331DL_GetAccelData_g(float *x_g, float *y_g, float *z_g)
{
    H3LIS331DL_Data_t data;
    HAL_StatusTypeDef status;

    status = H3LIS331DL_ReadSensorData(&data);

    if (status == HAL_OK) {
        *x_g = data.accel_x_g;
        *y_g = data.accel_y_g;
        *z_g = data.accel_z_g;
    }

    return status;
}

HAL_StatusTypeDef H3LIS331DL_SetFullScale(H3LIS331DL_FullScale_t fs)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    status = H3LIS331DL_ReadRegister(H3LIS331DL_REG_CTRL_REG4, &reg_value);
    if (status != HAL_OK) {
        return status;
    }

    reg_value &= ~0x30;

    switch (fs) {
        case H3LIS331DL_FS_100G:
            reg_value |= H3LIS331DL_CTRL4_FS_100G;
            break;
        case H3LIS331DL_FS_200G:
            reg_value |= H3LIS331DL_CTRL4_FS_200G;
            break;
        case H3LIS331DL_FS_400G:
            reg_value |= H3LIS331DL_CTRL4_FS_400G;
            break;
        default:
            return HAL_ERROR;
    }

    status = H3LIS331DL_WriteRegister(H3LIS331DL_REG_CTRL_REG4, reg_value);

    if (status == HAL_OK) {
        current_full_scale = fs;
    }

    return status;
}

HAL_StatusTypeDef H3LIS331DL_SetOutputDataRate(H3LIS331DL_ODR_t odr)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    status = H3LIS331DL_ReadRegister(H3LIS331DL_REG_CTRL_REG1, &reg_value);
    if (status != HAL_OK) {
        return status;
    }

    reg_value &= ~0x18;

    switch (odr) {
        case H3LIS331DL_ODR_50HZ:
            reg_value |= H3LIS331DL_CTRL1_DR_50HZ;
            break;
        case H3LIS331DL_ODR_100HZ:
            reg_value |= H3LIS331DL_CTRL1_DR_100HZ;
            break;
        case H3LIS331DL_ODR_400HZ:
            reg_value |= H3LIS331DL_CTRL1_DR_400HZ;
            break;
        case H3LIS331DL_ODR_1000HZ:
            reg_value |= H3LIS331DL_CTRL1_DR_1000HZ;
            break;
        default:
            return HAL_ERROR;
    }

    return H3LIS331DL_WriteRegister(H3LIS331DL_REG_CTRL_REG1, reg_value);
}

HAL_StatusTypeDef H3LIS331DL_CheckDataReady(bool *ready)
{
    HAL_StatusTypeDef status;
    uint8_t status_reg;

    status = H3LIS331DL_ReadRegister(H3LIS331DL_REG_STATUS, &status_reg);

    if (status == HAL_OK) {
        *ready = (status_reg & H3LIS331DL_STATUS_ZYXDA) ? true : false;
    }

    return status;
}

HAL_StatusTypeDef H3LIS331DL_SoftReset(void)
{
    HAL_StatusTypeDef status;

    status = H3LIS331DL_WriteRegister(H3LIS331DL_REG_CTRL_REG2, H3LIS331DL_CTRL2_BOOT);

    if (status == HAL_OK) {
        HAL_Delay(10);
    }

    return status;
}
