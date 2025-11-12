/**
  ******************************************************************************
  * @file    icm42688.c
  * @brief   ICM-42688-P driver - Production version (DMA only)
  ******************************************************************************
  */

#include "icm42688.h"
#include "gpio.h"

/* Private variables */
static volatile uint32_t imu_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) imu_tx_buffer[16];
static uint8_t __attribute__((aligned(4))) imu_rx_buffer[16];
static volatile bool imu_dma_busy = false;

/* Constants */
#define ICM42688_SPI_TIMEOUT    100

/* Private function prototypes */
static void ICM42688_CS_Low(void);
static void ICM42688_CS_High(void);
static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

/* Private functions */
static void ICM42688_CS_Low(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

static void ICM42688_CS_High(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {reg & 0x7F, value};

    ICM42688_CS_Low();
    status = HAL_SPI_Transmit(&hspi3, tx_data, 2, ICM42688_SPI_TIMEOUT);
    ICM42688_CS_High();

    return status;
}

static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef status;
    uint8_t tx_data = reg | 0x80;

    ICM42688_CS_Low();
    status = HAL_SPI_Transmit(&hspi3, &tx_data, 1, ICM42688_SPI_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi3, value, 1, ICM42688_SPI_TIMEOUT);
    }
    ICM42688_CS_High();

    return status;
}

static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;

    /* Wait if previous DMA busy */
    wait_start = HAL_GetTick();
    while (imu_dma_busy) {
        if (HAL_GetTick() - wait_start > ICM42688_SPI_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    /* Prepare TX buffer */
    imu_tx_buffer[0] = reg | 0x80;
    for (int i = 1; i <= len; i++) {
        imu_tx_buffer[i] = 0xFF;
    }

    imu_dma_busy = true;
    ICM42688_CS_Low();

    /* Start DMA transfer */
    status = HAL_SPI_TransmitReceive_DMA(&hspi3, imu_tx_buffer, imu_rx_buffer, len + 1);

    if (status != HAL_OK) {
        ICM42688_CS_High();
        imu_dma_busy = false;
        return status;
    }

    /* Wait for completion */
    uint32_t timeout = HAL_GetTick() + ICM42688_SPI_TIMEOUT;
    while (imu_dma_busy && HAL_GetTick() < timeout) {
        __NOP();
    }

    if (imu_dma_busy) {
        HAL_SPI_Abort(&hspi3);
        ICM42688_CS_High();
        imu_dma_busy = false;
        return HAL_TIMEOUT;
    }

    /* Copy received data (skip first byte) */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = imu_rx_buffer[i + 1];
    }

    return HAL_OK;
}

/* Public functions */
void ICM42688_DMA_Complete_Callback(void) {
    imu_dma_busy = false;
    imu_dma_callback_count++;
}

uint32_t ICM42688_GetDMACallbackCount(void) {
    return imu_dma_callback_count;
}

HAL_StatusTypeDef ICM42688_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t who_am_i, reg_val;

    ICM42688_CS_High();
    HAL_Delay(2);

    /* Verify WHO_AM_I */
    status = ICM42688_ReadWhoAmI(&who_am_i);
    if (status != HAL_OK || who_am_i != ICM42688_WHO_AM_I_VALUE) {
        return HAL_ERROR;
    }

    /* Soft reset */
    ICM42688_WriteRegister(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(2);

    /* Configure INT_ASYNC_RESET */
    ICM42688_ReadRegister(0x64, &reg_val);
    reg_val &= ~(1 << 4);
    ICM42688_WriteRegister(0x64, reg_val);

    /* Enable sensors in low-noise mode */
    ICM42688_WriteRegister(ICM42688_PWR_MGMT0,
                           ICM42688_PWR_GYRO_MODE_LN | ICM42688_PWR_ACCEL_MODE_LN);
    HAL_Delay(50);

    /* Configure 1kHz ODR, ±16g, ±2000dps */
    ICM42688_WriteRegister(ICM42688_GYRO_CONFIG0,
                           ICM42688_GYRO_FS_2000DPS | ICM42688_ODR_1KHZ);
    ICM42688_WriteRegister(ICM42688_ACCEL_CONFIG0,
                           ICM42688_ACCEL_FS_16G | ICM42688_ODR_1KHZ);

    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *who_am_i) {
    return ICM42688_ReadRegister(ICM42688_WHO_AM_I, who_am_i);
}

HAL_StatusTypeDef ICM42688_ReadSensorData(ICM42688_Data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[14];

    status = ICM42688_ReadRegisters(ICM42688_TEMP_DATA1, raw_data, 14);

    if (status == HAL_OK) {
        data->temperature = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        data->accel_x = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->accel_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        data->accel_z = (int16_t)((raw_data[6] << 8) | raw_data[7]);
        data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
        data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
        data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    }

    return status;
}

HAL_StatusTypeDef ICM42688_GetAccelData(int16_t *x, int16_t *y, int16_t *z) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    status = ICM42688_ReadRegisters(ICM42688_ACCEL_DATA_X1, raw_data, 6);

    if (status == HAL_OK) {
        *x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        *y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        *z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    }

    return status;
}

HAL_StatusTypeDef ICM42688_GetGyroData(int16_t *x, int16_t *y, int16_t *z) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    status = ICM42688_ReadRegisters(ICM42688_GYRO_DATA_X1, raw_data, 6);

    if (status == HAL_OK) {
        *x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        *y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        *z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    }

    return status;
}

HAL_StatusTypeDef ICM42688_GetTemperature(int16_t *temp) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[2];

    status = ICM42688_ReadRegisters(ICM42688_TEMP_DATA1, raw_data, 2);

    if (status == HAL_OK) {
        *temp = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    }

    return status;
}
