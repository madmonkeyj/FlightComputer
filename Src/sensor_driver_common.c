/**
  ******************************************************************************
  * @file    sensor_driver_common.c
  * @brief   Common utilities for I2C sensor drivers - implementation
  ******************************************************************************
  */

#include "sensor_driver_common.h"

/**
 * @brief Read registers via I2C with DMA and arbiter
 *
 * Complete DMA read implementation with:
 * - Wait for previous DMA completion
 * - Configurable retry on HAL_BUSY
 * - Timeout protection
 * - Automatic buffer copy
 *
 * @param driver Driver configuration structure
 * @param reg Starting register address
 * @param buffer Output buffer for read data
 * @param len Number of bytes to read
 * @return HAL status
 */
HAL_StatusTypeDef I2C_Sensor_ReadRegisters_DMA(
    const I2C_Sensor_Driver_t *driver,
    uint8_t reg,
    uint8_t *buffer,
    uint8_t len
) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;

    /* Validate inputs */
    if (driver == NULL || buffer == NULL || len == 0) {
        return HAL_ERROR;
    }

    /* Wait if previous DMA transfer is still busy */
    wait_start = HAL_GetTick();
    while (*driver->dma_busy_flag) {
        if (HAL_GetTick() - wait_start > driver->timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    /* Set busy flag before starting transfer */
    *driver->dma_busy_flag = true;

    /* Request DMA transfer through arbiter with retry logic */
    for (uint8_t retry = 0; retry <= driver->max_retries; retry++) {
        status = I2C_DMA_Arbiter_RequestTransfer(
            driver->hi2c,
            driver->arbiter_device,
            driver->dev_address,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            driver->dma_buffer,
            len,
            driver->dma_callback
        );

        /* Success - break out of retry loop */
        if (status == HAL_OK) {
            break;
        }

        /* If not busy or this is the last retry, give up */
        if (status != HAL_BUSY || retry == driver->max_retries) {
            *driver->dma_busy_flag = false;
            return status;
        }

        /* Arbiter busy - wait before retry */
        HAL_Delay(driver->retry_delay_ms);
    }

    /* Transfer failed to start */
    if (status != HAL_OK) {
        *driver->dma_busy_flag = false;
        return status;
    }

    /* Wait for DMA completion */
    uint32_t timeout = HAL_GetTick() + driver->timeout_ms;
    while (*driver->dma_busy_flag && HAL_GetTick() < timeout) {
        __NOP();  /* Busy wait */
    }

    /* Check for timeout */
    if (*driver->dma_busy_flag) {
        /* Timeout - abort transfer and clear flag */
        HAL_I2C_Master_Abort_IT(driver->hi2c, driver->dev_address);
        *driver->dma_busy_flag = false;
        return HAL_TIMEOUT;
    }

    /* Copy data from DMA buffer to output buffer */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = driver->dma_buffer[i];
    }

    return HAL_OK;
}
