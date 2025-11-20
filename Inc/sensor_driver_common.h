/**
  ******************************************************************************
  * @file    sensor_driver_common.h
  * @brief   Common utilities for I2C sensor drivers with DMA and arbiter
  * @note    Eliminates code duplication across MMC5983MA, BMP581, H3LIS331DL
  ******************************************************************************
  */

#ifndef SENSOR_DRIVER_COMMON_H_
#define SENSOR_DRIVER_COMMON_H_

#include "main.h"
#include "i2c.h"
#include "i2c_dma_arbiter.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief I2C sensor driver configuration structure
 * @note Encapsulates all configuration needed for DMA-based I2C reads
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;              /* I2C handle (e.g., &hi2c1) */
    uint16_t dev_address;                 /* I2C device address */
    I2C_DMA_Device_t arbiter_device;      /* Arbiter priority (MAG, BARO, HIGHG) */
    volatile bool *dma_busy_flag;         /* Pointer to driver's DMA busy flag */
    uint8_t *dma_buffer;                  /* Pointer to driver's DMA buffer */
    void (*dma_callback)(void);           /* DMA completion callback */
    uint32_t timeout_ms;                  /* Timeout in milliseconds */
    uint8_t max_retries;                  /* Number of retries on HAL_BUSY (0 = no retry) */
    uint16_t retry_delay_ms;              /* Delay between retries (ms) */
} I2C_Sensor_Driver_t;

/**
 * @brief Read registers via I2C with DMA and arbiter
 *
 * This function implements the complete DMA read flow:
 * 1. Wait for previous DMA transfer to complete (with timeout)
 * 2. Set busy flag
 * 3. Request transfer through arbiter with configurable retries
 * 4. Wait for DMA completion (with timeout)
 * 5. Copy data from DMA buffer to output buffer
 * 6. Handle timeouts and errors
 *
 * @param driver Driver configuration structure
 * @param reg Starting register address
 * @param buffer Output buffer for read data
 * @param len Number of bytes to read
 * @return HAL_OK if successful, HAL_BUSY if arbiter busy, HAL_TIMEOUT if timeout
 *
 * @note This function is blocking - it waits for DMA completion
 * @note High-priority sensors (MAG) should configure max_retries > 0
 * @note Low-priority sensors (BARO, HIGHG) should configure max_retries = 0
 */
HAL_StatusTypeDef I2C_Sensor_ReadRegisters_DMA(
    const I2C_Sensor_Driver_t *driver,
    uint8_t reg,
    uint8_t *buffer,
    uint8_t len
);

/**
 * @brief Read single register via blocking I2C (for initialization)
 *
 * Used during sensor initialization when DMA is not needed.
 * Blocking call with timeout.
 *
 * @param hi2c I2C handle
 * @param dev_address I2C device address
 * @param reg Register address
 * @param value Pointer to store read value
 * @param timeout_ms Timeout in milliseconds
 * @return HAL status
 */
static inline HAL_StatusTypeDef I2C_Sensor_ReadRegister_Blocking(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_address,
    uint8_t reg,
    uint8_t *value,
    uint32_t timeout_ms
) {
    return HAL_I2C_Mem_Read(hi2c, dev_address, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, timeout_ms);
}

/**
 * @brief Write single register via blocking I2C (for initialization)
 *
 * Used during sensor initialization for configuration writes.
 * Blocking call with timeout.
 *
 * @param hi2c I2C handle
 * @param dev_address I2C device address
 * @param reg Register address
 * @param value Value to write
 * @param timeout_ms Timeout in milliseconds
 * @return HAL status
 */
static inline HAL_StatusTypeDef I2C_Sensor_WriteRegister_Blocking(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_address,
    uint8_t reg,
    uint8_t value,
    uint32_t timeout_ms
) {
    return HAL_I2C_Mem_Write(hi2c, dev_address, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, timeout_ms);
}

#endif /* SENSOR_DRIVER_COMMON_H_ */
