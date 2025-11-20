/**
  ******************************************************************************
  * @file    i2c_dma_arbiter.h
  * @brief   Priority-based I2C DMA arbiter for multiple sensors on one bus
  * @note    Manages MAG (highest priority), BARO (medium), High-G (lowest)
  ******************************************************************************
  */

#ifndef I2C_DMA_ARBITER_H_
#define I2C_DMA_ARBITER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* Device priority levels (lower number = higher priority) */
typedef enum {
    I2C_DMA_DEVICE_MAG = 0,      // Highest priority - 1kHz updates for Mahony
    I2C_DMA_DEVICE_BARO = 1,     // Medium priority - 100Hz decimated
    I2C_DMA_DEVICE_HIGHG = 2,    // Lowest priority - 400Hz shock detection
    I2C_DMA_DEVICE_COUNT = 3
} I2C_DMA_Device_t;

/* Arbiter statistics (optional, for debugging) */
typedef struct {
    uint32_t mag_requests;
    uint32_t baro_requests;
    uint32_t highg_requests;
    uint32_t mag_conflicts;
    uint32_t baro_conflicts;
    uint32_t highg_conflicts;
    uint32_t total_transfers;
} I2C_DMA_Arbiter_Stats_t;

/* Callback function type */
typedef void (*I2C_DMA_Callback_t)(void);

/**
 * @brief Initialize the I2C DMA arbiter
 */
void I2C_DMA_Arbiter_Init(void);

/**
 * @brief Request a DMA transfer (priority-based)
 * @param hi2c: I2C handle
 * @param device: Device requesting transfer (determines priority)
 * @param dev_address: I2C device address (7-bit, shifted)
 * @param mem_address: Register address to read from
 * @param mem_size: Memory address size (I2C_MEMADD_SIZE_8BIT or 16BIT)
 * @param buffer: Buffer to receive data
 * @param size: Number of bytes to read
 * @param callback: Completion callback function
 * @retval HAL_OK if started, HAL_BUSY if arbiter occupied by higher priority
 */
HAL_StatusTypeDef I2C_DMA_Arbiter_RequestTransfer(
    I2C_HandleTypeDef *hi2c,
    I2C_DMA_Device_t device,
    uint16_t dev_address,
    uint16_t mem_address,
    uint16_t mem_size,
    uint8_t *buffer,
    uint16_t size,
    I2C_DMA_Callback_t callback
);

/**
 * @brief Check if arbiter is busy
 * @param hi2c: I2C handle
 * @retval true if arbiter is busy, false if available
 */
bool I2C_DMA_Arbiter_IsBusy(I2C_HandleTypeDef *hi2c);

/**
 * @brief Get current device using the bus
 * @param hi2c: I2C handle
 * @retval Current device, or I2C_DMA_DEVICE_COUNT if idle
 */
I2C_DMA_Device_t I2C_DMA_Arbiter_GetCurrentDevice(I2C_HandleTypeDef *hi2c);

/**
 * @brief Handle DMA transfer completion (called from HAL callback)
 * @param hi2c: I2C handle
 */
void I2C_DMA_Arbiter_HandleComplete(I2C_HandleTypeDef *hi2c);

/**
 * @brief Handle DMA transfer error (called from HAL callback)
 * @param hi2c: I2C handle
 */
void I2C_DMA_Arbiter_HandleError(I2C_HandleTypeDef *hi2c);

/**
 * @brief Get arbiter statistics (for debugging)
 * @param stats: Pointer to statistics structure
 */
void I2C_DMA_Arbiter_GetStats(I2C_DMA_Arbiter_Stats_t *stats);

/**
 * @brief Reset arbiter statistics
 */
void I2C_DMA_Arbiter_ResetStats(void);

#endif /* I2C_DMA_ARBITER_H_ */
