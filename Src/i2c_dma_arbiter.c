/**
  ******************************************************************************
  * @file    i2c_dma_arbiter.c
  * @brief   Priority-based I2C DMA arbiter implementation
  ******************************************************************************
  */

#include "i2c_dma_arbiter.h"
#include <string.h>

/* Private types */
typedef struct {
    volatile bool busy;
    volatile I2C_DMA_Device_t current_device;
    I2C_DMA_Callback_t callback;
    I2C_DMA_Arbiter_Stats_t stats;
} I2C_DMA_Arbiter_State_t;

/* Private variables */
static I2C_DMA_Arbiter_State_t arbiter_state = {
    .busy = false,
    .current_device = I2C_DMA_DEVICE_COUNT,
    .callback = NULL
};

/* Private functions */

/**
 * @brief Initialize the arbiter
 */
void I2C_DMA_Arbiter_Init(void) {
    arbiter_state.busy = false;
    arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
    arbiter_state.callback = NULL;
    memset(&arbiter_state.stats, 0, sizeof(I2C_DMA_Arbiter_Stats_t));
}

/**
 * @brief Request a DMA transfer
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
) {
    HAL_StatusTypeDef status;

    /* Update request statistics */
    switch (device) {
        case I2C_DMA_DEVICE_MAG:
            arbiter_state.stats.mag_requests++;
            break;
        case I2C_DMA_DEVICE_BARO:
            arbiter_state.stats.baro_requests++;
            break;
        case I2C_DMA_DEVICE_HIGHG:
            arbiter_state.stats.highg_requests++;
            break;
        default:
            break;
    }

    /* Check if arbiter is busy */
    if (arbiter_state.busy) {
        /* Check priority - only allow preemption by higher priority device */
        if (device >= arbiter_state.current_device) {
            /* Equal or lower priority - deny access */
            switch (device) {
                case I2C_DMA_DEVICE_MAG:
                    arbiter_state.stats.mag_conflicts++;
                    break;
                case I2C_DMA_DEVICE_BARO:
                    arbiter_state.stats.baro_conflicts++;
                    break;
                case I2C_DMA_DEVICE_HIGHG:
                    arbiter_state.stats.highg_conflicts++;
                    break;
                default:
                    break;
            }
            return HAL_BUSY;
        }

        /* Higher priority device - abort current transfer (not implemented for now) */
        /* For simplicity, we just deny access. Preemption adds complexity. */
        switch (device) {
            case I2C_DMA_DEVICE_MAG:
                arbiter_state.stats.mag_conflicts++;
                break;
            case I2C_DMA_DEVICE_BARO:
                arbiter_state.stats.baro_conflicts++;
                break;
            case I2C_DMA_DEVICE_HIGHG:
                arbiter_state.stats.highg_conflicts++;
                break;
            default:
                break;
        }
        return HAL_BUSY;
    }

    /* Arbiter is free - grant access */
    arbiter_state.busy = true;
    arbiter_state.current_device = device;
    arbiter_state.callback = callback;

    /* Start DMA transfer */
    status = HAL_I2C_Mem_Read_DMA(hi2c, dev_address, mem_address,
                                   mem_size, buffer, size);

    if (status != HAL_OK) {
        /* Transfer failed to start - release arbiter */
        arbiter_state.busy = false;
        arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
        arbiter_state.callback = NULL;
        return status;
    }

    /* Transfer started successfully */
    arbiter_state.stats.total_transfers++;
    return HAL_OK;
}

/**
 * @brief Check if arbiter is busy
 */
bool I2C_DMA_Arbiter_IsBusy(I2C_HandleTypeDef *hi2c) {
    (void)hi2c; // Unused parameter
    return arbiter_state.busy;
}

/**
 * @brief Get current device using the bus
 */
I2C_DMA_Device_t I2C_DMA_Arbiter_GetCurrentDevice(I2C_HandleTypeDef *hi2c) {
    (void)hi2c; // Unused parameter
    return arbiter_state.current_device;
}

/**
 * @brief Handle DMA transfer completion
 */
void I2C_DMA_Arbiter_HandleComplete(I2C_HandleTypeDef *hi2c) {
    (void)hi2c; // Unused parameter

    /* Call device-specific callback */
    if (arbiter_state.callback != NULL) {
        arbiter_state.callback();
    }

    /* Release arbiter */
    arbiter_state.busy = false;
    arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
    arbiter_state.callback = NULL;
}

/**
 * @brief Handle DMA transfer error
 */
void I2C_DMA_Arbiter_HandleError(I2C_HandleTypeDef *hi2c) {
    (void)hi2c; // Unused parameter

    /* Call device-specific callback (so device knows transfer failed) */
    if (arbiter_state.callback != NULL) {
        arbiter_state.callback();
    }

    /* Release arbiter */
    arbiter_state.busy = false;
    arbiter_state.current_device = I2C_DMA_DEVICE_COUNT;
    arbiter_state.callback = NULL;
}

/**
 * @brief Get arbiter statistics
 */
void I2C_DMA_Arbiter_GetStats(I2C_DMA_Arbiter_Stats_t *stats) {
    if (stats != NULL) {
        memcpy(stats, &arbiter_state.stats, sizeof(I2C_DMA_Arbiter_Stats_t));
    }
}

/**
 * @brief Reset arbiter statistics
 */
void I2C_DMA_Arbiter_ResetStats(void) {
    memset(&arbiter_state.stats, 0, sizeof(I2C_DMA_Arbiter_Stats_t));
}
