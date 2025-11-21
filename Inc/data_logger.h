/**
  ******************************************************************************
  * @file    data_logger.h
  * @brief   Data logger stub for BLE module compatibility
  * @note    Minimal implementation - can be expanded for flash logging
  ******************************************************************************
  */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Start data recording
 * @return true if started successfully
 */
bool DataLogger_StartRecording(void);

/**
 * @brief Stop data recording
 * @return true if stopped successfully
 */
bool DataLogger_StopRecording(void);

/**
 * @brief Check if recording is active
 * @return true if recording
 */
bool DataLogger_IsRecording(void);

/**
 * @brief Erase all logged data
 * @return true if erased successfully
 */
bool DataLogger_EraseAll(void);

/**
 * @brief Get status string for BLE display
 * @param buffer Buffer to store status string
 * @param buffer_size Size of buffer
 * @return true if status retrieved
 */
bool DataLogger_GetStatusString(char* buffer, size_t buffer_size);

#endif /* DATA_LOGGER_H_ */
