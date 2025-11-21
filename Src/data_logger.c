/**
  ******************************************************************************
  * @file    data_logger.c
  * @brief   Data logger stub implementation
  * @note    Minimal stub for BLE module compatibility
  *          Can be expanded to use QSPI flash for actual logging
  ******************************************************************************
  */

#include "data_logger.h"
#include <string.h>
#include <stdio.h>

/* Private variables */
static bool recording_active = false;

/**
 * @brief Start data recording
 */
bool DataLogger_StartRecording(void) {
    recording_active = true;
    return true;
}

/**
 * @brief Stop data recording
 */
bool DataLogger_StopRecording(void) {
    recording_active = false;
    return true;
}

/**
 * @brief Check if recording
 */
bool DataLogger_IsRecording(void) {
    return recording_active;
}

/**
 * @brief Erase all data
 */
bool DataLogger_EraseAll(void) {
    recording_active = false;
    return true;
}

/**
 * @brief Get status string
 */
bool DataLogger_GetStatusString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return false;
    }

    snprintf(buffer, buffer_size, "%s",
             recording_active ? "Recording active" : "Idle");

    return true;
}
