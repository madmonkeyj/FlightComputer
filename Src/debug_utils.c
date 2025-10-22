/**
  ******************************************************************************
  * @file    debug_utils.c
  * @brief   Debug and utility functions implementation with printf support
  ******************************************************************************
  */

#include "debug_utils.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/**
 * @brief Enhanced debug print function with printf-style formatting
 */
void DebugPrint(const char *format, ...) {
    char buffer[512];  // Adjust size as needed
    va_list args;

    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Ensure null termination and prevent buffer overflow
    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
        buffer[len] = '\0';
    }

    CDC_Transmit_FS((uint8_t*)buffer, len);
    HAL_Delay(10);
}

/**
 * @brief Simple debug print function (original version)
 */
void DebugPrintSimple(const char *str) {
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
    HAL_Delay(10);
}
