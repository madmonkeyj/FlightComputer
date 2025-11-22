/**
  ******************************************************************************
  * @file    ble_module.h
  * @brief   BLE module with DMA circular buffer - Self-Managing Header
  * @note    Uses DMA with idle line detection (same pattern as GPS module)
  * @note    Converted from interrupt-based to DMA for consistency and efficiency
  *
  * @hardware RN4871 Bluetooth Low Energy Module
  * @interface USART1 @ 115200 baud with DMA
  * @gpio Required GPIO pins (defined in main.h):
  *       - RST_BT (PC6) → RN4871 RST_N: Hardware reset (active LOW)
  *       - CONFIG (PB15) → RN4871 P2_0: Set HIGH for active operation
  *       - LPM (PA8) → RN4871 P1_6: Set HIGH for active (non-sleep) mode
  *
  * @note All three pins are BLE module specific and controlled during init
  ******************************************************************************
  */

#ifndef BLE_MODULE_H_
#define BLE_MODULE_H_

#include "main.h"
#include <stdbool.h>

/**
 * @brief BLE connection status
 */
typedef enum {
    BLE_STATUS_DISCONNECTED = 0,
    BLE_STATUS_CONNECTING,
    BLE_STATUS_CONNECTED,
    BLE_STATUS_ERROR
} BLE_Status_t;

/**
 * @brief BLE statistics structure
 */
typedef struct {
    uint32_t total_bytes_received;
    uint32_t total_bytes_sent;
    uint32_t last_activity_time;
    uint32_t connection_count;
    bool uart_active;
    BLE_Status_t status;
} BLE_Statistics_t;

/**
 * @brief Command callback function type
 * @param command The received command string (null-terminated)
 */
typedef void (*BLE_CommandCallback_t)(const char* command);

/**
 * @brief BLE Module API - Self-Managing with Integrated Timeout
 */

/**
 * @brief Initialize BLE module with DMA circular buffer
 * @return true if initialization successful
 * @note Sets up DMA with idle line detection (same pattern as GPS)
 * @note Sets up 2s command timeout automatically
 */
bool BLE_Init(void);

/**
 * @brief Update BLE module - call regularly from main loop
 * @note Processes data from DMA circular buffer
 * @note Handles command processing and timeout
 * @note Uses idle line detection for message boundaries
 * @note This is the ONLY function you need to call - everything else is automatic
 */
void BLE_Update(void);

/**
 * @brief Register command callback function
 * @param callback Function to call when commands are received
 * @note Commands are processed internally, callback is for additional handling
 */
void BLE_RegisterCommandCallback(BLE_CommandCallback_t callback);

/**
 * @brief Enable/disable data transmission over BLE
 * @param enabled true to allow sensor data transmission, false to block it
 * @note When disabled, sensor data is blocked but command responses still work
 */
void BLE_SetDataTransmissionEnabled(bool enabled);

/**
 * @brief Check if data transmission is enabled
 * @return true if sensor data transmission is enabled
 */
bool BLE_IsDataTransmissionEnabled(void);

/**
 * @brief Send response message (always allowed)
 * @param response Response string to send
 * @return true if response sent successfully
 */
bool BLE_SendResponse(const char* response);

/**
 * @brief Send data via BLE (respects data transmission setting)
 * @param data Pointer to data to send
 * @param length Length of data in bytes
 * @return true if data sent successfully
 * @note Sensor data is filtered based on data transmission setting
 */
bool BLE_SendData(const char* data, uint16_t length);

/**
 * @brief Send string via BLE (respects data transmission setting)
 * @param str Null-terminated string to send
 * @return true if string sent successfully
 */
bool BLE_SendString(const char* str);

/**
 * @brief Check if BLE is ready for communication
 * @return true if BLE is ready
 */
bool BLE_IsReady(void);

/**
 * @brief Get BLE connection status
 * @return Current BLE status
 */
BLE_Status_t BLE_GetStatus(void);

/**
 * @brief Check if BLE is connected
 * @return true if connected to a device
 */
bool BLE_IsConnected(void);

/**
 * @brief Get BLE statistics
 * @param stats Pointer to statistics structure to fill
 * @return true if statistics retrieved successfully
 */
bool BLE_GetStatistics(BLE_Statistics_t* stats);

/**
 * @brief Reset BLE module
 * @return true if reset successful
 */
bool BLE_Reset(void);

/**
 * @brief Configure BLE module with default settings
 * @return true if configuration successful
 */
bool BLE_Configure(void);

/**
 * @brief Get time since last BLE activity
 * @return Time in milliseconds since last activity
 */
uint32_t BLE_GetTimeSinceLastActivity(void);

/**
 * @brief Test functions for debugging BLE data mode
 */
bool BLE_Configure_NoConfig(void);

/**
 * @brief Get last processed command (for debugging)
 * @param command_buffer Buffer to store command
 * @param buffer_size Size of buffer
 * @return true if command was available
 * @note Clears the command flag after reading
 */
bool BLE_GetLastCommand(char* command_buffer, size_t buffer_size);

/* Internal callback for DMA idle line detection */
/**
 * @brief BLE UART RX Event handler - called from unified HAL_UARTEx_RxEventCallback
 * @note Internal function called by GPS module's unified UART callback
 * @note Do not call this directly - it's invoked automatically on idle line detection
 */
void BLE_UART_RxEventCallback(void);

/* Legacy compatibility functions - DEPRECATED with DMA implementation */
/* These are no-ops in DMA mode - kept for backward compatibility only */
void HandleReceivedByte(uint8_t byte);  /* @deprecated No-op in DMA mode */
void StartUartReception(void);          /* @deprecated No-op in DMA mode (auto-starts) */
bool ConfigureModule(void);             /* @deprecated Use BLE_Configure() instead */

#endif /* BLE_MODULE_H_ */
