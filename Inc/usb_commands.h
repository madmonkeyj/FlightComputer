/**
  ******************************************************************************
  * @file    usb_commands.h
  * @brief   USB Command Interface for Flight Controller
  ******************************************************************************
  */

#ifndef USB_COMMANDS_H_
#define USB_COMMANDS_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief USB command status
 */
typedef enum {
    USB_CMD_OK = 0,
    USB_CMD_ERROR,
    USB_CMD_INVALID,
    USB_CMD_NOT_READY
} USBCommandStatus_t;

/**
 * @brief USB command callback function type
 * @param command The received command string
 * @param response_buffer Buffer to write response
 * @param buffer_size Size of response buffer
 * @return Command execution status
 */
typedef USBCommandStatus_t (*USBCommandCallback_t)(const char* command, char* response_buffer, size_t buffer_size);

/**
 * @brief USB Command Interface API
 */

/**
 * @brief Initialize USB command interface
 * @return true if initialization successful
 */
bool USBCommands_Init(void);

/**
 * @brief Update USB command interface (call from main loop)
 * @note Processes pending commands and sends responses
 */
void USBCommands_Update(void);

/**
 * @brief Register custom command callback
 * @param callback Function to handle custom commands
 * @note Built-in commands (start, stop, status, help) are always available
 */
void USBCommands_RegisterCallback(USBCommandCallback_t callback);

/**
 * @brief Process received USB data (call from USB CDC receive callback)
 * @param data Pointer to received data
 * @param length Number of bytes received
 */
void USBCommands_ProcessReceivedData(uint8_t* data, uint32_t length);

/**
 * @brief Send response via USB
 * @param response Response string to send
 * @return true if response sent successfully
 */
bool USBCommands_SendResponse(const char* response);

/**
 * @brief Check if USB command interface is ready
 * @return true if ready to process commands
 */
bool USBCommands_IsReady(void);

/**
 * @brief Get last processed command (for debugging)
 * @param command_buffer Buffer to store command
 * @param buffer_size Size of buffer
 * @return true if command was available
 */
bool USBCommands_GetLastCommand(char* command_buffer, size_t buffer_size);

/**
 * @brief Enable/disable debug output
 * @param enabled true to enable debug output
 */
void USBCommands_SetDebugEnabled(bool enabled);

#endif /* USB_COMMANDS_H_ */
