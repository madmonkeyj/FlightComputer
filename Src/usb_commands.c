/**
  ******************************************************************************
  * @file    usb_commands.c
  * @brief   USB Command Interface Implementation - CLEANED UP
  ******************************************************************************
  */

#include "usb_commands.h"
#include "data_logger.h"
#include "usbd_cdc_if.h"
#include "quadspi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private constants */
#define USB_CMD_BUFFER_SIZE     64
#define USB_RESPONSE_BUFFER_SIZE 256
#define USB_RX_BUFFER_SIZE      128

/* Private variables */
static bool usb_commands_initialized = false;
static USBCommandCallback_t custom_callback = NULL;

/* Command processing */
static char usb_command_buffer[USB_CMD_BUFFER_SIZE];
static volatile bool usb_command_ready = false;

/* USB receive buffer */
static char usb_rx_buffer[USB_RX_BUFFER_SIZE];
static uint16_t usb_rx_index = 0;

/* Response handling */
static char response_buffer[USB_RESPONSE_BUFFER_SIZE];

/* Private function prototypes */
static USBCommandStatus_t ProcessBuiltinCommand(const char* command, char* response, size_t response_size);
static void ConvertToLowercase(char* str);

/**
 * @brief Initialize USB command interface
 */
bool USBCommands_Init(void) {
    // Clear all buffers
    memset(usb_command_buffer, 0, sizeof(usb_command_buffer));
    memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
    memset(response_buffer, 0, sizeof(response_buffer));

    // Reset state
    usb_command_ready = false;
    usb_rx_index = 0;
    custom_callback = NULL;

    usb_commands_initialized = true;
    return true;
}

/**
 * @brief Update USB command interface
 */
void USBCommands_Update(void) {
    if (!usb_commands_initialized) {
        return;
    }

    // Process pending command
    if (usb_command_ready) {
        // Clear response buffer
        memset(response_buffer, 0, sizeof(response_buffer));

        // Try built-in commands first
        USBCommandStatus_t status = ProcessBuiltinCommand(usb_command_buffer, response_buffer, sizeof(response_buffer));

        // If not a built-in command, try custom callback
        if (status == USB_CMD_INVALID && custom_callback) {
            status = custom_callback(usb_command_buffer, response_buffer, sizeof(response_buffer));
        }

        // Send response
        if (strlen(response_buffer) > 0) {
            USBCommands_SendResponse(response_buffer);
        } else {
            // Default response for unhandled commands
            USBCommands_SendResponse("Unknown command");
        }

        // Clear command
        usb_command_ready = false;
        memset(usb_command_buffer, 0, sizeof(usb_command_buffer));
    }
}

/**
 * @brief Register custom command callback
 */
void USBCommands_RegisterCallback(USBCommandCallback_t callback) {
    custom_callback = callback;
}

/**
 * @brief Process received USB data
 */
void USBCommands_ProcessReceivedData(uint8_t* data, uint32_t length) {
    if (!usb_commands_initialized || !data) {
        return;
    }

    // Process each received byte
    for (uint32_t i = 0; i < length; i++) {
        char ch = (char)data[i];

        // Check for command termination
        if (ch == '\r' || ch == '\n') {
            if (usb_rx_index > 0) {
                // Null terminate
                usb_rx_buffer[usb_rx_index] = '\0';

                // Copy to command buffer if not already processing a command
                if (!usb_command_ready) {
                    strncpy(usb_command_buffer, usb_rx_buffer, sizeof(usb_command_buffer) - 1);
                    usb_command_buffer[sizeof(usb_command_buffer) - 1] = '\0';
                    usb_command_ready = true;
                }

                // Reset receive buffer
                usb_rx_index = 0;
                memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
            }
        }
        // Add printable characters to buffer
        else if (ch >= 32 && ch <= 126 && usb_rx_index < sizeof(usb_rx_buffer) - 1) {
            usb_rx_buffer[usb_rx_index++] = ch;
        }
        // Ignore other characters (control chars, etc.)
    }
}

/**
 * @brief Send response via USB
 */
bool USBCommands_SendResponse(const char* response) {
    if (!usb_commands_initialized || !response) {
        return false;
    }

    // Format response with prefix and newline
    char formatted_response[USB_RESPONSE_BUFFER_SIZE];
    snprintf(formatted_response, sizeof(formatted_response), "USB_RESP: %s\r\n", response);

    // Send via USB CDC
    uint8_t result = CDC_Transmit_FS((uint8_t*)formatted_response, strlen(formatted_response));

    return (result == USBD_OK);
}

/**
 * @brief Check if USB command interface is ready
 */
bool USBCommands_IsReady(void) {
    return usb_commands_initialized;
}

/**
 * @brief Process built-in commands - ESSENTIAL COMMANDS ONLY
 */
static USBCommandStatus_t ProcessBuiltinCommand(const char* command, char* response, size_t response_size) {
    if (!command || !response) {
        return USB_CMD_ERROR;
    }

    // Convert to lowercase for comparison
    char cmd_lower[USB_CMD_BUFFER_SIZE];
    strncpy(cmd_lower, command, sizeof(cmd_lower) - 1);
    cmd_lower[sizeof(cmd_lower) - 1] = '\0';
    ConvertToLowercase(cmd_lower);

    // === HELP COMMAND ===
    if (strcmp(cmd_lower, "help") == 0) {
        snprintf(response, response_size,
                "Commands: help, status, info, start, stop, erase, download");
        return USB_CMD_OK;
    }

    // === STATUS COMMAND ===
    if (strcmp(cmd_lower, "status") == 0) {
        char status_buffer[256];
        if (DataLogger_GetStatusString(status_buffer, sizeof(status_buffer))) {
            strncpy(response, status_buffer, response_size - 1);
            response[response_size - 1] = '\0';
        } else {
            snprintf(response, response_size, "ERROR: Cannot get status");
        }
        return USB_CMD_OK;
    }

    // === INFO COMMAND ===
    if (strcmp(cmd_lower, "info") == 0) {
        LoggerStats_t stats;
        if (!DataLogger_GetStats(&stats)) {
            snprintf(response, response_size, "ERROR: Cannot get stats");
            return USB_CMD_OK;
        }

        float flash_usage_percent = (float)(stats.flash_bytes_used * 100) / DATA_AREA_SIZE;

        snprintf(response, response_size,
                "Records: %lu | Size: %lu bytes | Flash: %.1f%% | Rate: %luHz",
                stats.records_written,
                stats.record_size,
                flash_usage_percent,
                stats.recording_rate_hz);
        return USB_CMD_OK;
    }

    // === START/STOP COMMANDS ===
    if (strcmp(cmd_lower, "start") == 0) {
        if (DataLogger_StartRecording()) {
            snprintf(response, response_size, "Recording started");
        } else {
            snprintf(response, response_size, "Failed to start recording");
        }
        return USB_CMD_OK;
    }

    if (strcmp(cmd_lower, "stop") == 0) {
        if (DataLogger_StopRecording()) {
            snprintf(response, response_size, "Recording stopped");
        } else {
            snprintf(response, response_size, "Failed to stop recording");
        }
        return USB_CMD_OK;
    }

    // === ERASE COMMAND ===
    if (strcmp(cmd_lower, "erase") == 0) {
        if (DataLogger_IsRecording() || DataLogger_IsDownloading()) {
            snprintf(response, response_size, "Cannot erase while recording/downloading");
        } else if (DataLogger_EraseAll()) {
            snprintf(response, response_size, "Flash erased successfully");
        } else {
            snprintf(response, response_size, "Failed to erase flash");
        }
        return USB_CMD_OK;
    }

    // === DOWNLOAD COMMAND ===
    if (strcmp(cmd_lower, "download") == 0) {
        LoggerStats_t stats;
        if (!DataLogger_GetStats(&stats)) {
            snprintf(response, response_size, "ERROR: Cannot get logger stats");
            return USB_CMD_OK;
        }

        if (stats.records_written == 0) {
            snprintf(response, response_size, "No data to download");
            return USB_CMD_OK;
        }

        uint32_t record_size = stats.record_size;
        uint32_t total_records = stats.records_written;
        uint32_t total_bytes = total_records * record_size;

        // Send download header
        snprintf(response, response_size,
            "DOWNLOAD:RECORD_SIZE=%lu:RECORDS=%lu:BYTES=%lu:READY",
            record_size, total_records, total_bytes);
        USBCommands_SendResponse(response);

        // Wait for client to be ready
        HAL_Delay(100);

        // Calculate optimal chunk size
        uint32_t max_chunk_bytes = 8192;  // Conservative USB buffer size
        uint32_t records_per_chunk = max_chunk_bytes / record_size;
        if (records_per_chunk == 0) records_per_chunk = 1;  // At least 1 record

        uint32_t actual_chunk_size = records_per_chunk * record_size;

        // Allocate chunk buffer
        uint8_t* chunk_buffer = malloc(actual_chunk_size);
        if (!chunk_buffer) {
            snprintf(response, response_size, "ERROR: Memory allocation failed");
            USBCommands_SendResponse(response);
            return USB_CMD_OK;
        }

        uint32_t address = 0;
        uint32_t records_sent = 0;
        bool download_success = true;

        while (records_sent < total_records && download_success) {
            // Calculate this chunk's size
            uint32_t records_remaining = total_records - records_sent;
            uint32_t records_this_chunk = (records_remaining > records_per_chunk) ?
                                         records_per_chunk : records_remaining;
            uint32_t bytes_this_chunk = records_this_chunk * record_size;

            // Read chunk from flash
            if (CSP_QSPI_Read(chunk_buffer, address, bytes_this_chunk) != HAL_OK) {
                snprintf(response, response_size, "ERROR: Flash read failed at record %lu", records_sent);
                USBCommands_SendResponse(response);
                download_success = false;
                break;
            }

            // Send chunk header
            snprintf(response, response_size, "CHUNK:START=%lu:COUNT=%lu:BYTES=%lu:",
                    records_sent, records_this_chunk, bytes_this_chunk);
            USBCommands_SendResponse(response);
            HAL_Delay(10);

            // Send chunk data in pieces
            uint32_t bytes_sent_in_chunk = 0;
            while (bytes_sent_in_chunk < bytes_this_chunk && download_success) {
                uint32_t piece_size = 512;  // Send 512 bytes at a time
                if (bytes_sent_in_chunk + piece_size > bytes_this_chunk) {
                    piece_size = bytes_this_chunk - bytes_sent_in_chunk;
                }

                if (CDC_Transmit_FS(&chunk_buffer[bytes_sent_in_chunk], piece_size) != USBD_OK) {
                    snprintf(response, response_size, "ERROR: USB transmission failed");
                    USBCommands_SendResponse(response);
                    download_success = false;
                    break;
                }

                bytes_sent_in_chunk += piece_size;
                HAL_Delay(2);  // Small delay between pieces
            }

            if (download_success) {
                // Send chunk end marker
                HAL_Delay(10);
                snprintf(response, response_size, "CHUNK_END");
                USBCommands_SendResponse(response);

                // Update progress
                records_sent += records_this_chunk;
                address += bytes_this_chunk;

                // Progress update every 20 chunks
                if ((records_sent / records_per_chunk) % 20 == 0) {
                    float progress = (float)(records_sent * 100) / total_records;
                    snprintf(response, response_size, "PROGRESS:%.1f%%", progress);
                    USBCommands_SendResponse(response);
                }

                HAL_Delay(50);  // Delay between chunks
            }
        }

        // Send completion message
        if (download_success) {
            snprintf(response, response_size, "DOWNLOAD_COMPLETE:RECORDS=%lu", records_sent);
        } else {
            snprintf(response, response_size, "DOWNLOAD_FAILED:PARTIAL=%lu", records_sent);
        }
        USBCommands_SendResponse(response);

        free(chunk_buffer);
        response[0] = '\0';  // Clear response to avoid double-send
        return USB_CMD_OK;
    }

    // Command not recognized
    return USB_CMD_INVALID;
}

/**
 * @brief Convert string to lowercase
 */
static void ConvertToLowercase(char* str) {
    if (!str) return;

    for (int i = 0; str[i]; i++) {
        if (str[i] >= 'A' && str[i] <= 'Z') {
            str[i] = str[i] + 32;
        }
    }
}
