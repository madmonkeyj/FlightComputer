/**
  ******************************************************************************
  * @file    ble_module.c
  * @brief   BLE module with interrupt-based reception for simple commands
  * @note    Simple interrupt RX for low-frequency command interface
  *          DMA is overkill for receiving ~10 bytes/second command strings
  ******************************************************************************
  */

#include "ble_module.h"
#include "debug_utils.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "data_logger.h"

/* Private variables - fully encapsulated within module */

/* UART receive byte - now private to BLE module */
static uint8_t rxByte = 0;
static bool uart_rx_active = false;

/* Command processing buffer */
static uint8_t rx_buffer[128];
static uint16_t rx_index = 0;

/* Status and statistics */
static uint32_t last_rx_time = 0;
static uint32_t last_tx_time = 0;
static uint32_t total_bytes_received = 0;
static uint32_t total_bytes_sent = 0;
static char uart_response_buffer[256];
static uint16_t uart_response_index = 0;
static uint32_t connection_count = 0;
static BLE_Status_t ble_status = BLE_STATUS_DISCONNECTED;
static bool ble_initialized = false;

/* Command processing variables */
static BLE_CommandCallback_t command_callback = NULL;
static bool data_transmission_enabled = true; /* Controls sensor data transmission */

/* Command timeout variables */
static uint32_t last_char_time = 0;
static uint32_t char_timeout_ms = 2000;  /* 2 second timeout for incomplete commands */

// Command debugging variables
static volatile bool command_received_flag = false;
static char last_command_received[64] = {0};
static volatile bool response_sent_flag = false;

/* Private function prototypes */
static void ClearResponseBuffer(void);
static bool SendBleCommand(const char* cmd, const char* expectedResponse, uint32_t timeout);
static void ResetBleModule(void);
static void BLE_ProcessReceivedByte(uint8_t byte);
static void BLE_StartReception(void);
static void BLE_ProcessCommand(const char* command);
static void BLE_ProcessIncompleteBuffer(void);
static bool IsProtocolOverhead(const uint8_t* data, uint16_t length);
static bool IsInCommandMode(void);

bool BLE_GetLastCommand(char* command_buffer, size_t buffer_size) {
    if (command_received_flag && command_buffer && buffer_size > 0) {
        strncpy(command_buffer, last_command_received, buffer_size - 1);
        command_buffer[buffer_size - 1] = '\0';
        command_received_flag = false; // Clear flag
        return true;
    }
    return false;
}

/**
 * @brief Check if received data is likely BLE protocol overhead
 */
static bool IsProtocolOverhead(const uint8_t* data, uint16_t length) {
    if (!data || length == 0) return true;

    // Very short data is likely protocol overhead
    if (length < 3) return true;

    // Check for binary/non-printable data (common in BLE protocol)
    uint16_t non_printable = 0;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        // Count non-printable characters (excluding common line endings)
        if (byte < 32 && byte != '\r' && byte != '\n' && byte != '\t') {
            non_printable++;
        }
    }

    // If more than 25% is non-printable, likely protocol data
    return (non_printable * 100 / length) > 25;
}

/**
 * @brief Process incomplete buffer due to timeout - INTERNAL FUNCTION with filtering
 */
static void BLE_ProcessIncompleteBuffer(void) {
    if (rx_index > 0) {
        // First check if this looks like protocol overhead
        if (IsProtocolOverhead(rx_buffer, rx_index)) {
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                    "BLE: Ignoring protocol overhead (%d bytes)\r\n", rx_index);
            DebugPrint(debug_msg);
            rx_index = 0;
            return;
        }

        rx_buffer[rx_index] = '\0';

        // Show exactly what we received for debugging
        DebugPrint("BLE: Timeout buffer: ");
        for (int i = 0; i < rx_index && i < 20; i++) {  // Limit to first 20 bytes
            char hex[10];
            snprintf(hex, sizeof(hex), "%02X ", rx_buffer[i]);
            DebugPrint(hex);
        }
        DebugPrint("\r\n");

        /* Remove trailing whitespace */
        while (rx_index > 0 && (rx_buffer[rx_index-1] == '\r' ||
               rx_buffer[rx_index-1] == '\n' || rx_buffer[rx_index-1] == ' ')) {
            rx_buffer[--rx_index] = '\0';
        }

        if (rx_index >= 3) { /* Only process reasonable length commands */
            char cmd_msg[150];
            snprintf(cmd_msg, sizeof(cmd_msg), "BLE: Processing timeout command: '%.50s'\r\n", rx_buffer);
            DebugPrint(cmd_msg);
            BLE_ProcessCommand((char*)rx_buffer);
        } else {
            DebugPrint("BLE: Ignoring short timeout data\r\n");
        }

        rx_index = 0; /* Reset for next message */
    }
}

/**
 * @brief No-configuration BLE setup - just reset and use defaults
 */
bool BLE_Configure_NoConfig(void) {
    DebugPrint("BLE: Using default configuration (no commands)...\r\n");

    /* Just hardware reset and let it boot in default Data mode */
    ResetBleModule();

    /* Clear UART errors to ensure clean interrupt RX start */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;

    DebugPrint("BLE: Module reset complete\r\n");
    DebugPrint("BLE: Should be in default Data mode (transparent)\r\n");
    DebugPrint("BLE: No configuration commands sent\r\n");

    return true;
}

/**
 * @brief Check if already in command mode
 */
static bool IsInCommandMode(void) {
    DebugPrint("BLE: Checking if already in command mode...\r\n");
    ClearResponseBuffer();

    // Send a simple command that returns the prompt
    // V command shows version, or if in data mode it just passes through
    if (SendBleCommand("V\r", "Ver", 1000)) {
        DebugPrint("BLE: Already in command mode (got version)\r\n");
        return true;
    }

    // Check if we got CMD> prompt in the response
    if (strstr(uart_response_buffer, "CMD>") != NULL) {
        DebugPrint("BLE: Already in command mode (got prompt)\r\n");
        return true;
    }

    return false;
}

/**
 * @brief Try multiple times to enter command mode
 */
static bool EnterCommandMode(void) {
    DebugPrint("BLE: Attempting to enter command mode...\r\n");

    // First check if already in command mode
    if (IsInCommandMode()) {
        return true;
    }

    for (int attempt = 1; attempt <= 5; attempt++) {
        char attempt_msg[50];
        snprintf(attempt_msg, sizeof(attempt_msg), "BLE: Command mode attempt %d/5\r\n", attempt);
        DebugPrint(attempt_msg);

        // Clear response buffer
        ClearResponseBuffer();

        // Try to enter command mode
        if (SendBleCommand("$$$", "CMD>", 3000)) {
            DebugPrint("BLE: Successfully entered command mode!\r\n");
            return true;
        }

        // If failed, try alternative approaches
        if (attempt == 2) {
            DebugPrint("BLE: Trying with shorter command...\r\n");
            if (SendBleCommand("$", "CMD>", 2000)) {
                DebugPrint("BLE: Entered command mode with single $\r\n");
                return true;
            }
        }

        if (attempt == 3) {
            DebugPrint("BLE: Checking if already in command mode...\r\n");
            if (SendBleCommand("V\r", "RN", 2000)) {
                DebugPrint("BLE: Already in command mode!\r\n");
                return true;
            }
        }

        // Wait between attempts
        HAL_Delay(1000);
    }

    DebugPrint("BLE: ERROR - Failed to enter command mode after 5 attempts\r\n");
    return false;
}

/**
 * @brief Initialize BLE module with DMA
 */
bool BLE_Init(void) {
    DebugPrint("BLE: Initializing BLE module (interrupt mode)...\r\n");

    /* Reset all private variables */
    rxByte = 0;
    uart_rx_active = false;
    rx_index = 0;
    last_rx_time = 0;
    last_tx_time = 0;
    total_bytes_received = 0;
    total_bytes_sent = 0;
    uart_response_index = 0;
    connection_count = 0;
    ble_status = BLE_STATUS_CONNECTING;
    command_callback = NULL;
    data_transmission_enabled = true;

    /* Initialize timeout variables */
    last_char_time = 0;
    char_timeout_ms = 2000;  // 2 second timeout

    /* Clear buffers */
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(uart_response_buffer, 0, sizeof(uart_response_buffer));

    /* Configure the BLE module */
    if (!BLE_Configure()) {
        DebugPrint("BLE: ERROR - Configuration failed\r\n");
        ble_status = BLE_STATUS_ERROR;
        ble_initialized = false;
        return false;
    }

    /* CRITICAL: After blocking UART config, reset UART to clean state for interrupt RX */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    huart1.RxState = HAL_UART_STATE_READY;  // Reset RX state machine

    /* Clear any leftover data from configuration */
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 1) == HAL_OK) {
        /* Drain */
    }

    /* Start UART reception */
    BLE_StartReception();

    ble_status = BLE_STATUS_DISCONNECTED; /* Ready but not connected */
    ble_initialized = true;
    last_rx_time = HAL_GetTick();

    DebugPrint("BLE: Module initialized successfully (interrupt RX active)\r\n");
    char timeout_msg[50];
    snprintf(timeout_msg, sizeof(timeout_msg), "BLE: Command timeout set to %lums\r\n", char_timeout_ms);
    DebugPrint(timeout_msg);
    return true;
}

/**
 * @brief Register command callback function
 */
void BLE_RegisterCommandCallback(BLE_CommandCallback_t callback) {
    command_callback = callback;
    DebugPrint("BLE: Command callback registered\r\n");
}

/**
 * @brief Enable/disable data transmission over BLE
 */
void BLE_SetDataTransmissionEnabled(bool enabled) {
    data_transmission_enabled = enabled;
    DebugPrint(enabled ? "BLE: Data transmission enabled\r\n" : "BLE: Data transmission disabled\r\n");
}

/**
 * @brief Check if data transmission is enabled
 */
bool BLE_IsDataTransmissionEnabled(void) {
    return data_transmission_enabled;
}

/**
 * @brief Send response message
 */
bool BLE_SendResponse(const char* response) {
    DebugPrint("=== BLE_SendResponse called ===\r\n");

    if (!ble_initialized || !response) {
        DebugPrint("ERROR: BLE not initialized or response null\r\n");
        return false;
    }

    char response_msg[100];
    snprintf(response_msg, sizeof(response_msg), "%s\r\n", response);

    DebugPrint("Sending response: ");
    DebugPrint(response_msg);

    bool result = BLE_SendString(response_msg);

    if (result) {
        DebugPrint("Response sent successfully\r\n");
    } else {
        DebugPrint("Response send FAILED\r\n");
    }

    return result;
}

/**
 * @brief Update BLE module - handles timeouts and connection status
 * @note Call regularly from main loop
 */
void BLE_Update(void) {
    if (!ble_initialized) {
        return;
    }

    /* Ensure UART reception is active */
    BLE_StartReception();

    /* Handle command timeout */
    if (rx_index > 0) {
        uint32_t time_since_last_char = HAL_GetTick() - last_char_time;

        if (time_since_last_char > char_timeout_ms) {
            DebugPrint("BLE: Command timeout - processing incomplete buffer\r\n");
            BLE_ProcessIncompleteBuffer();
        }
    }

    /* Update connection status */
    uint32_t time_since_activity = HAL_GetTick() - last_rx_time;

    if (time_since_activity < 5000) { /* Active within 5 seconds */
        if (ble_status == BLE_STATUS_DISCONNECTED) {
            ble_status = BLE_STATUS_CONNECTED;
            connection_count++;
            DebugPrint("BLE: Device connected\r\n");
        }
    } else if (time_since_activity > 15000) { /* No activity for 15 seconds */
        if (ble_status == BLE_STATUS_CONNECTED) {
            ble_status = BLE_STATUS_DISCONNECTED;
            DebugPrint("BLE: Device disconnected\r\n");
        }
    }
}

/**
 * @brief Send data via BLE
 */
bool BLE_SendData(const char* data, uint16_t length) {
    if (!ble_initialized || !data || length == 0) {
        return false;
    }

    // Check if this is sensor data and if data transmission is disabled
    if (!data_transmission_enabled && strncmp(data, "NAV_DATA,", 9) == 0) {
        return true; // Silently ignore sensor data when disabled
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, length, 1000);

    if (status == HAL_OK) {
        total_bytes_sent += length;
        last_tx_time = HAL_GetTick();
        return true;
    }

    return false;
}

/**
 * @brief Send string via BLE
 */
bool BLE_SendString(const char* str) {
    if (!str) {
        return false;
    }

    return BLE_SendData(str, strlen(str));
}

/**
 * @brief Process a complete command - UPDATED with data logger support
 */
static void BLE_ProcessCommand(const char* command) {
    if (!command || strlen(command) == 0) {
        return;
    }

    // Set flag and copy command for main loop debugging
    strncpy(last_command_received, command, sizeof(last_command_received)-1);
    command_received_flag = true;

    /* Convert to lowercase */
    char cmd_lower[64];
    strncpy(cmd_lower, command, sizeof(cmd_lower) - 1);
    cmd_lower[sizeof(cmd_lower) - 1] = '\0';

    for (int i = 0; cmd_lower[i]; i++) {
        if (cmd_lower[i] >= 'A' && cmd_lower[i] <= 'Z') {
            cmd_lower[i] = cmd_lower[i] + 32;
        }
    }

    /* Built-in command processing */
    if (strcmp(cmd_lower, "start") == 0) {
        // Start data logging instead of just changing transmission mode
        if (DataLogger_StartRecording()) {
            data_transmission_enabled = false;  // Stop BLE sensor data transmission
            BLE_SendResponse("Recording started - data logging to flash");
        } else {
            BLE_SendResponse("ERROR: Failed to start recording");
        }
        response_sent_flag = true;
    }
    else if (strcmp(cmd_lower, "stop") == 0) {
        // Stop data logging and resume BLE transmission
        if (DataLogger_StopRecording()) {
            data_transmission_enabled = true;   // Resume BLE sensor data transmission
            BLE_SendResponse("Recording stopped - BLE transmission resumed");
        } else {
            BLE_SendResponse("ERROR: Failed to stop recording");
        }
        response_sent_flag = true;
    }
    else if (strcmp(cmd_lower, "status") == 0) {
        // Get comprehensive status including data logger
        char status_msg[200];
        char logger_status[150];

        if (DataLogger_GetStatusString(logger_status, sizeof(logger_status))) {
            snprintf(status_msg, sizeof(status_msg),
                    "BLE: %s, Connected: %s | Logger: %s",
                    data_transmission_enabled ? "Data mode" : "Recording mode",
                    BLE_IsConnected() ? "Yes" : "No",
                    logger_status);
        } else {
            snprintf(status_msg, sizeof(status_msg),
                    "BLE: %s, Connected: %s | Logger: Not available",
                    data_transmission_enabled ? "Data mode" : "Recording mode",
                    BLE_IsConnected() ? "Yes" : "No");
        }

        BLE_SendResponse(status_msg);
        response_sent_flag = true;
    }
    else if (strcmp(cmd_lower, "erase") == 0) {
        // Manually erase flash data
        if (DataLogger_EraseAll()) {
            BLE_SendResponse("Flash memory erased");
        } else {
            BLE_SendResponse("ERROR: Failed to erase flash");
        }
        response_sent_flag = true;
    }
    else if (strcmp(cmd_lower, "help") == 0) {
        // List available commands
        BLE_SendResponse("Commands: start, stop, status, erase, help");
        response_sent_flag = true;
    }
    else {
        BLE_SendResponse("Unknown command - type 'help' for commands");
        response_sent_flag = true;
    }
}

/**
 * @brief Check if BLE is ready
 */
bool BLE_IsReady(void) {
    return ble_initialized && (ble_status != BLE_STATUS_ERROR);
}

/**
 * @brief Get BLE status
 */
BLE_Status_t BLE_GetStatus(void) {
    return ble_status;
}

/**
 * @brief Check if BLE is connected
 */
bool BLE_IsConnected(void) {
    return ble_status == BLE_STATUS_CONNECTED;
}

/**
 * @brief Get BLE statistics
 */
bool BLE_GetStatistics(BLE_Statistics_t* stats) {
    if (!stats || !ble_initialized) {
        return false;
    }

    stats->total_bytes_received = total_bytes_received;
    stats->total_bytes_sent = total_bytes_sent;
    stats->last_activity_time = last_rx_time;
    stats->connection_count = connection_count;
    stats->uart_active = uart_rx_active;
    stats->status = ble_status;

    return true;
}

/**
 * @brief Reset BLE module
 */
bool BLE_Reset(void) {
    DebugPrint("BLE: Resetting BLE module...\r\n");

    ResetBleModule();

    /* Reinitialize after reset */
    return BLE_Init();
}

/**
 * @brief Enhanced BLE configuration with better error handling
 */
bool BLE_Configure(void) {
    DebugPrint("BLE: Starting BLE configuration...\r\n");

    // Hardware reset with longer delays
    ResetBleModule();

    // Try to enter command mode
    if (!EnterCommandMode()) {
        DebugPrint("BLE: ERROR - Cannot enter command mode\r\n");
        return false;
    }

    bool success = true;
    int failed_commands = 0;

    // Skip factory reset - causes issues with re-entering command mode
    // Module will use existing configuration or defaults
    DebugPrint("BLE: Skipping factory reset (using existing config)\r\n");

    // Configure device name
    DebugPrint("BLE: Setting device name...\r\n");
    if (!SendBleCommand("SN,STM32-SENSOR-GPS\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Device name setting failed\r\n");
        failed_commands++;
    }

    // Configure authentication
    DebugPrint("BLE: Setting authentication...\r\n");
    if (!SendBleCommand("SA,0\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Authentication setting failed\r\n");
        failed_commands++;
    }

    // Configure services
    DebugPrint("BLE: Setting services...\r\n");
    if (!SendBleCommand("SS,C0\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Services setting failed\r\n");
        failed_commands++;
    }

    /* NOTE: SS,C0 already enables Microchip Data Service (transparent UART)
     * This service includes built-in TX and RX characteristics
     * No need for PC (Private Characteristic) commands - they cause "Err" responses
     * The module handles bidirectional transparent UART automatically
     */
    DebugPrint("BLE: Transparent UART enabled via SS,C0 (built-in service)\r\n");

    // Configure output mode
    DebugPrint("BLE: Setting output mode...\r\n");
    if (!SendBleCommand("SO,0\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Output mode setting failed\r\n");
        failed_commands++;
    }

    // Reboot to apply settings
    DebugPrint("BLE: Rebooting to apply settings...\r\n");
    if (!SendBleCommand("R,1\r", "Reboot", 2000)) {
        DebugPrint("BLE: WARNING - Reboot command failed\r\n");
        failed_commands++;
    } else {
        HAL_Delay(5000); // Wait for reboot
    }

    // Re-enter command mode for final configuration
    if (!EnterCommandMode()) {
        DebugPrint("BLE: ERROR - Cannot enter command mode after reboot\r\n");
        return false;
    }

    // Configure advertising
    DebugPrint("BLE: Setting advertising...\r\n");
    if (!SendBleCommand("SGA,0\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Advertising setting failed\r\n");
        failed_commands++;
    }

    // Enable advertising
    DebugPrint("BLE: Enabling advertising...\r\n");
    if (!SendBleCommand("A\r", "AOK", 2000)) {
        DebugPrint("BLE: WARNING - Enable advertising failed\r\n");
        failed_commands++;
    }

    // Exit command mode
    DebugPrint("BLE: Exiting command mode...\r\n");
    if (!SendBleCommand("---\r", "END", 2000)) {
        DebugPrint("BLE: WARNING - Exit command mode failed\r\n");
        failed_commands++;
    }

    char result_msg[100];
    snprintf(result_msg, sizeof(result_msg), "BLE: Configuration complete. %d commands failed\r\n", failed_commands);
    DebugPrint(result_msg);

    // Consider it successful if most commands worked
    success = (failed_commands < 5);

    if (success) {
        DebugPrint("BLE: Configuration successful!\r\n");
    } else {
        DebugPrint("BLE: Configuration failed - too many command failures\r\n");
    }

    return success;
}

/**
 * @brief Get time since last activity
 */
uint32_t BLE_GetTimeSinceLastActivity(void) {
    return HAL_GetTick() - last_rx_time;
}

/**
 * @brief Clear the UART response buffer
 */
static void ClearResponseBuffer(void) {
    memset(uart_response_buffer, 0, sizeof(uart_response_buffer));
    uart_response_index = 0;
}

/**
 * @brief Send BLE command and wait for response - uses blocking UART
 * @note Used during configuration before interrupt RX is active
 * @note TX is always blocking and doesn't conflict with interrupt RX
 */
static bool SendBleCommand(const char* cmd, const char* expectedResponse, uint32_t timeout) {
    char debugMsg[300];

    ClearResponseBuffer();

    // Show command with length
    snprintf(debugMsg, sizeof(debugMsg), "BLE: Sending '%s' (%d bytes)\r\n", cmd, strlen(cmd));
    DebugPrint(debugMsg);

    // Send command with error checking (TX can be blocking - doesn't conflict with RX)
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
    if (tx_status != HAL_OK) {
        snprintf(debugMsg, sizeof(debugMsg), "BLE: ERROR - UART transmit failed (status=%d)\r\n", tx_status);
        DebugPrint(debugMsg);
        return false;
    }
    total_bytes_sent += strlen(cmd);

    // Wait for response using blocking UART receive
    uint32_t startTime = HAL_GetTick();
    bool responseFound = false;
    int bytes_received = 0;

    /* Blocking Mode: Use HAL UART receive (safe during configuration) */
    while ((HAL_GetTick() - startTime) < timeout && !responseFound) {
        uint8_t tempByte;
        HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart1, &tempByte, 1, 1);

        if (rx_status == HAL_OK) {
            bytes_received++;
            if (uart_response_index < sizeof(uart_response_buffer) - 1) {
                uart_response_buffer[uart_response_index++] = tempByte;
                uart_response_buffer[uart_response_index] = '\0';
            }

            // Check for expected response
            if (expectedResponse == NULL) {
                responseFound = true;
            } else if (strstr(uart_response_buffer, expectedResponse) != NULL) {
                responseFound = true;
            } else if (strstr(uart_response_buffer, "ERR") != NULL) {
                snprintf(debugMsg, sizeof(debugMsg), "BLE: Got error response: %s\r\n", uart_response_buffer);
                DebugPrint(debugMsg);
                return false;
            }
        }
    }

    if (responseFound) {
        snprintf(debugMsg, sizeof(debugMsg), "BLE: SUCCESS - Got '%s' (%d bytes)\r\n", uart_response_buffer, bytes_received);
        DebugPrint(debugMsg);
        return true;
    } else {
        snprintf(debugMsg, sizeof(debugMsg), "BLE: TIMEOUT - Received %d bytes: '%s'\r\n", bytes_received, uart_response_buffer);
        DebugPrint(debugMsg);
        return false;
    }
}

/**
 * @brief Enhanced BLE module reset with longer delays
 */
static void ResetBleModule(void) {
    DebugPrint("BLE: Hardware resetting BLE module...\r\n");

    /* Ensure reset pin is properly configured as output */
    HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);  /* Increased from 200ms to 500ms */

    HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_SET);
    HAL_Delay(5000); /* Increased from 3000ms to 5000ms - give module more time */

    DebugPrint("BLE: Module reset complete\r\n");

    /* Clear any pending UART data */
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 10) == HAL_OK) {
        /* Drain any leftover data */
    }
    DebugPrint("BLE: UART buffer cleared\r\n");
}

/**
 * @brief Process received byte - NOW UPDATES TIMEOUT TIMER with better filtering
 */
static void BLE_ProcessReceivedByte(uint8_t byte) {
    // Just toggle LED to show activity - this is interrupt-safe
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = byte;
    }

    /* UPDATE: Track timing for both general activity and character timeout */
    uint32_t current_time = HAL_GetTick();
    last_rx_time = current_time;
    last_char_time = current_time;
    total_bytes_received++;

    /* DEBUG: Show received character */
    char byte_msg[50];
    if (byte >= 32 && byte <= 126) {  // Printable
        snprintf(byte_msg, sizeof(byte_msg), "BLE RX: '%c' (0x%02X) idx=%u\r\n", byte, byte, rx_index-1);
    } else {
        snprintf(byte_msg, sizeof(byte_msg), "BLE RX: 0x%02X idx=%u\r\n", byte, rx_index-1);
    }
    DebugPrint(byte_msg);

    /* Process complete messages immediately if properly terminated */
    if (byte == '\n') {  // Only process on newline
        rx_buffer[rx_index] = '\0';

        /* Remove trailing newlines/carriage returns */
        while (rx_index > 0 && (rx_buffer[rx_index-1] == '\r' || rx_buffer[rx_index-1] == '\n')) {
            rx_buffer[--rx_index] = '\0';
        }

        if (rx_index > 0 && !IsProtocolOverhead(rx_buffer, rx_index)) {
            char complete_msg[150];
            snprintf(complete_msg, sizeof(complete_msg), "BLE: Complete command: '%.50s'\r\n", rx_buffer);
            DebugPrint(complete_msg);
            BLE_ProcessCommand((char*)rx_buffer);
        } else if (rx_index > 0) {
            DebugPrint("BLE: Ignoring protocol data on newline\r\n");
        }

        rx_index = 0; /* Reset for next message */
    }
    // Don't process on '\r' alone - wait for '\n' or timeout
}

/**
 * @brief Start UART reception with enhanced error handling
 */
static void BLE_StartReception(void) {
    if (!uart_rx_active) {
        /* Clear any existing errors first */
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        /* Try to start reception */
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &rxByte, 1);

        if (status == HAL_OK) {
            uart_rx_active = true;
        } else {
            /* If failed, wait and try once more */
            HAL_Delay(1);  /* Brief delay */

            if (HAL_UART_Receive_IT(&huart1, &rxByte, 1) == HAL_OK) {
                uart_rx_active = true;
            } else {
                /* Still failing - log for debugging */
                static uint32_t last_error_log = 0;
                if (HAL_GetTick() - last_error_log > 5000) {
                    DebugPrint("BLE: UART reception start failed\r\n");
                    last_error_log = HAL_GetTick();
                }
            }
        }
    }
}

/**
 * @brief UART RX Complete Callback - handles BLE interrupt byte reception
 * @note Called when 1 byte is received via HAL_UART_Receive_IT
 * @note GPS (USART3) uses DMA, not this callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* BLE UART handling */
        /* Check for UART errors and handle them inline */
        if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
            /* Clear error flags */
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);

            /* Reset error code */
            huart->ErrorCode = HAL_UART_ERROR_NONE;
        } else {
            /* Process the byte only if no errors */
            BLE_ProcessReceivedByte(rxByte);
        }

        /* Restart reception */
        uart_rx_active = false;
        if (HAL_UART_Receive_IT(&huart1, &rxByte, 1) == HAL_OK) {
            uart_rx_active = true;
        } else {
            /* If restart fails, try again in main loop */
            uart_rx_active = false;
        }
    }
}

/* Legacy compatibility functions - simplified since callback is now internal */

/**
 * @brief Legacy byte handler - now just calls internal function
 */
void HandleReceivedByte(uint8_t byte) {
    BLE_ProcessReceivedByte(byte);
    uart_rx_active = false;
    BLE_StartReception();
}

/**
 * @brief Legacy UART start - calls internal function
 */
void StartUartReception(void) {
    BLE_StartReception();
}

/**
 * @brief Legacy configuration - calls new BLE_Configure()
 */
bool ConfigureModule(void) {
    return BLE_Configure();
}
