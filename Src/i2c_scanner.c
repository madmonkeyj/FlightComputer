/**
  ******************************************************************************
  * @file    i2c_scanner.c
  * @brief   I2C bus scanner utility implementation
  ******************************************************************************
  */

#include "i2c_scanner.h"
#include "debug_utils.h"

/* I2C Address Range:
 * 0x00-0x07: Reserved addresses
 * 0x08-0x77: Valid 7-bit addresses
 * 0x78-0x7F: Reserved addresses
 */
#define I2C_SCAN_START_ADDR     0x08
#define I2C_SCAN_END_ADDR       0x77

/* Timeout for I2C device detection (in ms) */
#define I2C_SCAN_TIMEOUT        100

/**
 * @brief Scan I2C bus and print results via debug output
 * @note Uses I2C1 peripheral (hi2c1)
 */
void I2C_Scan(void) {
    HAL_StatusTypeDef result;
    uint8_t device_count = 0;
    uint8_t address;

    DebugPrint("\r\n");
    DebugPrint("========================================\r\n");
    DebugPrint("       I2C Bus Scanner (I2C1)          \r\n");
    DebugPrint("========================================\r\n");
    DebugPrint("Scanning I2C addresses 0x%02X to 0x%02X...\r\n\r\n",
               I2C_SCAN_START_ADDR, I2C_SCAN_END_ADDR);

    // Scan through all valid I2C addresses
    for (address = I2C_SCAN_START_ADDR; address <= I2C_SCAN_END_ADDR; address++) {
        // HAL_I2C_IsDeviceReady checks if device responds at given address
        // Address must be left-shifted by 1 for HAL function
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 3, I2C_SCAN_TIMEOUT);

        if (result == HAL_OK) {
            // Device found!
            DebugPrint("  [FOUND] Device at address 0x%02X\r\n", address);
            device_count++;
        }
    }

    DebugPrint("\r\n");
    DebugPrint("----------------------------------------\r\n");
    DebugPrint("Scan complete: %d device(s) found\r\n", device_count);
    DebugPrint("========================================\r\n\r\n");
}

/**
 * @brief Scan I2C bus without printing (quiet mode)
 * @param found_addresses: Array to store found addresses
 * @param max_addresses: Maximum number of addresses to store
 * @retval Number of devices found
 */
uint8_t I2C_ScanQuiet(uint8_t *found_addresses, uint8_t max_addresses) {
    HAL_StatusTypeDef result;
    uint8_t device_count = 0;
    uint8_t address;

    if (found_addresses == NULL || max_addresses == 0) {
        return 0;
    }

    // Scan through all valid I2C addresses
    for (address = I2C_SCAN_START_ADDR; address <= I2C_SCAN_END_ADDR; address++) {
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 3, I2C_SCAN_TIMEOUT);

        if (result == HAL_OK) {
            // Device found - store address if space available
            if (device_count < max_addresses) {
                found_addresses[device_count] = address;
            }
            device_count++;
        }
    }

    return device_count;
}
