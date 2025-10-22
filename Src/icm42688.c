/**
  ******************************************************************************
  * @file    icm42688.c
  * @brief   ICM-42688-P 6-axis IMU driver implementation (Datasheet compliant)
  ******************************************************************************
  */

#include "icm42688.h"
#include "debug_utils.h"
#include "gpio.h"

/* SPI timeout */
#define ICM42688_SPI_TIMEOUT    1000

/* Private function prototypes */
static void ICM42688_CS_Low(void);
static void ICM42688_CS_High(void);
static void ICM42688_Delay_us(uint32_t us);
static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

/**
 * @brief Microsecond delay helper
 */
static void ICM42688_Delay_us(uint32_t us) {
    // Approximate microsecond delay
    // Adjust multiplier based on your clock speed
    for(volatile uint32_t i = 0; i < us * 10; i++);
}

/**
 * @brief Set CS pin low (select device)
 * Per datasheet Table 6: CS setup time minimum 39ns
 */
static void ICM42688_CS_Low(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    // Wait for CS setup time (39ns min, we provide more for safety)
    ICM42688_Delay_us(1);
}

/**
 * @brief Set CS pin high (deselect device)
 * Per datasheet Table 6: CS hold time minimum 18ns
 */
static void ICM42688_CS_High(void) {
    // Small delay before raising CS (for hold time)
    ICM42688_Delay_us(1);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    // Wait before next transaction
    ICM42688_Delay_us(2);
}

/**
 * @brief Write to a single register
 * @param reg: Register address
 * @param value: Value to write
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];

    // Per datasheet Section 9.6: Write bit = 0 (bit 7 clear)
    tx_data[0] = reg & 0x7F;
    tx_data[1] = value;

    ICM42688_CS_Low();
    status = HAL_SPI_Transmit(&hspi3, tx_data, 2, ICM42688_SPI_TIMEOUT);
    ICM42688_CS_High();

    if (status != HAL_OK) {
        DebugPrint("  Write Error: Reg=0x%02X, Status=%d\r\n", reg, status);
    }

    return status;
}

/**
 * @brief Read from a single register
 * @param reg: Register address
 * @param value: Pointer to store read value
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef status;
    uint8_t tx_data;

    // Per datasheet Section 9.6: Read bit = 1 (bit 7 set)
    tx_data = reg | 0x80;

    ICM42688_CS_Low();

    // Send register address
    status = HAL_SPI_Transmit(&hspi3, &tx_data, 1, ICM42688_SPI_TIMEOUT);
    if (status != HAL_OK) {
        ICM42688_CS_High();
        DebugPrint("  Read TX Error: Reg=0x%02X, Status=%d\r\n", reg, status);
        return status;
    }

    // Receive data byte
    status = HAL_SPI_Receive(&hspi3, value, 1, ICM42688_SPI_TIMEOUT);
    if (status != HAL_OK) {
        DebugPrint("  Read RX Error: Reg=0x%02X, Status=%d\r\n", reg, status);
    }

    ICM42688_CS_High();

    return status;
}

/**
 * @brief Read multiple registers
 * @param reg: Starting register address
 * @param buffer: Pointer to buffer for read data
 * @param len: Number of bytes to read
 * @retval HAL status
 */
static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    uint8_t tx_data;

    // Per datasheet Section 9.6: Read bit = 1 (bit 7 set)
    tx_data = reg | 0x80;

    ICM42688_CS_Low();

    status = HAL_SPI_Transmit(&hspi3, &tx_data, 1, ICM42688_SPI_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi3, buffer, len, ICM42688_SPI_TIMEOUT);
    }

    ICM42688_CS_High();

    return status;
}

/**
 * @brief Initialize ICM-42688-P sensor
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t who_am_i;
    uint8_t reg_val;

    DebugPrint("\r\n=== ICM-42688-P Initialization ===\r\n");

    // Ensure CS is high initially
    ICM42688_CS_High();

    // CRITICAL: Per datasheet Table 4, page 14
    // "Start-up time for register read/write: From power-up 1 ms"
    DebugPrint("Waiting for sensor power-up...\r\n");
    HAL_Delay(10);  // Wait 10ms to be safe

    // Try to read WHO_AM_I before reset to verify communication
    DebugPrint("Testing SPI communication...\r\n");
    status = ICM42688_ReadWhoAmI(&who_am_i);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Initial SPI communication failed!\r\n");
        DebugPrint("Check:\r\n");
        DebugPrint("  1. SPI is configured for 8-bit mode\r\n");
        DebugPrint("  2. Connections: CS=PC13, SCK=PB3, MISO=PB4, MOSI=PB5\r\n");
        DebugPrint("  3. Sensor has 3.3V power\r\n");
        DebugPrint("  4. SPI clock speed <= 24MHz\r\n");
        return status;
    }

    DebugPrint("WHO_AM_I (before reset): 0x%02X\r\n", who_am_i);

    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        DebugPrint("WARNING: Unexpected WHO_AM_I value! Expected 0x47\r\n");
        DebugPrint("Attempting to continue anyway...\r\n");
    } else {
        DebugPrint("WHO_AM_I correct! SPI communication working.\r\n");
    }

    // Perform soft reset
    // Per datasheet Section 14.1: "After writing 1 to this bitfield, wait 1ms"
    DebugPrint("Performing soft reset...\r\n");
    status = ICM42688_WriteRegister(ICM42688_DEVICE_CONFIG, 0x01);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Soft reset write failed!\r\n");
        return status;
    }
    HAL_Delay(2);  // Wait 2ms for reset (datasheet says 1ms minimum)

    // Verify WHO_AM_I after reset
    status = ICM42688_ReadWhoAmI(&who_am_i);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Failed to read WHO_AM_I after reset!\r\n");
        return status;
    }

    DebugPrint("WHO_AM_I (after reset): 0x%02X (expected 0x47)\r\n", who_am_i);

    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        DebugPrint("ERROR: WHO_AM_I mismatch! Check connections.\r\n");
        return HAL_ERROR;
    }

    // CRITICAL: Per datasheet Section 12.6, page 58:
    // "For register INT_CONFIG1 bit 4 INT_ASYNC_RESET, user should change
    // setting to 0 from default setting of 1, for proper INT1 and INT2 operation"
    DebugPrint("Configuring INT_ASYNC_RESET...\r\n");
    status = ICM42688_ReadRegister(0x64, &reg_val);  // INT_CONFIG1
    if (status == HAL_OK) {
        reg_val &= ~(1 << 4);  // Clear bit 4 (INT_ASYNC_RESET)
        status = ICM42688_WriteRegister(0x64, reg_val);
        if (status != HAL_OK) {
            DebugPrint("WARNING: Failed to configure INT_ASYNC_RESET\r\n");
        }
    }

    // Per datasheet Section 14.36: "When transitioning from OFF to any of
    // the other modes, do not issue any register writes for 200μs"
    HAL_Delay(1);  // Wait 1ms to be safe

    // Configure power management - enable accel and gyro in low noise mode
    // Per datasheet Section 14.36, Table 12:
    // GYRO_MODE = 11b (Low Noise), ACCEL_MODE = 11b (Low Noise)
    // TEMP_DIS = 0 (Temperature enabled - bit 5 left at 0 by default)
    DebugPrint("Enabling sensors in Low Noise mode...\r\n");
    status = ICM42688_WriteRegister(ICM42688_PWR_MGMT0,
                                     ICM42688_PWR_GYRO_MODE_LN |
                                     ICM42688_PWR_ACCEL_MODE_LN);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Power management config failed!\r\n");
        return status;
    }

    // Per datasheet Section 14.36: "Gyroscope needs to be kept ON for
    // a minimum of 45ms"
    HAL_Delay(50);  // Wait 50ms for sensors to stabilize

    // Configure gyroscope: ±2000 dps, ODR 100Hz
    // Per datasheet Section 14.37
    DebugPrint("Configuring gyroscope (±2000dps, 100Hz)...\r\n");
    status = ICM42688_WriteRegister(ICM42688_GYRO_CONFIG0,
                                     ICM42688_GYRO_FS_2000DPS | ICM42688_ODR_100HZ);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Gyro config failed!\r\n");
        return status;
    }

    // Configure accelerometer: ±16g, ODR 100Hz
    // Per datasheet Section 14.38
    DebugPrint("Configuring accelerometer (±16g, 100Hz)...\r\n");
    status = ICM42688_WriteRegister(ICM42688_ACCEL_CONFIG0,
                                     ICM42688_ACCEL_FS_16G | ICM42688_ODR_100HZ);
    if (status != HAL_OK) {
        DebugPrint("ERROR: Accel config failed!\r\n");
        return status;
    }

    HAL_Delay(50);  // Allow time for configuration to take effect

    DebugPrint("ICM-42688-P initialized successfully!\r\n");
    DebugPrint("===================================\r\n\r\n");

    return HAL_OK;
}

/**
 * @brief Read WHO_AM_I register
 * @param who_am_i: Pointer to store WHO_AM_I value
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *who_am_i) {
    return ICM42688_ReadRegister(ICM42688_WHO_AM_I, who_am_i);
}

/**
 * @brief Read all sensor data (accel, gyro, temp)
 * @param data: Pointer to data structure
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_ReadSensorData(ICM42688_Data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[14];

    // Read all sensor data in one burst (temp + accel + gyro)
    status = ICM42688_ReadRegisters(ICM42688_TEMP_DATA1, raw_data, 14);

    if (status == HAL_OK) {
        // Temperature (16-bit signed, big-endian per default)
        data->temperature = (int16_t)((raw_data[0] << 8) | raw_data[1]);

        // Accelerometer data (16-bit signed, high byte first)
        data->accel_x = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->accel_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        data->accel_z = (int16_t)((raw_data[6] << 8) | raw_data[7]);

        // Gyroscope data (16-bit signed, high byte first)
        data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
        data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
        data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    }

    return status;
}

/**
 * @brief Read accelerometer data
 * @param x, y, z: Pointers to store axis values
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_GetAccelData(int16_t *x, int16_t *y, int16_t *z) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    status = ICM42688_ReadRegisters(ICM42688_ACCEL_DATA_X1, raw_data, 6);

    if (status == HAL_OK) {
        *x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        *y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        *z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    }

    return status;
}

/**
 * @brief Read gyroscope data
 * @param x, y, z: Pointers to store axis values
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_GetGyroData(int16_t *x, int16_t *y, int16_t *z) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    status = ICM42688_ReadRegisters(ICM42688_GYRO_DATA_X1, raw_data, 6);

    if (status == HAL_OK) {
        *x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        *y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        *z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    }

    return status;
}

/**
 * @brief Read temperature
 * @param temp: Pointer to store temperature value
 * @retval HAL status
 */
HAL_StatusTypeDef ICM42688_GetTemperature(int16_t *temp) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[2];

    status = ICM42688_ReadRegisters(ICM42688_TEMP_DATA1, raw_data, 2);

    if (status == HAL_OK) {
        *temp = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    }

    return status;
}

/**
 * @brief Test function to verify sensor operation
 */
void ICM42688_TestSensor(void) {
    ICM42688_Data_t sensor_data;
    int32_t accel_x_mg, accel_y_mg, accel_z_mg;
    int32_t gyro_x_mdps, gyro_y_mdps, gyro_z_mdps;
    int32_t temp_c_x100;

    DebugPrint("\r\n=== Reading ICM-42688-P Sensor Data ===\r\n");

    if (ICM42688_ReadSensorData(&sensor_data) == HAL_OK) {
        // Convert to physical units using integer math (avoids float printf issues)

        // Per datasheet Table 2: ±16g range → sensitivity = 2048 LSB/g
        // Convert to millig (mg): value * 1000 / 2048
        accel_x_mg = ((int32_t)sensor_data.accel_x * 1000) / 2048;
        accel_y_mg = ((int32_t)sensor_data.accel_y * 1000) / 2048;
        accel_z_mg = ((int32_t)sensor_data.accel_z * 1000) / 2048;

        // Per datasheet Table 1: ±2000 dps range → sensitivity = 16.4 LSB/(deg/s)
        // Convert to milli-dps (mdps): value * 1000 / 16.4 ≈ value * 61
        gyro_x_mdps = ((int32_t)sensor_data.gyro_x * 61);
        gyro_y_mdps = ((int32_t)sensor_data.gyro_y * 61);
        gyro_z_mdps = ((int32_t)sensor_data.gyro_z * 61);

        // Per datasheet Section 4.13: Temp (°C) = (TEMP_DATA / 132.48) + 25
        // Check for invalid temperature reading (per Section 12.8)
        if (sensor_data.temperature == -32768) {
            temp_c_x100 = -99900; // Invalid marker
        } else {
            // Convert to temp*100: (value * 100 / 132.48) + 2500
            // Approximation: value * 75 / 100 + 2500
            temp_c_x100 = (((int32_t)sensor_data.temperature * 75) / 100) + 2500;
        }

        // Print raw values
        DebugPrint("Raw Data:\r\n");
        DebugPrint("  Accel: X=%6d Y=%6d Z=%6d\r\n",
                   sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        DebugPrint("  Gyro:  X=%6d Y=%6d Z=%6d\r\n",
                   sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        DebugPrint("  Temp:  %d%s\r\n", sensor_data.temperature,
                   (sensor_data.temperature == -32768) ? " (INVALID)" : "");

        // Print converted values (integers to avoid float printf issues)
        DebugPrint("\r\nConverted Data:\r\n");
        DebugPrint("  Accel: X=%c%ld.%03ld g, Y=%c%ld.%03ld g, Z=%c%ld.%03ld g\r\n",
                   (accel_x_mg < 0) ? '-' : '+',
                   (accel_x_mg < 0 ? -accel_x_mg : accel_x_mg) / 1000,
                   (accel_x_mg < 0 ? -accel_x_mg : accel_x_mg) % 1000,
                   (accel_y_mg < 0) ? '-' : '+',
                   (accel_y_mg < 0 ? -accel_y_mg : accel_y_mg) / 1000,
                   (accel_y_mg < 0 ? -accel_y_mg : accel_y_mg) % 1000,
                   (accel_z_mg < 0) ? '-' : '+',
                   (accel_z_mg < 0 ? -accel_z_mg : accel_z_mg) / 1000,
                   (accel_z_mg < 0 ? -accel_z_mg : accel_z_mg) % 1000);

        DebugPrint("  Gyro:  X=%c%ld.%02ld dps, Y=%c%ld.%02ld dps, Z=%c%ld.%02ld dps\r\n",
                   (gyro_x_mdps < 0) ? '-' : '+',
                   (gyro_x_mdps < 0 ? -gyro_x_mdps : gyro_x_mdps) / 1000,
                   ((gyro_x_mdps < 0 ? -gyro_x_mdps : gyro_x_mdps) % 1000) / 10,
                   (gyro_y_mdps < 0) ? '-' : '+',
                   (gyro_y_mdps < 0 ? -gyro_y_mdps : gyro_y_mdps) / 1000,
                   ((gyro_y_mdps < 0 ? -gyro_y_mdps : gyro_y_mdps) % 1000) / 10,
                   (gyro_z_mdps < 0) ? '-' : '+',
                   (gyro_z_mdps < 0 ? -gyro_z_mdps : gyro_z_mdps) / 1000,
                   ((gyro_z_mdps < 0 ? -gyro_z_mdps : gyro_z_mdps) % 1000) / 10);

        if (temp_c_x100 == -99900) {
            DebugPrint("  Temp:  INVALID (sensor warming up)\r\n");
        } else {
            DebugPrint("  Temp:  %ld.%02ld C\r\n",
                       temp_c_x100 / 100,
                       (temp_c_x100 < 0 ? -temp_c_x100 : temp_c_x100) % 100);
        }

        DebugPrint("=======================================\r\n\r\n");
    } else {
        DebugPrint("ERROR: Failed to read sensor data!\r\n");
    }
}
