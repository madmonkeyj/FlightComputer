/**
  ******************************************************************************
  * @file    h3lis331dl.h
  * @brief   H3LIS331DL high-g 3-axis accelerometer driver header
  ******************************************************************************
  */

#ifndef H3LIS331DL_H_
#define H3LIS331DL_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* H3LIS331DL I2C Address */
#define H3LIS331DL_I2C_ADDR_LOW     (0x18 << 1)  // SA0 connected to GND
#define H3LIS331DL_I2C_ADDR_HIGH    (0x19 << 1)  // SA0 connected to VDD
#define H3LIS331DL_I2C_ADDR         H3LIS331DL_I2C_ADDR_LOW  // Default address

/* H3LIS331DL Register Addresses */
#define H3LIS331DL_REG_WHO_AM_I     0x0F  // Device identification register
#define H3LIS331DL_REG_CTRL_REG1    0x20  // Control register 1
#define H3LIS331DL_REG_CTRL_REG2    0x21  // Control register 2
#define H3LIS331DL_REG_CTRL_REG3    0x22  // Control register 3
#define H3LIS331DL_REG_CTRL_REG4    0x23  // Control register 4
#define H3LIS331DL_REG_CTRL_REG5    0x24  // Control register 5
#define H3LIS331DL_REG_HP_FILTER_RESET  0x25  // HP filter reset
#define H3LIS331DL_REG_REFERENCE    0x26  // Reference value for HP filter
#define H3LIS331DL_REG_STATUS       0x27  // Status register
#define H3LIS331DL_REG_OUT_X_L      0x28  // X-axis output LSB
#define H3LIS331DL_REG_OUT_X_H      0x29  // X-axis output MSB
#define H3LIS331DL_REG_OUT_Y_L      0x2A  // Y-axis output LSB
#define H3LIS331DL_REG_OUT_Y_H      0x2B  // Y-axis output MSB
#define H3LIS331DL_REG_OUT_Z_L      0x2C  // Z-axis output LSB
#define H3LIS331DL_REG_OUT_Z_H      0x2D  // Z-axis output MSB
#define H3LIS331DL_REG_INT1_CFG     0x30  // Interrupt 1 configuration
#define H3LIS331DL_REG_INT1_SRC     0x31  // Interrupt 1 source
#define H3LIS331DL_REG_INT1_THS     0x32  // Interrupt 1 threshold
#define H3LIS331DL_REG_INT1_DURATION 0x33  // Interrupt 1 duration
#define H3LIS331DL_REG_INT2_CFG     0x34  // Interrupt 2 configuration
#define H3LIS331DL_REG_INT2_SRC     0x35  // Interrupt 2 source
#define H3LIS331DL_REG_INT2_THS     0x36  // Interrupt 2 threshold
#define H3LIS331DL_REG_INT2_DURATION 0x37  // Interrupt 2 duration

/* WHO_AM_I expected value */
#define H3LIS331DL_WHO_AM_I_VALUE   0x32

/* CTRL_REG1 bits (0x20) */
#define H3LIS331DL_CTRL1_PM_POWER_DOWN  0x00  // Power-down mode
#define H3LIS331DL_CTRL1_PM_NORMAL      0x20  // Normal mode (001)
#define H3LIS331DL_CTRL1_PM_LP_0_5HZ    0x40  // Low-power 0.5 Hz (010)
#define H3LIS331DL_CTRL1_PM_LP_1HZ      0x60  // Low-power 1 Hz (011)
#define H3LIS331DL_CTRL1_PM_LP_2HZ      0x80  // Low-power 2 Hz (100)
#define H3LIS331DL_CTRL1_PM_LP_5HZ      0xA0  // Low-power 5 Hz (101)
#define H3LIS331DL_CTRL1_PM_LP_10HZ     0xC0  // Low-power 10 Hz (110)

#define H3LIS331DL_CTRL1_DR_50HZ        0x00  // 50 Hz output data rate
#define H3LIS331DL_CTRL1_DR_100HZ       0x08  // 100 Hz output data rate
#define H3LIS331DL_CTRL1_DR_400HZ       0x10  // 400 Hz output data rate
#define H3LIS331DL_CTRL1_DR_1000HZ      0x18  // 1000 Hz output data rate

#define H3LIS331DL_CTRL1_ZEN            0x04  // Z-axis enable
#define H3LIS331DL_CTRL1_YEN            0x02  // Y-axis enable
#define H3LIS331DL_CTRL1_XEN            0x01  // X-axis enable

/* CTRL_REG2 bits (0x21) - High-pass filter configuration */
#define H3LIS331DL_CTRL2_BOOT           0x80  // Reboot memory content
#define H3LIS331DL_CTRL2_HPM_NORMAL0    0x00  // Normal mode (reset reading HP_FILTER_RESET)
#define H3LIS331DL_CTRL2_HPM_REFERENCE  0x20  // Reference signal for filtering
#define H3LIS331DL_CTRL2_HPM_NORMAL1    0x40  // Normal mode
#define H3LIS331DL_CTRL2_FDS            0x10  // Filtered data selection
#define H3LIS331DL_CTRL2_HPEN2          0x08  // High-pass filter enabled for INT2
#define H3LIS331DL_CTRL2_HPEN1          0x04  // High-pass filter enabled for INT1
#define H3LIS331DL_CTRL2_HPCF_8         0x00  // HPc = 8
#define H3LIS331DL_CTRL2_HPCF_16        0x01  // HPc = 16
#define H3LIS331DL_CTRL2_HPCF_32        0x02  // HPc = 32
#define H3LIS331DL_CTRL2_HPCF_64        0x03  // HPc = 64

/* CTRL_REG3 bits (0x22) - Interrupt control */
#define H3LIS331DL_CTRL3_IHL            0x80  // Interrupt active low
#define H3LIS331DL_CTRL3_PP_OD          0x40  // Push-pull/Open drain
#define H3LIS331DL_CTRL3_LIR2           0x20  // Latch interrupt request on INT2
#define H3LIS331DL_CTRL3_I2_CFG1        0x10  // Data signal on INT2 config bit 1
#define H3LIS331DL_CTRL3_I2_CFG0        0x08  // Data signal on INT2 config bit 0
#define H3LIS331DL_CTRL3_LIR1           0x04  // Latch interrupt request on INT1
#define H3LIS331DL_CTRL3_I1_CFG1        0x02  // Data signal on INT1 config bit 1
#define H3LIS331DL_CTRL3_I1_CFG0        0x01  // Data signal on INT1 config bit 0

/* CTRL_REG4 bits (0x23) */
#define H3LIS331DL_CTRL4_BDU            0x80  // Block data update
#define H3LIS331DL_CTRL4_BLE            0x40  // Big/Little endian data selection
#define H3LIS331DL_CTRL4_FS_100G        0x00  // ±100g full scale
#define H3LIS331DL_CTRL4_FS_200G        0x10  // ±200g full scale
#define H3LIS331DL_CTRL4_FS_400G        0x30  // ±400g full scale
#define H3LIS331DL_CTRL4_SIM            0x01  // SPI 3-wire mode

/* CTRL_REG5 bits (0x24) - Sleep-to-wake configuration */
#define H3LIS331DL_CTRL5_TURNON_SLEEP_WAKE  0x03  // Sleep-to-wake enabled

/* STATUS_REG bits (0x27) */
#define H3LIS331DL_STATUS_ZYXOR         0x80  // X, Y, Z data overrun
#define H3LIS331DL_STATUS_ZOR           0x40  // Z data overrun
#define H3LIS331DL_STATUS_YOR           0x20  // Y data overrun
#define H3LIS331DL_STATUS_XOR           0x10  // X data overrun
#define H3LIS331DL_STATUS_ZYXDA         0x08  // X, Y, Z data available
#define H3LIS331DL_STATUS_ZDA           0x04  // Z data available
#define H3LIS331DL_STATUS_YDA           0x02  // Y data available
#define H3LIS331DL_STATUS_XDA           0x01  // X data available

/* Sensitivity values (LSB/g) for 12-bit mode */
#define H3LIS331DL_SENSITIVITY_100G     49.0f   // mg/digit at ±100g
#define H3LIS331DL_SENSITIVITY_200G     98.0f   // mg/digit at ±200g
#define H3LIS331DL_SENSITIVITY_400G     195.0f  // mg/digit at ±400g

/* Full scale range constants */
#define H3LIS331DL_RANGE_100G           100.0f  // ±100g
#define H3LIS331DL_RANGE_200G           200.0f  // ±200g
#define H3LIS331DL_RANGE_400G           400.0f  // ±400g

/* I2C auto-increment bit for multi-byte read */
#define H3LIS331DL_I2C_AUTO_INCREMENT   0x80

/* Data structure for sensor configuration */
typedef enum {
    H3LIS331DL_FS_100G = 0,
    H3LIS331DL_FS_200G = 1,
    H3LIS331DL_FS_400G = 2
} H3LIS331DL_FullScale_t;

typedef enum {
    H3LIS331DL_ODR_50HZ = 0,
    H3LIS331DL_ODR_100HZ = 1,
    H3LIS331DL_ODR_400HZ = 2,
    H3LIS331DL_ODR_1000HZ = 3
} H3LIS331DL_ODR_t;

/* Data structure for sensor readings */
typedef struct {
    int16_t accel_x;      // X-axis acceleration (raw value)
    int16_t accel_y;      // Y-axis acceleration (raw value)
    int16_t accel_z;      // Z-axis acceleration (raw value)
    float accel_x_g;      // X-axis acceleration in g
    float accel_y_g;      // Y-axis acceleration in g
    float accel_z_g;      // Z-axis acceleration in g
} H3LIS331DL_Data_t;

/* Self-test result structure */
typedef struct {
    uint8_t x_pass;
    uint8_t y_pass;
    uint8_t z_pass;
    int16_t x_response;
    int16_t y_response;
    int16_t z_response;
} H3LIS331DL_SelfTest_t;

/* Function prototypes */
HAL_StatusTypeDef H3LIS331DL_Init(void);
HAL_StatusTypeDef H3LIS331DL_ReadWhoAmI(uint8_t *who_am_i);
HAL_StatusTypeDef H3LIS331DL_ReadSensorData(H3LIS331DL_Data_t *data);
HAL_StatusTypeDef H3LIS331DL_GetAccelData(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef H3LIS331DL_GetAccelData_g(float *x_g, float *y_g, float *z_g);
HAL_StatusTypeDef H3LIS331DL_SetFullScale(H3LIS331DL_FullScale_t fs);
HAL_StatusTypeDef H3LIS331DL_SetOutputDataRate(H3LIS331DL_ODR_t odr);
HAL_StatusTypeDef H3LIS331DL_CheckDataReady(bool *ready);
HAL_StatusTypeDef H3LIS331DL_SoftReset(void);
void H3LIS331DL_TestSensor(void);
void H3LIS331DL_DiagnosticTest(void);

/* DMA completion callback */
void H3LIS331DL_DMA_Complete_Callback(void);
uint32_t H3LIS331DL_GetDMACallbackCount(void);

#endif /* H3LIS331DL_H_ */
