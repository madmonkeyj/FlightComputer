/**
  ******************************************************************************
  * @file    bmp581.h
  * @brief   BMP581 high-precision barometer driver header - DMA only version
  ******************************************************************************
  */

#ifndef BMP581_H_
#define BMP581_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* BMP581 I2C Address */
#define BMP581_I2C_ADDR             (0x47 << 1)  // SDO to VDD

/* BMP581 Register Addresses */
#define BMP581_REG_CHIP_ID          0x01  // Chip ID register (should read 0x50)
#define BMP581_REG_REV_ID           0x02  // Revision ID
#define BMP581_REG_CHIP_STATUS      0x11  // Chip status
#define BMP581_REG_DRIVE_CONFIG     0x13  // Drive configuration
#define BMP581_REG_INT_CONFIG       0x14  // Interrupt configuration
#define BMP581_REG_INT_SOURCE       0x15  // Interrupt source
#define BMP581_REG_FIFO_CONFIG      0x16  // FIFO configuration
#define BMP581_REG_FIFO_COUNT       0x17  // FIFO fill level
#define BMP581_REG_FIFO_SEL         0x18  // FIFO frame content selection
#define BMP581_REG_TEMP_DATA_XLSB   0x1D  // Temperature XLSB
#define BMP581_REG_TEMP_DATA_LSB    0x1E  // Temperature LSB
#define BMP581_REG_TEMP_DATA_MSB    0x1F  // Temperature MSB
#define BMP581_REG_PRESS_DATA_XLSB  0x20  // Pressure XLSB
#define BMP581_REG_PRESS_DATA_LSB   0x21  // Pressure LSB
#define BMP581_REG_PRESS_DATA_MSB   0x22  // Pressure MSB
#define BMP581_REG_INT_STATUS       0x27  // Interrupt status
#define BMP581_REG_STATUS           0x28  // Status register
#define BMP581_REG_FIFO_DATA        0x29  // FIFO data
#define BMP581_REG_NVM_ADDR         0x2B  // NVM address
#define BMP581_REG_NVM_DATA_LSB     0x2C  // NVM data LSB
#define BMP581_REG_NVM_DATA_MSB     0x2D  // NVM data MSB
#define BMP581_REG_DSP_CONFIG       0x30  // DSP configuration
#define BMP581_REG_DSP_IIR          0x31  // IIR filter configuration
#define BMP581_REG_OOR_CONFIG       0x33  // Out of range configuration
#define BMP581_REG_OOR_RANGE        0x34  // Out of range threshold
#define BMP581_REG_OSR_CONFIG       0x36  // Oversampling configuration
#define BMP581_REG_ODR_CONFIG       0x37  // Output data rate configuration
#define BMP581_REG_OSR_EFF          0x38  // Effective OSR
#define BMP581_REG_CMD              0x7E  // Command register

/* Expected CHIP_ID value */
#define BMP581_CHIP_ID_VALUE        0x50

/* CMD register commands */
#define BMP581_CMD_NOP              0x00  // No operation
#define BMP581_CMD_EXTMODE_EN_MID   0x34  // Enable extended mode (middle byte)
#define BMP581_CMD_SOFT_RESET       0xB6  // Soft reset

/* ODR (Output Data Rate) values for ODR_CONFIG register */
#define BMP581_ODR_240_HZ           0x00  // 240 Hz
#define BMP581_ODR_218_HZ           0x01  // 218 Hz
#define BMP581_ODR_199_HZ           0x02  // 199 Hz
#define BMP581_ODR_179_HZ           0x03  // 179 Hz
#define BMP581_ODR_160_HZ           0x04  // 160 Hz
#define BMP581_ODR_149_HZ           0x05  // 149 Hz
#define BMP581_ODR_140_HZ           0x06  // 140 Hz
#define BMP581_ODR_129_HZ           0x07  // 129 Hz
#define BMP581_ODR_120_HZ           0x08  // 120 Hz
#define BMP581_ODR_110_HZ           0x09  // 110 Hz
#define BMP581_ODR_100_HZ           0x0A  // 100 Hz
#define BMP581_ODR_89_HZ            0x0B  // 89 Hz
#define BMP581_ODR_80_HZ            0x0C  // 80 Hz
#define BMP581_ODR_70_HZ            0x0D  // 70 Hz
#define BMP581_ODR_60_HZ            0x0E  // 60 Hz
#define BMP581_ODR_50_HZ            0x0F  // 50 Hz â† Target for EKF
#define BMP581_ODR_45_HZ            0x10  // 45 Hz
#define BMP581_ODR_40_HZ            0x11  // 40 Hz
#define BMP581_ODR_35_HZ            0x12  // 35 Hz
#define BMP581_ODR_30_HZ            0x13  // 30 Hz
#define BMP581_ODR_25_HZ            0x14  // 25 Hz
#define BMP581_ODR_20_HZ            0x15  // 20 Hz
#define BMP581_ODR_15_HZ            0x16  // 15 Hz
#define BMP581_ODR_10_HZ            0x17  // 10 Hz
#define BMP581_ODR_5_HZ             0x18  // 5 Hz
#define BMP581_ODR_4_HZ             0x19  // 4 Hz
#define BMP581_ODR_3_HZ             0x1A  // 3 Hz
#define BMP581_ODR_2_HZ             0x1B  // 2 Hz
#define BMP581_ODR_1_HZ             0x1C  // 1 Hz
#define BMP581_ODR_0_5_HZ           0x1D  // 0.5 Hz
#define BMP581_ODR_0_25_HZ          0x1E  // 0.25 Hz
#define BMP581_ODR_0_125_HZ         0x1F  // 0.125 Hz

/* Power mode bits in ODR_CONFIG register */
#define BMP581_MODE_STANDBY         0x00  // Standby mode
#define BMP581_MODE_NORMAL          0x01  // Normal mode (continuous)
#define BMP581_MODE_FORCED          0x02  // Forced mode (one-shot)
#define BMP581_MODE_NONSTOP         0x03  // Non-stop mode
#define BMP581_MODE_SHIFT           5     // Mode bits are [6:5]

/* OSR (Oversampling) values for OSR_CONFIG register */
#define BMP581_OSR_P_1X             0x00  // Pressure oversampling x1
#define BMP581_OSR_P_2X             0x01  // Pressure oversampling x2
#define BMP581_OSR_P_4X             0x02  // Pressure oversampling x4
#define BMP581_OSR_P_8X             0x03  // Pressure oversampling x8
#define BMP581_OSR_P_16X            0x04  // Pressure oversampling x16
#define BMP581_OSR_P_32X            0x05  // Pressure oversampling x32
#define BMP581_OSR_P_64X            0x06  // Pressure oversampling x64
#define BMP581_OSR_P_128X           0x07  // Pressure oversampling x128

#define BMP581_OSR_T_1X             0x00  // Temperature oversampling x1
#define BMP581_OSR_T_2X             0x01  // Temperature oversampling x2
#define BMP581_OSR_T_4X             0x02  // Temperature oversampling x4
#define BMP581_OSR_T_8X             0x03  // Temperature oversampling x8
#define BMP581_OSR_T_16X            0x04  // Temperature oversampling x16
#define BMP581_OSR_T_32X            0x05  // Temperature oversampling x32
#define BMP581_OSR_T_64X            0x06  // Temperature oversampling x64
#define BMP581_OSR_T_128X           0x07  // Temperature oversampling x128

#define BMP581_OSR_T_SHIFT          3     // Temperature OSR bits are [5:3]

/* IIR Filter coefficients for DSP_IIR register */
#define BMP581_IIR_BYPASS           0x00  // IIR filter bypassed
#define BMP581_IIR_COEFF_1          0x01  // IIR coefficient 1
#define BMP581_IIR_COEFF_3          0x02  // IIR coefficient 3
#define BMP581_IIR_COEFF_7          0x03  // IIR coefficient 7
#define BMP581_IIR_COEFF_15         0x04  // IIR coefficient 15
#define BMP581_IIR_COEFF_31         0x05  // IIR coefficient 31
#define BMP581_IIR_COEFF_63         0x06  // IIR coefficient 63
#define BMP581_IIR_COEFF_127        0x07  // IIR coefficient 127

/* STATUS register bits */
#define BMP581_STATUS_POR_DETECTED  0x10  // Power-on reset detected
#define BMP581_STATUS_DRDY_PRESS    0x08  // Pressure data ready
#define BMP581_STATUS_DRDY_TEMP     0x04  // Temperature data ready

/* INT_STATUS register bits */
#define BMP581_INT_STATUS_DRDY      0x01  // Data ready interrupt
#define BMP581_INT_STATUS_FIFO_FULL 0x02  // FIFO full interrupt
#define BMP581_INT_STATUS_FIFO_TH   0x04  // FIFO threshold interrupt
#define BMP581_INT_STATUS_POR       0x08  // POR interrupt

/* Data structure for sensor readings */
typedef struct {
    int32_t pressure_raw;       // Raw 24-bit pressure data
    int32_t temperature_raw;    // Raw 24-bit temperature data
    float pressure_pa;          // Pressure in Pascals
    float temperature_c;        // Temperature in Celsius
    float altitude_m;           // Calculated altitude in meters (requires sea level pressure)
} BMP581_Data_t;

/* Configuration structure */
typedef enum {
    BMP581_ODR_50HZ = BMP581_ODR_50_HZ,
    BMP581_ODR_100HZ = BMP581_ODR_100_HZ,
    BMP581_ODR_200HZ = BMP581_ODR_199_HZ
} BMP581_ODR_t;

typedef enum {
    BMP581_OSR_1X = 0,
    BMP581_OSR_2X = 1,
    BMP581_OSR_4X = 2,
    BMP581_OSR_8X = 3,
    BMP581_OSR_16X = 4,
    BMP581_OSR_32X = 5
} BMP581_OSR_t;

typedef enum {
    BMP581_IIR_OFF = BMP581_IIR_BYPASS,
    BMP581_IIR_2 = BMP581_IIR_COEFF_1,
    BMP581_IIR_4 = BMP581_IIR_COEFF_3,
    BMP581_IIR_8 = BMP581_IIR_COEFF_7,
    BMP581_IIR_16 = BMP581_IIR_COEFF_15,
    BMP581_IIR_32 = BMP581_IIR_COEFF_31
} BMP581_IIR_t;

/* Function prototypes */
HAL_StatusTypeDef BMP581_Init(void);
HAL_StatusTypeDef BMP581_ReadChipID(uint8_t *chip_id);
HAL_StatusTypeDef BMP581_ReadSensorData(BMP581_Data_t *data);
HAL_StatusTypeDef BMP581_GetPressure(float *pressure_pa);
HAL_StatusTypeDef BMP581_GetTemperature(float *temperature_c);
HAL_StatusTypeDef BMP581_GetAltitude(float *altitude_m, float sea_level_pa);
HAL_StatusTypeDef BMP581_CheckDataReady(bool *ready);
HAL_StatusTypeDef BMP581_SetODR(BMP581_ODR_t odr);
HAL_StatusTypeDef BMP581_SetOversampling(BMP581_OSR_t press_osr, BMP581_OSR_t temp_osr);
HAL_StatusTypeDef BMP581_SetIIRFilter(BMP581_IIR_t iir_coeff);
HAL_StatusTypeDef BMP581_SoftReset(void);

/* DMA completion callback */
void BMP581_DMA_Complete_Callback(void);
uint32_t BMP581_GetDMACallbackCount(void);

#endif /* BMP581_H_ */






