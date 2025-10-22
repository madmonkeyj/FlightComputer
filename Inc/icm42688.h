/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688-P 6-axis IMU driver header
  ******************************************************************************
  */

#ifndef ICM42688_H_
#define ICM42688_H_

#include "main.h"
#include "spi.h"
#include <stdbool.h>

/* ICM-42688-P Register Addresses */
#define ICM42688_DEVICE_CONFIG      0x11
#define ICM42688_WHO_AM_I           0x75
#define ICM42688_REG_BANK_SEL       0x76
#define ICM42688_PWR_MGMT0          0x4E
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_ACCEL_CONFIG0      0x50
#define ICM42688_GYRO_CONFIG1       0x51
#define ICM42688_ACCEL_CONFIG1      0x53
#define ICM42688_INT_CONFIG         0x14
#define ICM42688_TEMP_DATA1         0x1D
#define ICM42688_TEMP_DATA0         0x1E
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_ACCEL_DATA_X0      0x20
#define ICM42688_ACCEL_DATA_Y1      0x21
#define ICM42688_ACCEL_DATA_Y0      0x22
#define ICM42688_ACCEL_DATA_Z1      0x23
#define ICM42688_ACCEL_DATA_Z0      0x24
#define ICM42688_GYRO_DATA_X1       0x25
#define ICM42688_GYRO_DATA_X0       0x26
#define ICM42688_GYRO_DATA_Y1       0x27
#define ICM42688_GYRO_DATA_Y0       0x28
#define ICM42688_GYRO_DATA_Z1       0x29
#define ICM42688_GYRO_DATA_Z0       0x2A

/* Self-test registers */
#define ICM42688_SELF_TEST_CONFIG   0x70  // Bank 0
#define ICM42688_XG_ST_DATA         0x5F  // Bank 1
#define ICM42688_YG_ST_DATA         0x60  // Bank 1
#define ICM42688_ZG_ST_DATA         0x61  // Bank 1
#define ICM42688_XA_ST_DATA         0x3B  // Bank 2
#define ICM42688_YA_ST_DATA         0x3C  // Bank 2
#define ICM42688_ZA_ST_DATA         0x3D  // Bank 2

/* WHO_AM_I expected value */
#define ICM42688_WHO_AM_I_VALUE     0x47

/* Power Management */
#define ICM42688_PWR_TEMP_DIS       0x20  // Bit 5: 1=Disable temp sensor
#define ICM42688_PWR_IDLE           0x10
#define ICM42688_PWR_GYRO_MODE_LN   0x0C
#define ICM42688_PWR_ACCEL_MODE_LN  0x03

/* Gyro Full Scale Range */
#define ICM42688_GYRO_FS_2000DPS    0x00
#define ICM42688_GYRO_FS_1000DPS    0x20
#define ICM42688_GYRO_FS_500DPS     0x40
#define ICM42688_GYRO_FS_250DPS     0x60

/* Accel Full Scale Range */
#define ICM42688_ACCEL_FS_16G       0x00
#define ICM42688_ACCEL_FS_8G        0x20
#define ICM42688_ACCEL_FS_4G        0x40
#define ICM42688_ACCEL_FS_2G        0x60

/* ODR (Output Data Rate) */
#define ICM42688_ODR_32KHZ          0x01
#define ICM42688_ODR_16KHZ          0x02
#define ICM42688_ODR_8KHZ           0x03
#define ICM42688_ODR_4KHZ           0x04
#define ICM42688_ODR_2KHZ           0x05
#define ICM42688_ODR_1KHZ           0x06
#define ICM42688_ODR_200HZ          0x07
#define ICM42688_ODR_100HZ          0x08
#define ICM42688_ODR_50HZ           0x09
#define ICM42688_ODR_25HZ           0x0A

/* Self-test configuration bits */
#define ICM42688_ST_ACCEL_POWER     0x80
#define ICM42688_ST_EN_AZ           0x20
#define ICM42688_ST_EN_AY           0x10
#define ICM42688_ST_EN_AX           0x08
#define ICM42688_ST_EN_GZ           0x04
#define ICM42688_ST_EN_GY           0x02
#define ICM42688_ST_EN_GX           0x01

/* Data structure for sensor readings */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} ICM42688_Data_t;

/* Self-test result structure */
typedef struct {
    uint8_t gyro_x_pass;
    uint8_t gyro_y_pass;
    uint8_t gyro_z_pass;
    uint8_t accel_x_pass;
    uint8_t accel_y_pass;
    uint8_t accel_z_pass;
    int16_t gyro_x_response;
    int16_t gyro_y_response;
    int16_t gyro_z_response;
    int16_t accel_x_response;
    int16_t accel_y_response;
    int16_t accel_z_response;
} ICM42688_SelfTest_t;

/* Function prototypes */
HAL_StatusTypeDef ICM42688_Init(void);
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *who_am_i);
HAL_StatusTypeDef ICM42688_ReadSensorData(ICM42688_Data_t *data);
HAL_StatusTypeDef ICM42688_GetAccelData(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef ICM42688_GetGyroData(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef ICM42688_GetTemperature(int16_t *temp);
void ICM42688_TestSensor(void);

/* Self-test function */
HAL_StatusTypeDef ICM42688_SelfTest(ICM42688_SelfTest_t *result);

#endif /* ICM42688_H_ */
