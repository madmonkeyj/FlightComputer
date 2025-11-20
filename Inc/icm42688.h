/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688-P header - DMA only version
  ******************************************************************************
  */

#ifndef ICM42688_H_
#define ICM42688_H_

#include "main.h"
#include "spi.h"
#include <stdbool.h>

/* Register addresses and constants remain the same */
#define ICM42688_DEVICE_CONFIG      0x11
#define ICM42688_WHO_AM_I           0x75
#define ICM42688_PWR_MGMT0          0x4E
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_ACCEL_CONFIG0      0x50
#define ICM42688_TEMP_DATA1         0x1D
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_GYRO_DATA_X1       0x25
#define ICM42688_WHO_AM_I_VALUE     0x47

#define ICM42688_PWR_GYRO_MODE_LN   0x0C
#define ICM42688_PWR_ACCEL_MODE_LN  0x03
#define ICM42688_GYRO_FS_2000DPS    0x00
#define ICM42688_ACCEL_FS_16G       0x00
#define ICM42688_ODR_1KHZ           0x06

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} ICM42688_Data_t;

/* Function prototypes - REMOVED TestSensor() */
HAL_StatusTypeDef ICM42688_Init(void);
HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *who_am_i);
HAL_StatusTypeDef ICM42688_ReadSensorData(ICM42688_Data_t *data);
HAL_StatusTypeDef ICM42688_GetAccelData(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef ICM42688_GetGyroData(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef ICM42688_GetTemperature(int16_t *temp);
void ICM42688_DMA_Complete_Callback(void);
uint32_t ICM42688_GetDMACallbackCount(void);

#endif /* ICM42688_H_ */
