/**
  ******************************************************************************
  * @file    mmc5983ma.h
  * @brief   MMC5983MA header - DMA only version
  ******************************************************************************
  */

#ifndef MMC5983MA_H_
#define MMC5983MA_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* Register addresses and constants remain the same */
#define MMC5983MA_I2C_ADDR          (0x30 << 1)
#define MMC5983MA_REG_XOUT_0        0x00
#define MMC5983MA_REG_TEMP          0x07
#define MMC5983MA_REG_STATUS        0x08
#define MMC5983MA_REG_CONTROL_0     0x09
#define MMC5983MA_REG_CONTROL_1     0x0A
#define MMC5983MA_REG_CONTROL_2     0x0B
#define MMC5983MA_REG_CONTROL_3     0x0C
#define MMC5983MA_REG_PRODUCT_ID    0x2F

#define MMC5983MA_STATUS_MEAS_M_DONE    0x01
#define MMC5983MA_CTRL0_TM_M            0x01
#define MMC5983MA_CTRL0_SET             0x08
#define MMC5983MA_CTRL0_RESET           0x10
#define MMC5983MA_CTRL0_AUTO_SR_EN      0x20
#define MMC5983MA_CTRL1_SW_RST          0x80
#define MMC5983MA_CTRL1_BW_800HZ        0x03
#define MMC5983MA_CTRL2_CM_FREQ_1000HZ  0x07
#define MMC5983MA_CTRL2_CMM_EN          0x08
#define MMC5983MA_CTRL2_PRD_SET_100     0x30
#define MMC5983MA_CTRL2_EN_PRD_SET      0x80
#define MMC5983MA_CTRL3_ST_ENP          0x10
#define MMC5983MA_CTRL3_ST_ENM          0x20

#define MMC5983MA_PRODUCT_ID            0x30
#define MMC5983MA_NULL_FIELD            131072

typedef struct {
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
    uint8_t temperature;
} MMC5983MA_Data_t;

typedef struct {
    uint8_t x_pass;
    uint8_t y_pass;
    uint8_t z_pass;
    int32_t x_response;
    int32_t y_response;
    int32_t z_response;
} MMC5983MA_SelfTest_t;

/* Function prototypes - REMOVED TestSensor() */
HAL_StatusTypeDef MMC5983MA_Init(void);
HAL_StatusTypeDef MMC5983MA_ReadProductID(uint8_t *product_id);
HAL_StatusTypeDef MMC5983MA_ReadSensorData(MMC5983MA_Data_t *data);
HAL_StatusTypeDef MMC5983MA_GetMagData(int32_t *x, int32_t *y, int32_t *z);
HAL_StatusTypeDef MMC5983MA_GetTemperature(uint8_t *temp);
HAL_StatusTypeDef MMC5983MA_PerformSET(void);
HAL_StatusTypeDef MMC5983MA_PerformRESET(void);
HAL_StatusTypeDef MMC5983MA_TriggerMeasurement(void);
HAL_StatusTypeDef MMC5983MA_CheckMeasurementDone(bool *done);
HAL_StatusTypeDef MMC5983MA_SelfTest(MMC5983MA_SelfTest_t *result);
void MMC5983MA_DMA_Complete_Callback(void);
uint32_t MMC5983MA_GetDMACallbackCount(void);

#endif /* MMC5983MA_H_ */
