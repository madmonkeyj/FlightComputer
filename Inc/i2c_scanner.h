/**
  ******************************************************************************
  * @file    i2c_scanner.h
  * @brief   I2C bus scanner utility
  ******************************************************************************
  */

#ifndef I2C_SCANNER_H_
#define I2C_SCANNER_H_

#include "main.h"
#include "i2c.h"

/* Function prototypes */
void I2C_Scan(void);
uint8_t I2C_ScanQuiet(uint8_t *found_addresses, uint8_t max_addresses);

#endif /* I2C_SCANNER_H_ */
