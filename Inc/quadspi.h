/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.h
  * @brief   Modular QSPI Flash Driver for STM32G4xx (Winbond W25Q32)
  * @version 2.0 - Refactored for modularity and reusability
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __QUADSPI_H__
#define __QUADSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================================
   PUBLIC CONSTANTS - Memory Parameters
   ========================================================================= */
#define MEMORY_FLASH_SIZE    0x400000  /* 32Mbit = 4MB */
#define MEMORY_SECTOR_SIZE   0x1000    /* 4KB sectors */
#define MEMORY_PAGE_SIZE     0x100     /* 256 byte pages */

/* ============================================================================
   PUBLIC API - Initialization
   ========================================================================= */

/**
 * @brief  Initialize QSPI peripheral with timing workarounds
 * @note   Must be called before any other QSPI operations
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Simple_Init(void);

/**
 * @brief  Read flash chip JEDEC ID
 * @param  id_buffer: Pointer to 3-byte buffer for ID (Manufacturer, Type, Capacity)
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Read_ID(uint8_t *id_buffer);

/* ============================================================================
   PUBLIC API - Erase Operations
   ========================================================================= */

/**
 * @brief  Erase 4KB sector at specified address
 * @param  address: Sector address (will be aligned to 4KB boundary)
 * @note   Takes ~50-300ms per sector
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Simple_Erase(uint32_t address);

/**
 * @brief  Erase entire 4MB flash chip
 * @note   Takes ~30-40 seconds for full chip
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t CSP_QSPI_Erase_Chip(void);

/* ============================================================================
   PUBLIC API - Write Operations (Choose ONE or BOTH)
   ========================================================================= */

/**
 * @brief  Write data using Quad SPI mode (blocking)
 * @param  buffer: Pointer to data buffer
 * @param  address: Flash start address
 * @param  size: Number of bytes to write
 * @note   Handles page boundary crossing automatically
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Quad_Write(uint8_t *buffer, uint32_t address, uint32_t size);

/**
 * @brief  Write data using Quad SPI mode with DMA (non-blocking)
 * @param  buffer: Pointer to data buffer (must remain valid during transfer)
 * @param  address: Flash start address
 * @param  size: Number of bytes to write
 * @note   Uses DMA for faster transfers, callbacks signal completion
 * @note   Ensure DMA is initialized before calling
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Quad_Write_DMA(uint8_t *buffer, uint32_t address, uint32_t size);

/* ============================================================================
   PUBLIC API - Read Operations
   ========================================================================= */

/**
 * @brief  Read data using Quad SPI mode
 * @param  buffer: Pointer to receive buffer
 * @param  address: Flash start address
 * @param  size: Number of bytes to read (can read entire chip: 0x400000)
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
uint8_t QSPI_Quad_Read(uint8_t *buffer, uint32_t address, uint32_t size);

/* ============================================================================
   INTERNAL FUNCTIONS - Do not call directly from application code
   ========================================================================= */

extern QSPI_HandleTypeDef hqspi1;

void MX_QUADSPI1_Init(void);
uint8_t QSPI_Timing_Workaround(void);
uint8_t QSPI_AutoPollingMemReady(void);
uint8_t QSPI_Fast_AutoPolling(void);
uint8_t QSPI_AutoPoll_IT(void);

/* ============================================================================
   PRIVATE DEFINES - Flash Commands (Internal Use)
   ========================================================================= */
#define CHIP_ERASE_CMD           0xC7
#define READ_STATUS_REG_CMD      0x05
#define WRITE_ENABLE_CMD         0x06
#define SECTOR_ERASE_CMD         0x20
#define QUAD_IN_FAST_PROG_CMD    0x32
#define QUAD_OUT_FAST_READ_CMD   0x6B
#define DUMMY_CLOCK_CYCLES_READ_QUAD 8

#ifdef __cplusplus
}
#endif

#endif /* __QUADSPI_H__ */
