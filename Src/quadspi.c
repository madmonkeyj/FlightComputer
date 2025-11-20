/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   Modular QSPI Flash Driver Implementation
  * @version 2.0 - Refactored for modularity and reusability
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "quadspi.h"

/* USER CODE BEGIN 0 */

/* ============================================================================
   PRIVATE FUNCTION PROTOTYPES
   ========================================================================= */
static uint8_t QSPI_WriteEnable(void);

/* DMA completion flags for non-blocking operations */
volatile uint8_t qspi_tx_complete = 0;
volatile uint8_t qspi_error = 0;

/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi1;
DMA_HandleTypeDef hdma_quadspi;

/* QUADSPI1 init function */
void MX_QUADSPI1_Init(void)
{

  /* USER CODE BEGIN QUADSPI1_Init 0 */

  /* USER CODE END QUADSPI1_Init 0 */

  /* USER CODE BEGIN QUADSPI1_Init 1 */

  /* USER CODE END QUADSPI1_Init 1 */
  hqspi1.Instance = QUADSPI;
  hqspi1.Init.ClockPrescaler = 0;
  hqspi1.Init.FifoThreshold = 1;
  hqspi1.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi1.Init.FlashSize = 21;
  hqspi1.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi1.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi1.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi1.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI1_Init 2 */

  /* USER CODE END QUADSPI1_Init 2 */

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInit.QspiClockSelection = RCC_QSPICLKSOURCE_SYSCLK;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**QUADSPI1 GPIO Configuration
    PA6     ------> QUADSPI1_BK1_IO3
    PA7     ------> QUADSPI1_BK1_IO2
    PB0     ------> QUADSPI1_BK1_IO1
    PB1     ------> QUADSPI1_BK1_IO0
    PB10     ------> QUADSPI1_CLK
    PB11     ------> QUADSPI1_BK1_NCS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* QUADSPI DMA Init */
    /* QUADSPI Init */
    hdma_quadspi.Instance = DMA1_Channel7;
    hdma_quadspi.Init.Request = DMA_REQUEST_QUADSPI;
    hdma_quadspi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_quadspi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_quadspi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_quadspi.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_quadspi.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_quadspi.Init.Mode = DMA_NORMAL;
    hdma_quadspi.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_quadspi) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(qspiHandle,hdma,hdma_quadspi);

    /* QUADSPI interrupt Init */
    HAL_NVIC_SetPriority(QUADSPI_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);
  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI1 GPIO Configuration
    PA6     ------> QUADSPI1_BK1_IO3
    PA7     ------> QUADSPI1_BK1_IO2
    PB0     ------> QUADSPI1_BK1_IO1
    PB1     ------> QUADSPI1_BK1_IO0
    PB10     ------> QUADSPI1_CLK
    PB11     ------> QUADSPI1_BK1_NCS
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11);

    /* QUADSPI DMA DeInit */
    HAL_DMA_DeInit(qspiHandle->hdma);

    /* QUADSPI interrupt Deinit */
    HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ============================================================================
   PUBLIC API - INITIALIZATION
   ========================================================================= */

uint8_t QSPI_Timing_Workaround(void)
{
  /* Workaround for STM32G4 QSPI timing errata 2.6.2 */
  CLEAR_BIT(hqspi1.Instance->CR, QUADSPI_CR_EN);
  while(hqspi1.Instance->SR & QUADSPI_SR_BUSY) {}

  hqspi1.Instance->CR = 0xFF000001;  /* Max prescaler + enable */
  hqspi1.Instance->CCR = 0x20000000; /* Free-running clock */
  hqspi1.Instance->CCR = 0x20000000; /* Repeated per errata */

  CLEAR_BIT(hqspi1.Instance->CR, QUADSPI_CR_EN);
  while(hqspi1.Instance->SR & QUADSPI_SR_BUSY) {}

  return HAL_OK;
}

uint8_t QSPI_Simple_Init(void)
{
  /* Basic initialization */
  MX_QUADSPI1_Init();

  /* Apply timing workaround for STM32G4 errata */
  CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_EN);
  while(QUADSPI->SR & QUADSPI_SR_BUSY) {}

  QUADSPI->CR = 0xFF000001;
  QUADSPI->CCR = 0x20000000;
  QUADSPI->CCR = 0x20000000;

  CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_EN);
  while(QUADSPI->SR & QUADSPI_SR_BUSY) {}

  SET_BIT(QUADSPI->CR, QUADSPI_CR_EN);

  return HAL_OK;
}

uint8_t QSPI_Read_ID(uint8_t *id_buffer)
{
  QSPI_CommandTypeDef sCommand = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x9F;  /* JEDEC ID command */
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.NbData = 3;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  if (HAL_QSPI_Receive(&hqspi1, id_buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/* ============================================================================
   PUBLIC API - ERASE OPERATIONS
   ========================================================================= */

uint8_t QSPI_Simple_Erase(uint32_t address)
{
  QSPI_CommandTypeDef sCommand = {0};

  /* Enable write */
  if (QSPI_WriteEnable() != HAL_OK)
    return HAL_ERROR;

  /* 4KB Sector Erase */
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = SECTOR_ERASE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  /* Wait for erase completion */
  uint8_t status = 0;
  do {
    HAL_Delay(10);

    sCommand.Instruction = READ_STATUS_REG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_NONE;
    sCommand.DataMode = QSPI_DATA_1_LINE;
    sCommand.NbData = 1;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return HAL_ERROR;

    if (HAL_QSPI_Receive(&hqspi1, &status, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return HAL_ERROR;
  } while (status & 0x01);

  return HAL_OK;
}

uint8_t CSP_QSPI_Erase_Chip(void)
{
  QSPI_CommandTypeDef sCommand = {0};

  if (QSPI_WriteEnable() != HAL_OK)
    return HAL_ERROR;

  /* Chip Erase Command */
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = CHIP_ERASE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_MAX_DELAY) != HAL_OK)
    return HAL_ERROR;

  /* Wait for chip erase (takes ~30 seconds) */
  if (QSPI_AutoPollingMemReady() != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/* ============================================================================
   PUBLIC API - WRITE OPERATIONS
   ========================================================================= */

uint8_t QSPI_Quad_Write(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint32_t current_addr = address;
  uint32_t remaining = size;
  uint32_t page_offset, write_size;

  while (remaining > 0) {
    /* Calculate write size respecting page boundaries */
    page_offset = current_addr % MEMORY_PAGE_SIZE;
    write_size = MEMORY_PAGE_SIZE - page_offset;
    if (write_size > remaining)
      write_size = remaining;

    /* Write Enable */
    if (QSPI_WriteEnable() != HAL_OK)
      return HAL_ERROR;

    /* Quad Page Program */
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = current_addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.DummyCycles = 0;
    sCommand.NbData = write_size;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return HAL_ERROR;

    if (HAL_QSPI_Transmit(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return HAL_ERROR;

    if (QSPI_Fast_AutoPolling() != HAL_OK)
      return HAL_ERROR;

    /* Move to next page */
    buffer += write_size;
    current_addr += write_size;
    remaining -= write_size;
  }

  return HAL_OK;
}

uint8_t QSPI_Quad_Write_DMA(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint32_t current_addr = address;
  uint32_t remaining = size;
  uint32_t page_offset, write_size;

  while (remaining > 0) {
    page_offset = current_addr % MEMORY_PAGE_SIZE;
    write_size = MEMORY_PAGE_SIZE - page_offset;
    if (write_size > remaining)
      write_size = remaining;

    /* Write Enable */
    if (QSPI_WriteEnable() != HAL_OK)
      return HAL_ERROR;

    /* Quad Page Program */
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = current_addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.DummyCycles = 0;
    sCommand.NbData = write_size;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return HAL_ERROR;

    /* DMA transmit */
    qspi_tx_complete = 0;
    qspi_error = 0;

    if (HAL_QSPI_Transmit_DMA(&hqspi1, buffer) != HAL_OK)
      return HAL_ERROR;

    /* Wait for DMA completion */
    uint32_t timeout = HAL_GetTick() + 1000;
    while (!qspi_tx_complete && !qspi_error && HAL_GetTick() < timeout)
      __NOP();

    if (qspi_error || !qspi_tx_complete)
      return HAL_ERROR;

    /* Wait for flash to finish programming */
    if (QSPI_AutoPoll_IT() != HAL_OK)
      return HAL_ERROR;

    buffer += write_size;
    current_addr += write_size;
    remaining -= write_size;
  }

  return HAL_OK;
}

/* ============================================================================
   PUBLIC API - READ OPERATIONS
   ========================================================================= */

uint8_t QSPI_Quad_Read(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_4_LINES;
  sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
  sCommand.NbData = size;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  if (HAL_QSPI_Receive(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/* ============================================================================
   HELPER FUNCTIONS - Status Polling
   ========================================================================= */

uint8_t QSPI_AutoPollingMemReady(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = READ_STATUS_REG_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match = 0x00;
  sConfig.Mask = 0x01;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig, HAL_MAX_DELAY) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

uint8_t QSPI_Fast_AutoPolling(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = READ_STATUS_REG_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match = 0x00;
  sConfig.Mask = 0x01;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig, 1000) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

uint8_t QSPI_AutoPoll_IT(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = READ_STATUS_REG_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match = 0x00;
  sConfig.Mask = 0x01;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  qspi_tx_complete = 0;
  if (HAL_QSPI_AutoPolling_IT(&hqspi1, &sCommand, &sConfig) != HAL_OK)
    return HAL_ERROR;

  /* Wait with timeout */
  uint32_t timeout = HAL_GetTick() + 1000;
  while (!qspi_tx_complete && HAL_GetTick() < timeout)
    __NOP();

  return qspi_tx_complete ? HAL_OK : HAL_TIMEOUT;
}

/* ============================================================================
   PRIVATE HELPER FUNCTIONS
   ========================================================================= */

static uint8_t QSPI_WriteEnable(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  /* Send Write Enable command */
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = WRITE_ENABLE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  /* Poll for write enable bit */
  sConfig.Match = 0x02;
  sConfig.Mask = 0x02;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction = READ_STATUS_REG_CMD;
  sCommand.DataMode = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/* ============================================================================
   HAL CALLBACKS
   ========================================================================= */

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  qspi_tx_complete = 1;
}

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
  qspi_error = 1;
}

void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
  qspi_tx_complete = 1;
}

/* USER CODE END 1 */
