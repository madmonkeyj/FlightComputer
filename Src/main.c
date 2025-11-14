/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "sensor_manager.h"
#include "debug_utils.h"
#include "icm42688.h"
#include "mmc5983ma.h"
#include "sensor_manager.h"
#include "i2c_scanner.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RunHighSpeedLoop(void);
void PrintStatistics(void);
void TestDMAVerification(void);

static inline uint32_t GetMicros(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  // Wait for USB CDC to be ready
  HAL_Delay(2000);

  // Initialize sensors
  SensorManager_Config_t config = {
      .imu_odr_hz = 1000,
      .mag_odr_hz = 1000,
      .use_mag = true,
      .use_high_g = true
  };
  SensorManager_Init(&config);

  I2C_Scan();

  HAL_Delay(1000);  // Give time to read output

  DebugPrint("\r\n=== DMA Test Started ===\r\n");

  uint32_t loop_count = 0;
  uint32_t last_print_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	    SensorManager_RawData_t data;
	    uint32_t start_us = GetMicros();

	    // Read all sensors via DMA
	    SensorManager_ReadRaw(&data);

	    uint32_t end_us = GetMicros();
	    uint32_t read_time_us = end_us - start_us;

	    loop_count++;

	    // Print stats every second
	    if (HAL_GetTick() - last_print_time >= 1000) {
	        SensorManager_Status_t sensor_status;
	        SensorManager_GetStatus(&sensor_status);

	        uint32_t imu_dma_count = ICM42688_GetDMACallbackCount();
	        uint32_t mag_dma_count = MMC5983MA_GetDMACallbackCount();

	        DebugPrint("\r\n=== Stats (1 sec) ===\r\n");
	        DebugPrint("Loop rate: %.1f Hz (target: 500+)\r\n",
	                   sensor_status.actual_rate_hz);
	        DebugPrint("Read time: %lu us\r\n", read_time_us);
	        DebugPrint("DMA callbacks: IMU=%lu MAG=%lu\r\n",
	                   imu_dma_count, mag_dma_count);
	        DebugPrint("Errors: IMU=%lu MAG=%lu\r\n",
	                   sensor_status.imu_error_count,
	                   sensor_status.mag_error_count);
	        DebugPrint("Valid flags: IMU=%d MAG=%d\r\n\r\n",
	                   data.imu_valid, data.mag_valid);

	        // Accelerometer (raw + scaled)
	        float accel_mag = sqrtf(data.accel_x*data.accel_x +
	                               data.accel_y*data.accel_y +
	                               data.accel_z*data.accel_z) / 2048.0f;
	        DebugPrint("Accel [raw]: X=%6d Y=%6d Z=%6d  (%.2f g total)\r\n",
	                   data.accel_x, data.accel_y, data.accel_z, accel_mag);

	        // Gyroscope (raw + scaled to deg/s)
	        DebugPrint("Gyro  [raw]: X=%6d Y=%6d Z=%6d  (LSB)\r\n",
	                   data.gyro_x, data.gyro_y, data.gyro_z);
	        DebugPrint("      [dps]: X=%6.1f Y=%6.1f Z=%6.1f  (°/s)\r\n",
	                   data.gyro_x / 16.4f,
	                   data.gyro_y / 16.4f,
	                   data.gyro_z / 16.4f);

	        // Magnetometer (raw + scaled)
	        float mag_x_gauss = data.mag_x / 16384.0f;
	        float mag_y_gauss = data.mag_y / 16384.0f;
	        float mag_z_gauss = data.mag_z / 16384.0f;
	        float mag_total = sqrtf(mag_x_gauss*mag_x_gauss +
	                               mag_y_gauss*mag_y_gauss +
	                               mag_z_gauss*mag_z_gauss);

	        DebugPrint("Mag   [raw]: X=%6ld Y=%6ld Z=%6ld  (after null offset)\r\n",
	                   data.mag_x, data.mag_y, data.mag_z);
	        DebugPrint("    [Gauss]: X=%6.3f Y=%6.3f Z=%6.3f  (%.3f G total)\r\n",
	                   mag_x_gauss, mag_y_gauss, mag_z_gauss, mag_total);

	        last_print_time = HAL_GetTick();
	        loop_count = 0;
	    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Get microsecond timestamp (uses SysTick)
 */
static inline uint32_t GetMicros(void) {
    uint32_t m = HAL_GetTick();
    uint32_t u = SysTick->LOAD - SysTick->VAL;

    // Check for pending SysTick interrupt
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        m++;
        u = SysTick->LOAD - SysTick->VAL;
    }

    // Convert to microseconds (assuming 170MHz CPU)
    return (m * 1000) + (u * 1000 / SysTick->LOAD);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
