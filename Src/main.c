/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - With GPS Integration
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
#include "debug_utils.h"
#include "icm42688.h"
#include "mmc5983ma.h"
#include "h3lis331dl.h"
#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include "sensor_manager.h"
#include "mahony_filter.h"
#include "gps_module.h"
#include "ble_module.h"
#include "data_logger.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
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
/* BLE Test Variables */
static uint32_t ble_test_counter = 0;
static uint32_t last_ble_send_time = 0;
static bool ble_initialized = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* BLE Command Callback */
static void BLE_CommandCallback(const char* command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief BLE command callback - handles custom commands
 * @param command The received command string
 * @note Built-in commands (start, stop, status, help, erase) are handled by BLE module
 * @note This callback is for additional custom commands
 */
static void BLE_CommandCallback(const char* command) {
    char response[128];

    // Example: custom "test" command
    if (strcmp(command, "test") == 0) {
        snprintf(response, sizeof(response), "BLE Test OK! Counter=%lu\r\n", ble_test_counter);
        BLE_SendResponse(response);
    }
    // Example: get system info
    else if (strcmp(command, "info") == 0) {
        snprintf(response, sizeof(response),
                "FlightComputer v1.0\r\nBLE Module Active\r\nUptime: %lu ms\r\n",
                HAL_GetTick());
        BLE_SendResponse(response);
    }
    // Unknown command
    else {
        snprintf(response, sizeof(response), "Unknown command: %s\r\n", command);
        BLE_SendResponse(response);
    }
}

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

  // ===== BLE MODULE TEST =====
  // Initialize BLE module
  if (BLE_Init()) {
      ble_initialized = true;
      DebugPrint("BLE Module initialized successfully\r\n");

      // Register command callback for custom commands
      BLE_RegisterCommandCallback(BLE_CommandCallback);
      DebugPrint("BLE Command callback registered\r\n");

      // Enable data transmission
      BLE_SetDataTransmissionEnabled(true);
      DebugPrint("BLE Data transmission enabled\r\n");

      // Send welcome message
      BLE_SendResponse("=== FlightComputer BLE Test ===\r\n");
      BLE_SendResponse("Built-in commands: start, stop, status, help, erase\r\n");
      BLE_SendResponse("Custom commands: test, info\r\n");
      BLE_SendResponse("Ready!\r\n");
  } else {
      DebugPrint("ERROR: BLE Module initialization failed!\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ===== BLE MODULE TEST LOOP =====
    if (ble_initialized) {
        // CRITICAL: Call BLE_Update() every loop iteration
        // This handles UART reception, command processing, and timeout
        BLE_Update();

        // Send test data every 1 second
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_ble_send_time >= 1000) {
            last_ble_send_time = current_time;
            ble_test_counter++;

            // Send test telemetry data (respects data transmission enable/disable)
            char telemetry[128];
            snprintf(telemetry, sizeof(telemetry),
                    "Test Data #%lu | Time: %lu ms | Status: %s\r\n",
                    ble_test_counter,
                    current_time,
                    BLE_IsConnected() ? "Connected" : "Disconnected");

            BLE_SendData(telemetry, strlen(telemetry));

            // Optional: Get and display BLE statistics
            BLE_Statistics_t stats;
            if (BLE_GetStatistics(&stats)) {
                char stats_msg[128];
                snprintf(stats_msg, sizeof(stats_msg),
                        "Stats: RX=%lu TX=%lu Connections=%lu\r\n",
                        stats.total_bytes_received,
                        stats.total_bytes_sent,
                        stats.connection_count);
                DebugPrint(stats_msg);
            }
        }
    }

    // Small delay to prevent CPU saturation (optional)
    HAL_Delay(1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
