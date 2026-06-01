/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fdcan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if DEBUG_CAN_USE_FDCAN1
#define DEBUG_CAN_HANDLE hfdcan1
#define DEBUG_CAN_INIT() MX_FDCAN1_Init()
#define DEBUG_CAN_ACTIVE_INSTANCE 1u
#else
#define DEBUG_CAN_HANDLE hfdcan2
#define DEBUG_CAN_INIT() MX_FDCAN2_Init()
#define DEBUG_CAN_ACTIVE_INSTANCE 2u
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_can_sniffer_started;
volatile uint32_t g_can_active_instance;
volatile uint32_t g_can_sniffer_start_error;
volatile uint32_t g_can_total_messages;
volatile uint32_t g_can_std_messages;
volatile uint32_t g_can_ext_messages;
volatile uint32_t g_can_rtr_messages;
volatile uint32_t g_can_messages_per_second;
volatile uint32_t g_can_last_window_count;
volatile uint32_t g_can_last_window_ms;
volatile uint32_t g_can_last_id;
volatile uint32_t g_can_last_std_id;
volatile uint32_t g_can_last_ext_id;
volatile uint32_t g_can_last_dlc;
volatile uint32_t g_can_last_payload0;
volatile uint32_t g_can_last_payload1;
volatile uint32_t g_can_last_rx_tick;
volatile uint32_t g_can_last_delta_ms;
volatile uint32_t g_can_min_delta_ms = 0xFFFFFFFFu;
volatile uint32_t g_can_max_delta_ms;
volatile uint32_t g_can_last_odrive_node;
volatile uint32_t g_can_last_odrive_cmd;
volatile uint32_t g_can_rx_fifo0_peak;
volatile uint32_t g_can_fdcan_psr;
volatile uint32_t g_can_fdcan_ecr;
volatile uint32_t g_can_fdcan_rxf0s;
volatile uint32_t g_can_fdcan_cccr;
volatile uint32_t g_can_id_count[2048];
volatile uint32_t g_can_odrive_node_count[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t debug_can_dlc_to_bytes(uint32_t dlc)
{
  if (dlc <= FDCAN_DLC_BYTES_8) {
    return (uint8_t)dlc;
  }

  switch (dlc) {
    case FDCAN_DLC_BYTES_12: return 12u;
    case FDCAN_DLC_BYTES_16: return 16u;
    case FDCAN_DLC_BYTES_20: return 20u;
    case FDCAN_DLC_BYTES_24: return 24u;
    case FDCAN_DLC_BYTES_32: return 32u;
    case FDCAN_DLC_BYTES_48: return 48u;
    case FDCAN_DLC_BYTES_64: return 64u;
    default: return 0u;
  }
}

static void debug_can_update_status(void)
{
  g_can_fdcan_psr = READ_REG(DEBUG_CAN_HANDLE.Instance->PSR);
  g_can_fdcan_ecr = READ_REG(DEBUG_CAN_HANDLE.Instance->ECR);
  g_can_fdcan_rxf0s = READ_REG(DEBUG_CAN_HANDLE.Instance->RXF0S);
  g_can_fdcan_cccr = READ_REG(DEBUG_CAN_HANDLE.Instance->CCCR);

  const uint32_t fifo0 = HAL_FDCAN_GetRxFifoFillLevel(&DEBUG_CAN_HANDLE, FDCAN_RX_FIFO0);
  if (fifo0 > g_can_rx_fifo0_peak) {
    g_can_rx_fifo0_peak = fifo0;
  }
}

static void debug_can_process_rx(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];

  while (HAL_FDCAN_GetRxFifoFillLevel(&DEBUG_CAN_HANDLE, FDCAN_RX_FIFO0) > 0u) {
    if (HAL_FDCAN_GetRxMessage(&DEBUG_CAN_HANDLE, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
      break;
    }

    const uint32_t now = HAL_GetTick();
    const bool is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
    const bool is_rtr = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);
    const uint32_t id = is_ext ? (rx_header.Identifier & 0x1FFFFFFFu)
                               : (rx_header.Identifier & 0x7FFu);

    if (g_can_last_rx_tick != 0u) {
      const uint32_t delta = now - g_can_last_rx_tick;
      g_can_last_delta_ms = delta;
      if (delta < g_can_min_delta_ms) {
        g_can_min_delta_ms = delta;
      }
      if (delta > g_can_max_delta_ms) {
        g_can_max_delta_ms = delta;
      }
    }
    g_can_last_rx_tick = now;

    g_can_total_messages++;
    g_can_last_id = id;
    g_can_last_dlc = debug_can_dlc_to_bytes(rx_header.DataLength);
    g_can_last_payload0 = ((uint32_t)rx_data[0]) |
                          ((uint32_t)rx_data[1] << 8) |
                          ((uint32_t)rx_data[2] << 16) |
                          ((uint32_t)rx_data[3] << 24);
    g_can_last_payload1 = ((uint32_t)rx_data[4]) |
                          ((uint32_t)rx_data[5] << 8) |
                          ((uint32_t)rx_data[6] << 16) |
                          ((uint32_t)rx_data[7] << 24);

    if (is_rtr) {
      g_can_rtr_messages++;
    }

    if (is_ext) {
      g_can_ext_messages++;
      g_can_last_ext_id = id;
    } else {
      g_can_std_messages++;
      g_can_last_std_id = id;
      g_can_id_count[id]++;

      /* ODrive CAN Simple: standard ID = (node_id << 5) | command_id. */
      const uint32_t node = (id >> 5) & 0x3Fu;
      const uint32_t cmd = id & 0x1Fu;
      g_can_last_odrive_node = node;
      g_can_last_odrive_cmd = cmd;
      g_can_odrive_node_count[node]++;
    }
  }
}

static void debug_can_update_frequency(void)
{
  static uint32_t last_tick;
  static uint32_t last_total;

  const uint32_t now = HAL_GetTick();
  const uint32_t elapsed = now - last_tick;
  if (elapsed >= 1000u) {
    const uint32_t total = g_can_total_messages;
    const uint32_t count = total - last_total;

    g_can_last_window_ms = elapsed;
    g_can_last_window_count = count;
    g_can_messages_per_second = (count * 1000u) / elapsed;

    last_total = total;
    last_tick = now;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  DEBUG_CAN_INIT();
  /* USER CODE BEGIN 2 */
  g_can_active_instance = DEBUG_CAN_ACTIVE_INSTANCE;
  if (HAL_FDCAN_Start(&DEBUG_CAN_HANDLE) == HAL_OK) {
    g_can_sniffer_started = 1u;
  } else {
    g_can_sniffer_start_error = HAL_FDCAN_GetError(&DEBUG_CAN_HANDLE);
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    debug_can_process_rx();
    debug_can_update_status();
    debug_can_update_frequency();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 120;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 12;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
