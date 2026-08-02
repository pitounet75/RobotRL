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
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "app_halt.h"
#include "app_ctrl_params.h"
#include "app_freertos_diag.h"
#include "app_telemetry.h"
#include "icm45686.h"
#include "imu_async.h"
#include "imu_spi_async.h"
#include "odrive_can_hal.h"
#include "odrive_torque_mode_startup.h"
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
volatile uint32_t g_halt_magic;
volatile uint32_t g_halt_step;
volatile uint32_t g_halt_startup_error;
volatile uint32_t g_halt_startup_fail_line;
volatile uint32_t g_halt_fdcan_start_fail_step;
volatile uint32_t g_halt_rx_std_frames;
volatile uint32_t g_halt_rx_fifo0_peak;
volatile uint32_t g_halt_last_rx_std_id;
volatile uint32_t g_halt_fdcan_psr;
volatile uint32_t g_halt_fdcan_ecr;
volatile uint32_t g_halt_fdcan_cccr;
volatile uint32_t g_halt_fdcan_rxf0s;
volatile uint32_t g_halt_tx_fail_op;
volatile uint32_t g_halt_tx_fifo_free;
volatile uint32_t g_app_halt_step = APP_HALT_NONE;
static icm45686_t s_main_icm45686_probe;
volatile int32_t g_main_icm45686_probe_status;
volatile uint8_t g_main_icm45686_probe_who_am_i;
volatile uint32_t g_tim1_up_isr_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void app_halt_record(uint32_t step)
{
  g_halt_magic = 0u;
  g_halt_step = step;
  g_halt_startup_error = g_odrive_startup_last_error;
  g_halt_startup_fail_line = g_odrive_startup_fail_line;
  g_halt_fdcan_start_fail_step = g_fdcan_start_fail_step;
  g_halt_rx_std_frames = g_odrive_startup_rx_std_frames;
  g_halt_rx_fifo0_peak = g_odrive_startup_rx_fifo0_peak;
  g_halt_last_rx_std_id = g_odrive_startup_last_rx_std_id;
  g_halt_fdcan_psr = g_odrive_startup_fdcan_psr;
  g_halt_fdcan_ecr = g_odrive_startup_fdcan_ecr;
  g_halt_fdcan_cccr = g_odrive_startup_fdcan_cccr;
  g_halt_fdcan_rxf0s = g_odrive_startup_fdcan_rxf0s;
  g_halt_tx_fail_op = g_odrive_startup_tx_fail_op;
  g_halt_tx_fifo_free = g_odrive_startup_tx_fifo_free;
  g_app_halt_step = step;
  __DSB();
  g_halt_magic = APP_HALT_MAGIC;
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
  app_freertos_diag_capture_cpuid();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  g_main_icm45686_probe_status = icm45686_init_spi(&s_main_icm45686_probe,
                                                   &hspi3,
                                                   PE6_SPI3_CS_GPIO_Port,
                                                   PE6_SPI3_CS_Pin,
                                                   ICM45686_ACCEL_16G,
                                                   ICM45686_GYRO_2000_DPS);
  g_main_icm45686_probe_who_am_i = icm45686_last_who_am_i;
  if (g_main_icm45686_probe_status == 0) {
    imu_async_note_hw_probed_ok(g_main_icm45686_probe_who_am_i);
  }
#if defined(DEBUG_DESK_NO_ODRIVE) || (APP_TELEMETRY_BENCH_MODE != 0)
  /* Bench / telemetry test: skip ODrive boot (see APP_TELEMETRY_BENCH_MODE). */
  volatile uint8_t g_app_odrive_boot_skipped = 1u;
  (void)g_app_odrive_boot_skipped;
#else
  volatile uint8_t g_app_odrive_boot_skipped = 0u;
  (void)g_app_odrive_boot_skipped;
  if (!odrive_can_fdcan_start(&ODRIVE_CAN_LEFT_HANDLE) ||
      !odrive_can_fdcan_start(&ODRIVE_CAN_RIGHT_HANDLE)) {
    app_halt_record(APP_HALT_FDCAN_START);
    Error_Handler();
  }

  /* RX IRQ stays off until odrive_torque_mode_startup() finishes polling heartbeats. */
  if (!odrive_torque_mode_startup(&ODRIVE_CAN_LEFT_HANDLE) ||
      !odrive_torque_mode_startup(&ODRIVE_CAN_RIGHT_HANDLE)) {
    app_halt_record(APP_HALT_ODRIVE_STARTUP);
    Error_Handler();
  }
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  if (g_halt_magic != APP_HALT_MAGIC) {
    uint32_t step = g_app_halt_step;
    if (step == APP_HALT_NONE) {
      step = APP_HALT_UNKNOWN;
    }
    app_halt_record(step);
  }
  __disable_irq();
  while (1) {
    __NOP();
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
