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
#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm45686.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ODRIVE_CAN_NODE_ID              0u
#define ODRIVE_CAN_CMD_ID_BITS          5u
#define ODRIVE_CMD_SET_AXIS_REQUESTED_STATE 0x07u
#define ODRIVE_CMD_SET_CONTROLLER_MODES     0x0Bu
#define ODRIVE_CMD_SET_INPUT_VEL            0x0Du
#define ODRIVE_CMD_CLEAR_ERRORS             0x18u

#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define ODRIVE_CONTROL_MODE_VELOCITY         2
#define ODRIVE_INPUT_MODE_PASSTHROUGH        1
#define ODRIVE_TARGET_VELOCITY_TURNS_S       0.2f
#define UART_TEST_PERIOD_MS                  1000u
#define UART_RX_STRING_SIZE                  128u
#define UART_RX_MAX_BYTES_PER_POLL           64u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static icm45686_t g_icm45686;
static uint8_t g_uart4_rx_byte = 0;
static uint8_t g_uart5_rx_byte = 0;

volatile int g_icm45686_init_status = -1;
volatile int g_icm45686_read_status = -1;
volatile uint32_t g_icm45686_sample_count = 0;
volatile uint32_t g_icm45686_error_count = 0;
volatile float g_icm45686_accel_mps2[3];
volatile float g_icm45686_gyro_rads[3];
volatile float g_icm45686_temp_celsius;

volatile int g_odrive_can_start_status = -1;
volatile int g_odrive_init_status = -1;
volatile int g_odrive_velocity_status = -1;
volatile uint32_t g_odrive_velocity_tx_count = 0;
volatile uint32_t g_odrive_velocity_tx_error_count = 0;
volatile float g_odrive_velocity_target_turns_s = ODRIVE_TARGET_VELOCITY_TURNS_S;
volatile uint32_t g_odrive_can_tx_free_level = 0;
volatile uint32_t g_odrive_can_last_error_code = 0;
volatile uint32_t g_odrive_can_bus_off = 0;
volatile uint32_t g_odrive_can_error_passive = 0;
volatile uint32_t g_odrive_can_warning = 0;
volatile uint32_t g_odrive_can_tx_error_count = 0;
volatile uint32_t g_odrive_can_rx_error_count = 0;
volatile int g_odrive1_can_start_status = -1;
volatile int g_odrive1_init_status = -1;
volatile int g_odrive1_velocity_status = -1;
volatile uint32_t g_odrive1_velocity_tx_count = 0;
volatile uint32_t g_odrive1_velocity_tx_error_count = 0;
volatile uint32_t g_odrive1_can_tx_free_level = 0;
volatile uint32_t g_odrive1_can_last_error_code = 0;
volatile uint32_t g_odrive1_can_bus_off = 0;
volatile uint32_t g_odrive1_can_error_passive = 0;
volatile uint32_t g_odrive1_can_warning = 0;
volatile uint32_t g_odrive1_can_tx_error_count = 0;
volatile uint32_t g_odrive1_can_rx_error_count = 0;
volatile int g_odrive2_can_start_status = -1;
volatile int g_odrive2_init_status = -1;
volatile int g_odrive2_velocity_status = -1;
volatile uint32_t g_odrive2_velocity_tx_count = 0;
volatile uint32_t g_odrive2_velocity_tx_error_count = 0;
volatile uint32_t g_odrive2_can_tx_free_level = 0;
volatile uint32_t g_odrive2_can_last_error_code = 0;
volatile uint32_t g_odrive2_can_bus_off = 0;
volatile uint32_t g_odrive2_can_error_passive = 0;
volatile uint32_t g_odrive2_can_warning = 0;
volatile uint32_t g_odrive2_can_tx_error_count = 0;
volatile uint32_t g_odrive2_can_rx_error_count = 0;

volatile int g_encoder1_start_status = -1;
volatile uint32_t g_encoder1_count_raw = 0;
volatile int32_t g_encoder1_delta_steps = 0;
volatile int64_t g_encoder1_total_steps = 0;
volatile uint32_t g_encoder1_z_count = 0;
volatile uint32_t g_encoder1_z_state = 0;
volatile uint32_t g_encoder1_count_at_last_z = 0;

volatile int g_encoder2_start_status = -1;
volatile uint32_t g_encoder2_count_raw = 0;
volatile int32_t g_encoder2_delta_steps = 0;
volatile int64_t g_encoder2_total_steps = 0;
volatile uint32_t g_encoder2_z_count = 0;
volatile uint32_t g_encoder2_z_state = 0;
volatile uint32_t g_encoder2_count_at_last_z = 0;

volatile int g_tim4_encoder_start_status = -1;
volatile uint32_t g_tim4_encoder_count_raw = 0;
volatile int32_t g_tim4_encoder_delta_steps = 0;
volatile int64_t g_tim4_encoder_total_steps = 0;
volatile uint32_t g_tim4_encoder_z_count = 0;
volatile uint32_t g_tim4_encoder_z_state = 0;
volatile uint32_t g_tim4_encoder_count_at_last_z = 0;

volatile int g_uart4_test_status = -1;
volatile int g_uart5_test_status = -1;
volatile uint32_t g_uart4_test_tx_count = 0;
volatile uint32_t g_uart5_test_tx_count = 0;
char g_uart4_rx_string[UART_RX_STRING_SIZE];
char g_uart5_rx_string[UART_RX_STRING_SIZE];
volatile uint32_t g_uart4_rx_length = 0;
volatile uint32_t g_uart5_rx_length = 0;
volatile uint32_t g_uart4_rx_byte_count = 0;
volatile uint32_t g_uart5_rx_byte_count = 0;
volatile uint32_t g_uart4_rx_line_count = 0;
volatile uint32_t g_uart5_rx_line_count = 0;
volatile uint32_t g_uart4_rx_overflow_count = 0;
volatile uint32_t g_uart5_rx_overflow_count = 0;
volatile int g_uart4_rx_status = -1;
volatile int g_uart5_rx_status = -1;
volatile uint32_t g_uart4_rx_hal_error = 0;
volatile uint32_t g_uart5_rx_hal_error = 0;
volatile uint32_t g_uart4_rx_isr = 0;
volatile uint32_t g_uart5_rx_isr = 0;
volatile uint32_t g_uart4_rx_overrun_count = 0;
volatile uint32_t g_uart5_rx_overrun_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void ICM45686_Init(void);
static void ICM45686_Poll(void);
static void ODrive_Init(void);
static void ODrive_Poll(void);
static void Encoders_ABZ_Init(void);
static void Encoders_ABZ_Poll(void);
static void UART_Test_Poll(void);
static void UART_Rx_Start(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ICM45686_Init(void)
{
  g_icm45686_init_status = icm45686_init_spi(&g_icm45686,
                                             &hspi3,
                                             PE6_SPI3_CS_GPIO_Port,
                                             PE6_SPI3_CS_Pin,
                                             ICM45686_ACCEL_16G,
                                             ICM45686_GYRO_2000_DPS);
}

static void ICM45686_Poll(void)
{
  imu_data_t sample;

  if (g_icm45686_init_status != 0)
  {
    ICM45686_Init();
    HAL_Delay(100);
    return;
  }

  g_icm45686_read_status = icm45686_read(&g_icm45686, &sample);
  if (g_icm45686_read_status == 0)
  {
    g_icm45686_accel_mps2[0] = sample.accel_mps2[0];
    g_icm45686_accel_mps2[1] = sample.accel_mps2[1];
    g_icm45686_accel_mps2[2] = sample.accel_mps2[2];
    g_icm45686_gyro_rads[0] = sample.gyro_rads[0];
    g_icm45686_gyro_rads[1] = sample.gyro_rads[1];
    g_icm45686_gyro_rads[2] = sample.gyro_rads[2];
    g_icm45686_temp_celsius = sample.temp_celsius;
    g_icm45686_sample_count++;
  }
  else
  {
    g_icm45686_error_count++;
  }
}

static uint32_t ODrive_StdId(uint32_t node_id, uint32_t command_id)
{
  return ((node_id & 0x3Fu) << ODRIVE_CAN_CMD_ID_BITS) | (command_id & 0x1Fu);
}

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan;
  volatile int *can_start_status;
  volatile int *init_status;
  volatile int *velocity_status;
  volatile uint32_t *velocity_tx_count;
  volatile uint32_t *velocity_tx_error_count;
  volatile uint32_t *can_tx_free_level;
  volatile uint32_t *can_last_error_code;
  volatile uint32_t *can_bus_off;
  volatile uint32_t *can_error_passive;
  volatile uint32_t *can_warning;
  volatile uint32_t *can_tx_error_count;
  volatile uint32_t *can_rx_error_count;
} odrive_bus_t;

static const odrive_bus_t g_odrive_bus1 = {
    &hfdcan1,
    &g_odrive1_can_start_status,
    &g_odrive1_init_status,
    &g_odrive1_velocity_status,
    &g_odrive1_velocity_tx_count,
    &g_odrive1_velocity_tx_error_count,
    &g_odrive1_can_tx_free_level,
    &g_odrive1_can_last_error_code,
    &g_odrive1_can_bus_off,
    &g_odrive1_can_error_passive,
    &g_odrive1_can_warning,
    &g_odrive1_can_tx_error_count,
    &g_odrive1_can_rx_error_count};

static const odrive_bus_t g_odrive_bus2 = {
    &hfdcan2,
    &g_odrive2_can_start_status,
    &g_odrive2_init_status,
    &g_odrive2_velocity_status,
    &g_odrive2_velocity_tx_count,
    &g_odrive2_velocity_tx_error_count,
    &g_odrive2_can_tx_free_level,
    &g_odrive2_can_last_error_code,
    &g_odrive2_can_bus_off,
    &g_odrive2_can_error_passive,
    &g_odrive2_can_warning,
    &g_odrive2_can_tx_error_count,
    &g_odrive2_can_rx_error_count};

static void ODrive_UpdateCanDiagnostics(const odrive_bus_t *bus)
{
  FDCAN_ProtocolStatusTypeDef protocol_status;
  FDCAN_ErrorCountersTypeDef error_counters;

  *bus->can_tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan);
  if (HAL_FDCAN_GetProtocolStatus(bus->hfdcan, &protocol_status) == HAL_OK)
  {
    *bus->can_last_error_code = protocol_status.LastErrorCode;
    *bus->can_bus_off = protocol_status.BusOff;
    *bus->can_error_passive = protocol_status.ErrorPassive;
    *bus->can_warning = protocol_status.Warning;
  }

  if (HAL_FDCAN_GetErrorCounters(bus->hfdcan, &error_counters) == HAL_OK)
  {
    *bus->can_tx_error_count = error_counters.TxErrorCnt;
    *bus->can_rx_error_count = error_counters.RxErrorCnt;
  }
}

static int ODrive_Tx(const odrive_bus_t *bus,
                     uint32_t command_id,
                     const uint8_t *data,
                     uint32_t len,
                     uint32_t timeout_ms)
{
  FDCAN_TxHeaderTypeDef header = {0};
  const uint32_t start = HAL_GetTick();

  header.Identifier = ODrive_StdId(ODRIVE_CAN_NODE_ID, command_id);
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  if (len > 8u)
  {
    return -1;
  }

  while (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) == 0u)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      ODrive_UpdateCanDiagnostics(bus);
      return -2;
    }
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &header, (uint8_t *)data) != HAL_OK)
  {
    ODrive_UpdateCanDiagnostics(bus);
    return -3;
  }

  while (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) < bus->hfdcan->Init.TxFifoQueueElmtsNbr)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      ODrive_UpdateCanDiagnostics(bus);
      return -4;
    }
  }

  ODrive_UpdateCanDiagnostics(bus);
  return 0;
}

static int ODrive_TxU32(const odrive_bus_t *bus, uint32_t command_id, uint32_t value)
{
  uint8_t data[8] = {0};
  memcpy(data, &value, sizeof(value));
  return ODrive_Tx(bus, command_id, data, sizeof(data), 100u);
}

static int ODrive_TxControllerModes(const odrive_bus_t *bus)
{
  uint8_t data[8] = {0};
  int32_t control_mode = ODRIVE_CONTROL_MODE_VELOCITY;
  int32_t input_mode = ODRIVE_INPUT_MODE_PASSTHROUGH;

  memcpy(data, &control_mode, sizeof(control_mode));
  memcpy(data + 4, &input_mode, sizeof(input_mode));
  return ODrive_Tx(bus, ODRIVE_CMD_SET_CONTROLLER_MODES, data, sizeof(data), 100u);
}

static int ODrive_TxInputVelocity(const odrive_bus_t *bus, float velocity_turns_s, float torque_ff_nm)
{
  uint8_t data[8] = {0};

  memcpy(data, &velocity_turns_s, sizeof(velocity_turns_s));
  memcpy(data + 4, &torque_ff_nm, sizeof(torque_ff_nm));
  return ODrive_Tx(bus, ODRIVE_CMD_SET_INPUT_VEL, data, sizeof(data), 100u);
}

static void ODrive_InitBus(const odrive_bus_t *bus)
{
  uint8_t zeros[8] = {0};

  *bus->can_start_status = (HAL_FDCAN_Start(bus->hfdcan) == HAL_OK) ? 0 : -1;
  if (*bus->can_start_status != 0)
  {
    *bus->init_status = -1;
    return;
  }

  *bus->init_status = ODrive_Tx(bus, ODRIVE_CMD_CLEAR_ERRORS, zeros, sizeof(zeros), 100u);
  if (*bus->init_status != 0)
  {
    return;
  }

  HAL_Delay(50);
  *bus->init_status = ODrive_TxControllerModes(bus);
  if (*bus->init_status != 0)
  {
    return;
  }

  HAL_Delay(50);
  *bus->init_status = ODrive_TxU32(bus,
                                   ODRIVE_CMD_SET_AXIS_REQUESTED_STATE,
                                   ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
  if (*bus->init_status != 0)
  {
    return;
  }

  HAL_Delay(100);
  *bus->init_status = ODrive_TxInputVelocity(bus, g_odrive_velocity_target_turns_s, 0.0f);
}

static void ODrive_PollBus(const odrive_bus_t *bus)
{
  *bus->velocity_status = ODrive_TxInputVelocity(bus, g_odrive_velocity_target_turns_s, 0.0f);
  if (*bus->velocity_status == 0)
  {
    (*bus->velocity_tx_count)++;
  }
  else
  {
    (*bus->velocity_tx_error_count)++;
  }
}

static int ODrive_CombineStatus(int status1, int status2)
{
  if (status1 == 0 && status2 == 0)
  {
    return 0;
  }
  return (status1 != 0) ? status1 : status2;
}

static void ODrive_UpdateLegacyDebug(void)
{
  g_odrive_can_start_status = ODrive_CombineStatus(g_odrive1_can_start_status,
                                                   g_odrive2_can_start_status);
  g_odrive_init_status = ODrive_CombineStatus(g_odrive1_init_status,
                                              g_odrive2_init_status);
  g_odrive_velocity_status = ODrive_CombineStatus(g_odrive1_velocity_status,
                                                  g_odrive2_velocity_status);
  g_odrive_velocity_tx_count = g_odrive1_velocity_tx_count + g_odrive2_velocity_tx_count;
  g_odrive_velocity_tx_error_count = g_odrive1_velocity_tx_error_count + g_odrive2_velocity_tx_error_count;
  g_odrive_can_tx_free_level = g_odrive1_can_tx_free_level;
  g_odrive_can_last_error_code = g_odrive1_can_last_error_code;
  g_odrive_can_bus_off = g_odrive1_can_bus_off;
  g_odrive_can_error_passive = g_odrive1_can_error_passive;
  g_odrive_can_warning = g_odrive1_can_warning;
  g_odrive_can_tx_error_count = g_odrive1_can_tx_error_count;
  g_odrive_can_rx_error_count = g_odrive1_can_rx_error_count;
}

static void ODrive_Init(void)
{
  ODrive_InitBus(&g_odrive_bus1);
  ODrive_InitBus(&g_odrive_bus2);
  ODrive_UpdateLegacyDebug();
}

static void ODrive_Poll(void)
{
  if (g_odrive1_init_status != 0 || g_odrive2_init_status != 0)
  {
    ODrive_Init();
    HAL_Delay(250);
    return;
  }

  ODrive_PollBus(&g_odrive_bus1);
  ODrive_PollBus(&g_odrive_bus2);
  ODrive_UpdateLegacyDebug();
}

static void Encoder_ABZ_ResetDebug(volatile int *start_status,
                                   volatile uint32_t *count_raw,
                                   volatile int32_t *delta_steps,
                                   volatile int64_t *total_steps,
                                   volatile uint32_t *z_count,
                                   volatile uint32_t *z_state,
                                   volatile uint32_t *count_at_last_z)
{
  *start_status = -1;
  *count_raw = 0u;
  *delta_steps = 0;
  *total_steps = 0;
  *z_count = 0u;
  *z_state = 0u;
  *count_at_last_z = 0u;
}

static void Encoders_ABZ_Init(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0u);
  __HAL_TIM_SET_COUNTER(&htim4, 0u);

  Encoder_ABZ_ResetDebug(&g_encoder1_start_status,
                         &g_encoder1_count_raw,
                         &g_encoder1_delta_steps,
                         &g_encoder1_total_steps,
                         &g_encoder1_z_count,
                         &g_encoder1_z_state,
                         &g_encoder1_count_at_last_z);

  Encoder_ABZ_ResetDebug(&g_encoder2_start_status,
                         &g_encoder2_count_raw,
                         &g_encoder2_delta_steps,
                         &g_encoder2_total_steps,
                         &g_encoder2_z_count,
                         &g_encoder2_z_state,
                         &g_encoder2_count_at_last_z);

  Encoder_ABZ_ResetDebug(&g_tim4_encoder_start_status,
                         &g_tim4_encoder_count_raw,
                         &g_tim4_encoder_delta_steps,
                         &g_tim4_encoder_total_steps,
                         &g_tim4_encoder_z_count,
                         &g_tim4_encoder_z_state,
                         &g_tim4_encoder_count_at_last_z);

  g_encoder1_start_status =
      (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) == HAL_OK) ? 0 : -1;
  g_encoder2_start_status =
      (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK) ? 0 : -1;

  g_tim4_encoder_start_status = g_encoder2_start_status;
}

static void Encoder_ABZ_Poll(TIM_HandleTypeDef *htim,
                             GPIO_TypeDef *z_gpio_port,
                             uint16_t z_gpio_pin,
                             uint32_t counter_mask,
                             volatile uint32_t *last_count,
                             volatile uint32_t *last_z_state,
                             volatile uint32_t *count_raw,
                             volatile int32_t *delta_steps,
                             volatile int64_t *total_steps,
                             volatile uint32_t *z_count,
                             volatile uint32_t *z_state,
                             volatile uint32_t *count_at_last_z)
{
  const GPIO_PinState z_pin_state = HAL_GPIO_ReadPin(z_gpio_port, z_gpio_pin);
  const uint32_t now = __HAL_TIM_GET_COUNTER(htim) & counter_mask;
  int32_t delta;

  if (counter_mask == 0xFFFFu)
  {
    delta = (int32_t)(int16_t)(uint16_t)(now - *last_count);
  }
  else
  {
    delta = (int32_t)(now - *last_count);
  }

  *z_state = (z_pin_state == GPIO_PIN_SET) ? 1u : 0u;

  if (z_pin_state == GPIO_PIN_SET && *last_z_state == 0u)
  {
    *total_steps += (int64_t)delta;
    *delta_steps = delta;
    *count_at_last_z = now;
    (*z_count)++;

    __HAL_TIM_SET_COUNTER(htim, 0u);
    *last_count = 0u;
    *count_raw = 0u;
    *last_z_state = 1u;
    return;
  }

  *last_count = now;
  *last_z_state = (z_pin_state == GPIO_PIN_SET) ? 1u : 0u;
  *count_raw = now;
  *delta_steps = delta;
  *total_steps += (int64_t)delta;
}

static void Encoders_ABZ_Poll(void)
{
  static volatile uint32_t s_encoder1_last_count = 0u;
  static volatile uint32_t s_encoder2_last_count = 0u;
  static volatile uint32_t s_encoder1_last_z_state = 0u;
  static volatile uint32_t s_encoder2_last_z_state = 0u;

  Encoder_ABZ_Poll(&htim2,
                   TIM2_Z_GPIO_Port,
                   TIM2_Z_Pin,
                   0xFFFFFFFFu,
                   &s_encoder1_last_count,
                   &s_encoder1_last_z_state,
                   &g_encoder1_count_raw,
                   &g_encoder1_delta_steps,
                   &g_encoder1_total_steps,
                   &g_encoder1_z_count,
                   &g_encoder1_z_state,
                   &g_encoder1_count_at_last_z);

  Encoder_ABZ_Poll(&htim4,
                   TIM4_Z_GPIO_Port,
                   TIM4_Z_Pin,
                   0xFFFFu,
                   &s_encoder2_last_count,
                   &s_encoder2_last_z_state,
                   &g_encoder2_count_raw,
                   &g_encoder2_delta_steps,
                   &g_encoder2_total_steps,
                   &g_encoder2_z_count,
                   &g_encoder2_z_state,
                   &g_encoder2_count_at_last_z);

  g_tim4_encoder_start_status = g_encoder2_start_status;
  g_tim4_encoder_count_raw = g_encoder2_count_raw;
  g_tim4_encoder_delta_steps = g_encoder2_delta_steps;
  g_tim4_encoder_total_steps = g_encoder2_total_steps;
  g_tim4_encoder_z_count = g_encoder2_z_count;
  g_tim4_encoder_z_state = g_encoder2_z_state;
  g_tim4_encoder_count_at_last_z = g_encoder2_count_at_last_z;
}

static void UART_Test_Poll(void)
{
  static uint32_t s_last_tx_ms = 0u;
  static const uint8_t msg[] = "UART OK\r\n";
  const uint32_t now = HAL_GetTick();

  if ((now - s_last_tx_ms) < UART_TEST_PERIOD_MS)
  {
    return;
  }
  s_last_tx_ms = now;

  g_uart4_test_status = (HAL_UART_Transmit(&huart4, (uint8_t *)msg,
                                           (uint16_t)(sizeof(msg) - 1u), 100u) == HAL_OK) ? 0 : -1;
  if (g_uart4_test_status == 0)
  {
    g_uart4_test_tx_count++;
  }

  g_uart5_test_status = (HAL_UART_Transmit(&huart5, (uint8_t *)msg,
                                           (uint16_t)(sizeof(msg) - 1u), 100u) == HAL_OK) ? 0 : -1;
  if (g_uart5_test_status == 0)
  {
    g_uart5_test_tx_count++;
  }
}

static void UART_RxAppendByte(char *buffer,
                              volatile uint32_t *length,
                              volatile uint32_t *line_count,
                              volatile uint32_t *overflow_count,
                              uint8_t byte)
{
  uint32_t len = *length;

  if (byte == '\r' || byte == '\n')
  {
    if (len == 0u)
    {
      return;
    }
    buffer[len] = '\0';
    *length = 0u;
    (*line_count)++;
    return;
  }

  if (len >= (UART_RX_STRING_SIZE - 1u))
  {
    buffer[UART_RX_STRING_SIZE - 1u] = '\0';
    *length = 0u;
    (*overflow_count)++;
    return;
  }

  buffer[len++] = (char)byte;
  buffer[len] = '\0';
  *length = len;
}

static void UART_Rx_Start(void)
{
  g_uart4_rx_status = (HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1u) == HAL_OK) ? 0 : -1;
  g_uart5_rx_status = (HAL_UART_Receive_IT(&huart5, &g_uart5_rx_byte, 1u) == HAL_OK) ? 0 : -1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    UART_RxAppendByte(g_uart4_rx_string,
                      &g_uart4_rx_length,
                      &g_uart4_rx_line_count,
                      &g_uart4_rx_overflow_count,
                      g_uart4_rx_byte);
    g_uart4_rx_byte_count++;
    g_uart4_rx_status = (HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1u) == HAL_OK) ? 0 : -1;
  }
  else if (huart->Instance == UART5)
  {
    UART_RxAppendByte(g_uart5_rx_string,
                      &g_uart5_rx_length,
                      &g_uart5_rx_line_count,
                      &g_uart5_rx_overflow_count,
                      g_uart5_rx_byte);
    g_uart5_rx_byte_count++;
    g_uart5_rx_status = (HAL_UART_Receive_IT(&huart5, &g_uart5_rx_byte, 1u) == HAL_OK) ? 0 : -1;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    g_uart4_rx_hal_error = huart->ErrorCode;
    g_uart4_rx_isr = huart->Instance->ISR;
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0u)
    {
      g_uart4_rx_overrun_count++;
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    g_uart4_rx_status = (HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1u) == HAL_OK) ? 0 : -1;
  }
  else if (huart->Instance == UART5)
  {
    g_uart5_rx_hal_error = huart->ErrorCode;
    g_uart5_rx_isr = huart->Instance->ISR;
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0u)
    {
      g_uart5_rx_overrun_count++;
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    g_uart5_rx_status = (HAL_UART_Receive_IT(&huart5, &g_uart5_rx_byte, 1u) == HAL_OK) ? 0 : -1;
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
 //HAL_Delay(100);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI3_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  Encoders_ABZ_Init();
  ICM45686_Init();
  HAL_Delay(100);
  ODrive_Init();
  //UART_Rx_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Encoders_ABZ_Poll();
    ICM45686_Poll();
    ODrive_Poll();
    UART_Test_Poll();
    HAL_Delay(2);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 6;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
