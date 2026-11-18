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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR          (0x68 << 1)
#define MPU9250_WHO_AM_I      0x75
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_TIMEOUT_MS    10
#define MPU9250_ACCEL_XOUT_H   0x3B
#define IMU_PERIOD_MS         100
#define UART_RX_RING_SIZE 256
#define GYRO_BIAS_X    (-93.59f)
#define GYRO_BIAS_Y    (193.61f)
#define GYRO_BIAS_Z    (-44.02f)
#define ACCEL_BIAS_X    433.2640f
#define ACCEL_BIAS_Y    369.6160f
#define ACCEL_BIAS_Z    4045.8200f

#define ACCEL_SCALE_X   0.999424f
#define ACCEL_SCALE_Y   1.000798f
#define ACCEL_SCALE_Z   0.983851f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STEERING_NEG_US          1100
#define STEERING_CENTER_US       1400
#define STEERING_POS_US          1700

#define ESC_FULL_REVERSE_US      1100
#define ESC_REVERSE_START_US     1200
#define ESC_NEUTRAL_US           1500
#define ESC_FORWARD_START_US     1600
#define ESC_FULL_FORWARD_US      1900

#define COMMAND_MAX              1000
#define UART_BUFFER_SIZE         64
#define SERIAL_WATCHDOG_MS       500
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_byte;

char rx_buffer[UART_BUFFER_SIZE];
uint8_t rx_index = 0;

uint32_t last_command_ms = 0;
uint8_t watchdog_active = 0;

int16_t steering_cmd = 0;
int16_t throttle_cmd = 0;

uint16_t steering_pwm = STEERING_CENTER_US;
uint16_t esc_pwm = ESC_NEUTRAL_US;

int8_t drive_direction = 0;
// -1 = reverse
//  0 = neutral
// +1 = forward

static uint32_t last_encoder_print_ms = 0;

static uint32_t prev_left = 0;
static uint16_t prev_right = 0;

static int64_t total_left = 0;
static int64_t total_right = 0;

static uint8_t imu_ready = 0;

static uint32_t last_imu_read_ms = 0;

static int16_t ax, ay, az;
static int16_t gx, gy, gz;

static volatile uint8_t rx_ring[UART_RX_RING_SIZE];

static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static volatile uint8_t rx_fault = 0;
static uint8_t rx_discard = 0;


static volatile uint32_t rx_count = 0;
static volatile uint32_t rx_error = 0;
static uint32_t last_debug_ms = 0;

static float gx_corrected;
static float gy_corrected;
static float gz_corrected;

static float ax_corrected;
static float ay_corrected;
static float az_corrected;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t mpu9250_read_raw(void)
{
    uint8_t data[14];

    if (HAL_I2C_Mem_Read(
            &hi2c1,
            MPU9250_ADDR,
            MPU9250_ACCEL_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,
            data,
            14,
            MPU9250_TIMEOUT_MS) != HAL_OK)
    {
        return 0;
    }

    ax = (int16_t)((data[0] << 8) | data[1]);
    ay = (int16_t)((data[2] << 8) | data[3]);
    az = (int16_t)((data[4] << 8) | data[5]);

    gx = (int16_t)((data[8] << 8) | data[9]);
    gy = (int16_t)((data[10] << 8) | data[11]);
    gz = (int16_t)((data[12] << 8) | data[13]);

    gx_corrected = (float)gx - GYRO_BIAS_X;
    gy_corrected = (float)gy - GYRO_BIAS_Y;
    gz_corrected = (float)gz - GYRO_BIAS_Z;

    ax_corrected = ((float)ax - ACCEL_BIAS_X) * ACCEL_SCALE_X;
    ay_corrected = ((float)ay - ACCEL_BIAS_Y) * ACCEL_SCALE_Y;
    az_corrected = ((float)az - ACCEL_BIAS_Z) * ACCEL_SCALE_Z;

    return 1;
}


static uint8_t mpu9250_init(void)
{
    uint8_t who_am_i = 0;

    if (HAL_I2C_Mem_Read(
            &hi2c1,
            MPU9250_ADDR,
            MPU9250_WHO_AM_I,
            I2C_MEMADD_SIZE_8BIT,
            &who_am_i,
            1,
            MPU9250_TIMEOUT_MS) != HAL_OK)
    {
        return 0;
    }

    if (who_am_i != 0x71)
    {
        return 0;
    }

    uint8_t data = 0x00;

    if (HAL_I2C_Mem_Write(
            &hi2c1,
            MPU9250_ADDR,
            MPU9250_PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            MPU9250_TIMEOUT_MS) != HAL_OK)
    {
        return 0;
    }

    return 1;
}


static int16_t clamp_command(int value)
{
    if (value > COMMAND_MAX)
        return COMMAND_MAX;

    if (value < -COMMAND_MAX)
        return -COMMAND_MAX;

    return (int16_t)value;
}


static uint16_t steering_to_pwm(int16_t command)
{
    if (command > 0)
    {
        return STEERING_CENTER_US -
            ((uint32_t)command *
            (STEERING_CENTER_US - STEERING_NEG_US)) /
            COMMAND_MAX;
    }

    if (command < 0)
    {
        return STEERING_CENTER_US +
            ((uint32_t)(-command) *
            (STEERING_POS_US - STEERING_CENTER_US)) /
            COMMAND_MAX;
    }

    return STEERING_CENTER_US;
}


static uint16_t throttle_to_pwm(int16_t command)
{
    if (command > 0)
    {
        return ESC_FORWARD_START_US +
            ((uint32_t)command *
            (ESC_FULL_FORWARD_US - ESC_FORWARD_START_US)) /
            COMMAND_MAX;
    }

    if (command < 0)
    {
        return ESC_REVERSE_START_US -
            ((uint32_t)(-command) *
            (ESC_REVERSE_START_US - ESC_FULL_REVERSE_US)) /
            COMMAND_MAX;
    }

    return ESC_NEUTRAL_US;
}


static void apply_vehicle_command(int16_t steering, int16_t throttle)
{
    steering = clamp_command(steering);
    throttle = clamp_command(throttle);

    steering_cmd = steering;
    throttle_cmd = throttle;

    steering_pwm = steering_to_pwm(steering);

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        steering_pwm
    );

    int8_t requested_direction = 0;

    if (throttle > 0)
        requested_direction = 1;
    else if (throttle < 0)
        requested_direction = -1;

    /*
     * Direct direction change:
     *
     * forward -> reverse
     * reverse -> forward
     *
     * Force an explicit neutral cycle first.
     */
    if (
        drive_direction != 0 &&
        requested_direction != 0 &&
        requested_direction != drive_direction
    )
    {
        esc_pwm = ESC_NEUTRAL_US;
        drive_direction = 0;

        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_4,
            esc_pwm
        );

        return;
    }

    esc_pwm = throttle_to_pwm(throttle);

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_4,
        esc_pwm
    );

    drive_direction = requested_direction;
}


static void send_status(void)
{
    char buffer[128];

    snprintf(
        buffer,
        sizeof(buffer),
        "OK S=%d T=%d SPWM=%u EPWM=%u\r\n",
        steering_cmd,
        throttle_cmd,
        steering_pwm,
        esc_pwm
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)buffer,
        strlen(buffer),
        HAL_MAX_DELAY
    );
}


static void process_command(char *line)
{
    int steering;
    int throttle;

    if (sscanf(line, "C,%d,%d", &steering, &throttle) == 2)
    {
        apply_vehicle_command(
            (int16_t)steering,
            (int16_t)throttle
        );

        last_command_ms = HAL_GetTick();
        watchdog_active = 0;

        send_status();
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_LPTIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  apply_vehicle_command(0, 0);

  last_command_ms = HAL_GetTick();

  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_LPTIM_Encoder_Start(&hlptim1, 65535) != HAL_OK)
  {
      Error_Handler();
  }

  prev_left = __HAL_TIM_GET_COUNTER(&htim2);
  prev_right = HAL_LPTIM_ReadCounter(&hlptim1);

  imu_ready = mpu9250_init();

  const char *msg = imu_ready
      ? "IMU OK\r\n"
      : "IMU ERROR\r\n";

  HAL_UART_Transmit(
      &huart2,
      (uint8_t *)msg,
      strlen(msg),
      20
  );

  if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
  {
      rx_fault = 1;
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GetTick() - last_encoder_print_ms >= 100)
	  {
	      last_encoder_print_ms = HAL_GetTick();

	      uint32_t left = __HAL_TIM_GET_COUNTER(&htim2);
	      uint16_t right = HAL_LPTIM_ReadCounter(&hlptim1);

	      // Confirm LPTIM counter reading
	      if (right == HAL_LPTIM_ReadCounter(&hlptim1))
	      {
	          // Compute signed differences with wraparound
	          int32_t delta_left = (int32_t)(left - prev_left);
	          int16_t delta_right = (int16_t)(right - prev_right);

	          // Accumulate counts
	          total_left += delta_left;
	          total_right += delta_right;

	          prev_left = left;
	          prev_right = right;

	          char msg[80];

	          int len = snprintf(msg, sizeof(msg),
	                             "ENC L=%ld R=%ld\r\n",
	                             (long)total_left,
	                             (long)total_right);

	          HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 10);
	      }
	  }

	  if (imu_ready &&
	      HAL_GetTick() - last_imu_read_ms >= IMU_PERIOD_MS)
	  {
	      last_imu_read_ms = HAL_GetTick();

	      if (mpu9250_read_raw())
	      {
	          char msg[100];

	          int len = snprintf(
	              msg, sizeof(msg),
	              "IMU A=%d,%d,%d G=%d,%d,%d\r\n",
	              (int)ax_corrected,
	              (int)ay_corrected,
	              (int)az_corrected,
	              (int)gx_corrected,
	              (int)gy_corrected,
	              (int)gz_corrected
	          );

	          HAL_UART_Transmit(
	              &huart2, (uint8_t *)msg, len, 10
	          );
	      }
	  }

	  while (!rx_fault && rx_tail != rx_head)
	  {
	      uint8_t byte = rx_ring[rx_tail];
	      rx_tail = (uint8_t)(rx_tail + 1);

	      if (byte == '\n' || byte == '\r')
	      {
	          if (!rx_discard && rx_index > 0 && !rx_fault)
	          {
	              rx_buffer[rx_index] = '\0';
	              process_command(rx_buffer);
	          }

	          rx_index = 0;
	          rx_discard = 0;
	      }
	      else if (!rx_discard)
	      {
	          if (rx_index < UART_BUFFER_SIZE - 1)
	          {
	              rx_buffer[rx_index++] = byte;
	          }
	          else
	          {
	              rx_index = 0;
	              rx_discard = 1;
	          }
	      }
	  }

	  if (rx_fault && !watchdog_active)
	  {
	      apply_vehicle_command(0, 0);
	      watchdog_active = 1;
	  }

	  if (
		  !watchdog_active &&
		  (HAL_GetTick() - last_command_ms > SERIAL_WATCHDOG_MS)
	  )
	  {
		  apply_vehicle_command(0, 0);

		  watchdog_active = 1;
	  }
	  if (HAL_GetTick() - last_debug_ms >= 1000)
	  {
		  last_debug_ms = HAL_GetTick();

		  char msg[100];

		  int len = snprintf(
			  msg, sizeof(msg),
			  "DEBUG RX=%lu FAULT=%u ERROR=0x%lX\r\n",
			  (unsigned long)rx_count,
			  (unsigned int)rx_fault,
			  (unsigned long)rx_error
		  );

		  HAL_UART_Transmit(
			  &huart2, (uint8_t *)msg, len, 10
		  );
	  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING_FALLING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;


    rx_count++;


    if (rx_fault)
        return;

    uint8_t next = (uint8_t)(rx_head + 1);

    if (next == rx_tail)
    {
        rx_fault = 1;
        return;
    }

    rx_ring[rx_head] = rx_byte;
    rx_head = next;

    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
    {
        rx_fault = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rx_error = HAL_UART_GetError(huart);
        rx_fault = 1;
    }
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
