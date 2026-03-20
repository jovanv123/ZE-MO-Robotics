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
#include "PID.h"
#include "board_config.h"
#include "motors.h"
#include "SpeedProfile.h"
#include "encoders.h"
#include "sequences.h"
#include "odometry.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float des_acc = 200;
float max_vel = 400;

volatile float ref_fi;
volatile float spin_target      = 0.0f;
volatile float spin_start_angle = 0.0f;
volatile float remaining;
volatile float cx, cy, cfi, c_speedr, c_speedl, c_speedr_old, c_speedl_old, c_speedl_new, c_speedr_new;
float global_goal_x = 0, global_goal_y = 0;
uint8_t  movement_phase;
volatile int8_t  dir;
volatile float   v_ref2;

int sys_t            = 0;
int sys_tax          = 0;
int designated_time  = 0;

bool debug_sensor = false;

int sensor_filter_counter   = 0;
const int FILTER_THRESHOLD  = 10;

float start_x1, start_y1;
volatile bool ax_moving = false;

extern volatile float current_position, current_position_back;
extern volatile int   step_counter, step_counter_back;
extern volatile int   pwm_active, pwm_active_back;
extern float          target_pos, target_pos_back;
extern int            nmbr_of_steps, nmbr_of_steps_back;
extern volatile bool  stepper_moving, stepper_back_moving;
int state = 0;
int debugaa, debuga;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t ss1, ss2, ss3, ss4;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  PWM_Init();
  HAL_TIM_Base_Start_IT(&htim7);
  encoder_init();
  odometry_init(81.54, 81.54, 422.0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  set_AX_WheelMode(RIGHT_LEADSCREW_AX, 1);
  HAL_Delay(500);
  set_AX_WheelMode(LEFT_LEADSCREW_AX, 1);
  HAL_Delay(500);
  state= 100;
//  leadscrew_closed(3500);
//  HAL_Delay(10);
  pushers_off();
//  storage_off();
//  HAL_Delay(1000);
  leadscrew_opened(2500);
  while (1)  {
	  bool motors_idle = !ax_moving && !stepper_back_moving && !stepper_moving;

	  switch(state)
	  {
	  case 0:
		  if(motors_idle)
		  {
			  leadscrew_opened(3500);

//			  move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OFF, RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
			  move_step_motors(50.0);
//			  HAL_Delay(50);
//			  leadscrew_opened(3500);
	//		  move_step_motors(90.0);
	//		  HAL_Delay(500);
	//		  move_step_back(90.0);
//			  steppers_up();
			  navigate(550, 0, FORWARDS, MAX_PHYSICAL_SPEED, 1000);
			  state++;
		  }
		  break;

	  case 1:
		  if(motors_idle && movement_phase == IDLE)
		  {
			  navigate(750, 0, FORWARDS, 100, 1000);
			  state++;
		  }
		  break;
	  case 2:
		  if(movement_phase==IDLE && motors_idle)
		  {
			  HAL_Delay(10);
			  pushers_on();
			  HAL_Delay(10);
			  move_step_motors(0.0);
			  HAL_Delay(500);
			  pushers_off();
			  state++;
		  }
		  break;
	  case 3:
		  if(movement_phase==IDLE && motors_idle)
		  {
			  HAL_Delay(10);
			  leadscrew_closed(2000);
			  pushers_off();
			  state++;
		  }
		  break;
	  case 4:
		  if(motors_idle)
		  {
			  pushers_off();
			  HAL_Delay(200);
			  navigate(0, 0, BACKWARDS, MAX_PHYSICAL_SPEED, 1000);
			  steppers_up();
			  state++;
		  }
		  break;
	  }
//      move_AX_Wheels_Sync(LEFT_STORAGE_AX, -100, RIGHT_STORAGE_AX, 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
    	debug_sensor = read_sensors();
        calculate_odometry();
        cx       = get_x();
        cy       = get_y();
        cfi      = get_fi();
        c_speedl_new = get_speed_l();
        c_speedr_new = get_speed_r();

        c_speedl = (0.8f * c_speedl_new) + ((1 - 0.8f) * c_speedl_old);
        c_speedr = (0.8f * c_speedr_new) + ((1 - 0.8f) * c_speedr_old);
        if(!read_sensors())
        {
			if (movement_phase == TRANSLATION || movement_phase == ROTATION || movement_phase == IDLE) {
				ref_fi = atan2f(global_goal_y - cy, global_goal_x - cx);
				if (dir == BACKWARDS && movement_phase == ROTATION)
				{
					ref_fi -= M_PI;
				}

			} else if (movement_phase == SPIN) {
				ref_fi = spin_start_angle + spin_target;
			}

			float dx = global_goal_x - cx;
			float dy = global_goal_y - cy;
			remaining = sqrtf(dx * dx + dy * dy);

			if (sys_t % 10 == 0) {
				if (movement_phase == ROTATION) {
					v_ref2 = calculate_angular_trapezoid(50000, 250000, cfi, ref_fi, &movement_phase);
				} else if (movement_phase == TRANSLATION) {
					v_ref2 = calculate_trapezoid(des_acc, max_vel, cx, cy, global_goal_x, global_goal_y, &movement_phase);
				} else if (movement_phase == SPIN) {
					float angle_turned = get_unwrapped_fi() - spin_start_angle;
					v_ref2 = calculate_spin_trapezoid(100000, 100000, angle_turned, spin_target, &movement_phase);
				}
			}
        }
        else
        {
        	if (sys_t % 10 == 0)
        		v_ref2 = v_ref2 * 0.95;
        	reset_move_profiles();
        }

        if (ax_moving) {
        	sys_tax++;
        	if (sys_tax % designated_time == 0) {
        		move_AX_Wheels_Sync(LEFT_LEADSCREW_AX, 0, RIGHT_LEADSCREW_AX, 0);
        		move_AX_Wheels_Sync(LEFT_STORAGE_AX,   0, RIGHT_STORAGE_AX,   0);
				designated_time = 0;
				sys_tax         = 0;
				ax_moving       = false;
			}
		}

        movement_PID(v_ref2, &movement_phase, MAX_ACCEL, global_goal_x, global_goal_y, ref_fi, dir);
        c_speedr_old = c_speedr;
        c_speedl_old = c_speedl;
        sys_t = (sys_t >= 2000) ? 0 : sys_t + 1;
    }

    if (htim->Instance == TIM4) {
        if (pwm_active) {
            step_counter++;
            if (step_counter >= nmbr_of_steps) {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                pwm_active       = 0;
                current_position = target_pos;
                stepper_moving   = false;
                step_counter     = 0;
            }
        }
        if (pwm_active_back) {
            step_counter_back++;
            if (step_counter_back >= nmbr_of_steps_back) {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
                pwm_active_back       = 0;
                current_position_back = target_pos_back;
                stepper_back_moving   = false;
            }
        }
        if (pwm_active == 0 && pwm_active_back == 0)
            HAL_TIM_Base_Stop_IT(&htim4);
    }
}

void navigate(float tx, float ty, int8_t direction, float acc, float vel)
{
    reset_PID();
    reset_move_profiles();
    start_x1       = cx;
    start_y1       = cy;
    global_goal_x  = tx;
    global_goal_y  = ty;
    dir            = direction;
    des_acc        = acc;
    max_vel        = vel;
    movement_phase = ROTATION;
}

void spin_robot(float num_circles)
{
    reset_PID();
    reset_move_profiles();
    spin_start_angle = get_unwrapped_fi();
    spin_target      = num_circles * 2.0f * M_PI;
    movement_phase   = SPIN;
}

void move_AX_Wheels_SyncTime(uint8_t id1, float speed1, uint8_t id2, float speed2, int time)
{
    designated_time = time;
    sys_tax         = 0;
    ax_moving       = true;
    move_AX_Wheels_Sync(id1, speed1, id2, speed2);
}

bool read_sensors(void)
{
    if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5)) { //PG2 - PG5 su inputi za senzor
            return true;
    } else {
        return false;
    }
}

void turn_crates(void)
{
    uint8_t inputVal = (GPIOB->IDR >> 12) & 0x0F;

    switch (inputVal) {
    case 0x01: PWM_SetServo_Position(1, 180); break;
    case 0x02: break;
    case 0x04: break;
    case 0x08: break;
    default:   break;
    }
}
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2099;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 450;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR1_SLP_Pin|MOTOR2_SLP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR1_DIR_Pin|MOTOR2_DIR_Pin|MS1_1_MICROSTEP_Pin|STEP1_DIR_Pin
                          |STEPDRIVER1_EN_Pin|MS1_2_MICROSTEP_Pin|STEPDRIVER2_EN_Pin|STEP2_DIR_Pin
                          |MS2_1_MICROSTEP_Pin|MS2_2_MICROSTEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR1_SLP_Pin MOTOR2_SLP_Pin MS1_1_MICROSTEP_Pin STEP1_DIR_Pin
                           STEPDRIVER1_EN_Pin MS1_2_MICROSTEP_Pin STEPDRIVER2_EN_Pin STEP2_DIR_Pin
                           MS2_1_MICROSTEP_Pin MS2_2_MICROSTEP_Pin */
  GPIO_InitStruct.Pin = MOTOR1_SLP_Pin|MOTOR2_SLP_Pin|MS1_1_MICROSTEP_Pin|STEP1_DIR_Pin
                          |STEPDRIVER1_EN_Pin|MS1_2_MICROSTEP_Pin|STEPDRIVER2_EN_Pin|STEP2_DIR_Pin
                          |MS2_1_MICROSTEP_Pin|MS2_2_MICROSTEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR1_DIR_Pin MOTOR2_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR1_DIR_Pin|MOTOR2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG4
                           PG5 PG6 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG11 PG12
                           PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
     ex: printf("Wrong parameters value: file %s on line %d\rogic: Think of this as a "Disconnect" pin. When you set it HIGH, you are disconnecting the motor coils from the power bridge.\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
