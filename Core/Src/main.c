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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Inc/SpeedProfile.h"
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

UART_HandleTypeDef huart6, huart2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
DMA_HandleTypeDef hdma_usart6_rx;
/* USER CODE BEGIN PV */
float des_acc = 200;
float max_vel = 400;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
bool read_sensors();
void turn_crates();
void navigate(float tx, float ty, int8_t direction, float acc, float vel);
void spin_robot(float num_circles);
void move_AX_Wheels_SyncTime(uint8_t id1, float speed1, uint8_t id2,
		float speed2, int time);
/* USER CODE END PFP */
uint32_t freq;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile float ref_fi;
volatile float spin_target = 0.0f;
volatile float spin_start_angle = 0.0f;
volatile float remaining;
volatile float cx, cy, cfi, c_speedr, c_speedl;
float global_goal_x, global_goal_y;
float dampened_goal_x1, dampened_goal_y1;
float dampened_goal_x2, dampened_goal_y2;
float dampened_vel1, dampened_acc1;
float dampened_vel2, dampened_acc2;
volatile bool dampened = false;
uint8_t movement_phase;
uint8_t rx_dma_buffer[64];
uint8_t rx_dma_buffer2[64];
volatile int8_t dir;
volatile float v_ref2;
int sys_t = 0;
int sys_tax = 0;
int designated_time = 0;
int sensor_filter_counter = 0;
const int FILTER_THRESHOLD = 10;
float start_x1, start_y1;
volatile bool ax_moving = false;

extern volatile float current_position, current_position_back;
extern volatile int step_counter, step_counter_back;
extern volatile int pwm_active, pwm_active_back;
extern float target_pos, target_pos_back;
extern int nmbr_of_steps, nmbr_of_steps_back;
extern volatile bool stepper_moving, stepper_back_moving;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM10_Init();
	MX_TIM4_Init();
	MX_TIM9_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_TIM11_Init();
	MX_USART6_UART_Init();
	PWM_Init();
	encoder_init();
	odometry_init(81.54, 81.54, 424.77);
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END 2 */
	int state = 0;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//  move_step_motors(50.0);
//  move_step_back(50.0);
//  move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, -100, RIGHT_LEADSCREW_AX, -100, 4500);
//  move_step_motors(50.0);
//  while (pwm_active || pwm_active_back)
//  {
//
//		move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, 100, RIGHT_LEADSCREW_AX,
//	100, 4800);
//  }
//  move_step_motors(0.0);
	while (1) {
		switch (state) {
		case 0:
			// Start moving lead screws
//        	PWM_SetServo_Position(1, 0);
//			HAL_Delay(2000);
//			move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_OFF,BACK_ROTATOR_AX, BACK_ROTATOR_OFF, 100);
//			move_AX_Servo_Sync(LEFT_PUSHER_AX, 60, RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
//			HAL_Delay(2000);
//			move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_ON, RIGHT_PUSHER_AX, RIGHT_PUSHER_ON, 100);

//			move_AX_Servo_Sync(LEFT_STORAGE_AX, LEFT_STORAGE_OFF, RIGHT_STORAGE_AX, RIGHT_STORAGE_OFF, 100);
//			move_step_motors(35.0);
//			move_step_back(35.0);
//			move_AX_Wheels_SyncTime(LEFT_STORAGE_AX, 100, RIGHT_STORAGE_AX, 100, 3000);

			move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, 100, RIGHT_LEADSCREW_AX, 100, 2500);
//			navigate(700, 0, FORWARDS, 800, 1000);
//			move_step_motors(50.0);
//			set_AX_WheelMode(LEFT_STORAGE_AX, 1);
//			HAL_Delay(50);
//			set_AX_WheelMode(RIGHT_STORAGE_AX, 1);
//			HAL_Delay(50);
//			move_AX_Wheels_SyncTime(LEFT_STORAGE_AX, 100, RIGHT_STORAGE_AX, 100, 1700);
			state++;
			break;

		case 1:
			if (!stepper_moving
					&& !stepper_back_moving && !ax_moving) {
//				move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OFF, RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
				move_step_back(88.0);
				move_step_motors(88.0);
				state++;
			}
			break;

		case 2:
			if (!stepper_moving
					&& !stepper_back_moving && !ax_moving) {

				move_AX_Wheels_SyncTime(LEFT_STORAGE_AX, -100, RIGHT_STORAGE_AX, -100, 3000);

//				move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OPENED,
//				RIGHT_PUSHER_AX, RIGHT_PUSHER_OPENED, 100);
//				move_step_motors(0.0);
//				navigate(800, -710, FORWARDS, 600, 600);
				state++;
			}
			break;

		case 3:
			// Wait for steppers, then open rotators
			if (!stepper_moving && !stepper_back_moving && !ax_moving) {
				move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, -100, RIGHT_LEADSCREW_AX, -100, 3000);
//				move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, 100,
//				RIGHT_LEADSCREW_AX, 100, 3200);
//				navigate(800, -210, FORWARDS, 600, 600);
				state++;
			}
			break;

		case 4:
			if (!stepper_moving && !stepper_back_moving && !ax_moving) {
//				move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_CLOSED,
//				RIGHT_PUSHER_AX, RIGHT_PUSHER_CLOSED, 100);
//				move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_OPENED,
//				BACK_ROTATOR_AX, BACK_ROTATOR_OPENED, 100);
//				navigate(360, 0, FORWARDS, 200, 600);

//				move_step_motors(30.0);
//				move_step_back(30.0);
//				state++;
			}
			break;

		case 5:
			if (!stepper_moving && !stepper_back_moving && !ax_moving) {
//				PWM_SetServo_Position(0, 1);
				navigate(0, 0, FORWARDS, 200, 600);

				state++;
			}
			break;

		case 6:
			if (movement_phase == IDLE) {
				state++;
			}
			break;

		case 7:
			if (movement_phase == IDLE) {
				state++;
			}
			break;

		default:
			break;
		}

	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		calculate_odometry();
		cx = get_x();
		cy = get_y();
		cfi = get_fi();

		c_speedl = get_speed_l();
		c_speedr = get_speed_r();

		if (movement_phase == TRANSLATION || movement_phase == ROTATION
				|| movement_phase == IDLE) {
			ref_fi = atan2f(global_goal_y - cy, global_goal_x - cx);
			if (dir == BACKWARDS) {
				ref_fi = ref_fi - M_PI;
			}
		} else if (movement_phase == SPIN) {
			ref_fi = spin_start_angle + spin_target;
		}

		float dx = global_goal_x - cx;
		float dy = global_goal_y - cy;
		remaining = sqrtf(dx * dx + dy * dy);

		if (sys_t % 10 == 0) {
			if (movement_phase == ROTATION) {
				movement_phase = ROTATION;
				v_ref2 = calculate_angular_trapezoid(600, MAX_ANG_ACCEL, cfi,
						ref_fi, &movement_phase);
			} else if (movement_phase == TRANSLATION) {
//				if (dampened == true)
//				{
//
//				}
//				else
//				{
					movement_phase = TRANSLATION;
					v_ref2 = calculate_trapezoid(des_acc, max_vel, cx, cy,
							global_goal_x, global_goal_y, &movement_phase);
//				}
			} else if (movement_phase == SPIN) {
				movement_phase = SPIN;
				float angle_turned = get_unwrapped_fi() - spin_start_angle;
				v_ref2 = calculate_spin_trapezoid(400, MAX_ANG_ACCEL,
						angle_turned, spin_target, &movement_phase);
			}
		}

		if (ax_moving) {
			sys_tax++;
		}

		if (ax_moving && sys_tax % designated_time == 0) {
			designated_time = 0;
			move_AX_Wheels_Sync(LEFT_LEADSCREW_AX, 0, RIGHT_LEADSCREW_AX, 0);
			move_AX_Wheels_Sync(LEFT_STORAGE_AX, 0, RIGHT_STORAGE_AX, 0);
			sys_tax = 0;
			ax_moving = false;

		}
		movement_PID(v_ref2, &movement_phase, MAX_ACCEL, global_goal_x,
				global_goal_y, ref_fi, dir);
		sys_t = (sys_t >= 2000) ? 0 : sys_t + 1;

	}

	if (htim->Instance == TIM1) {
		if (pwm_active) {
			step_counter++;
			if (step_counter >= nmbr_of_steps) {
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				pwm_active = 0;
				current_position = target_pos;
				stepper_moving = false;
				step_counter = 0;
			}
		}
		if (pwm_active_back) {
			step_counter_back++;
			if (step_counter_back >= nmbr_of_steps_back) {
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
				pwm_active_back = 0;
				current_position_back = target_pos_back;
				stepper_back_moving = false;
			}
		}
		if (pwm_active == 0 && pwm_active_back == 0) {
			HAL_TIM_Base_Stop_IT(&htim1);
		}
	}
}

void navigate(float tx, float ty, int8_t direction, float acc, float vel) {

	reset_PID();
	reset_move_profiles();
	start_x1 = cx;
	start_y1 = cy;
	global_goal_x = tx;
	global_goal_y = ty;
	dir = direction;
	movement_phase = ROTATION;
	des_acc = acc;
	max_vel = vel;
	dampened = false;
}

void navigate_dampened(float tx, float ty, int8_t direction, float acc, float vel, float tx2, float ty2, float acc2, float vel2) {

	reset_PID();
	reset_move_profiles();
	start_x1 = cx;
	start_y1 = cy;
	dampened_goal_x1 = tx;
	dampened_goal_y1 = ty;
	dampened_goal_x2 = tx2;
	dampened_goal_y2 = ty2;
	dampened_vel1 = vel;
	dampened_vel2 = vel2;
	dampened_acc1 = acc;
	dampened_acc2 = acc2;
	dir = direction;
	dampened = true;
	movement_phase = ROTATION;

}

void spin_robot(float num_circles) {
	reset_PID();
	reset_move_profiles();

	spin_start_angle = get_unwrapped_fi(); // Get current absolute angle
	spin_target = num_circles * 2.0f * M_PI;

	movement_phase = SPIN;
}

bool read_sensors() {
	if (GPIOC->IDR & GPIO_PIN_8) {
		sensor_filter_counter++;
		if (sensor_filter_counter >= FILTER_THRESHOLD) {
			return true;
		}
	} else {
		sensor_filter_counter = 0;
		return false;
	}
}

void turn_crates() {
	uint8_t inputVal = (GPIOB->IDR >> 12) & 0x0F;

	switch (inputVal) {
	case 0x01:
//        	uso = true;
		PWM_SetServo_Position(1, 180);
		break;

	case 0x02:
		//
		break;

	case 0x04:
		// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2500);
		break;

	case 0x08:
		// Use TIM2_CH2 or similar here.
		break;

	default:
		//
		break;
	}
}

void move_AX_Wheels_SyncTime(uint8_t id1, float speed1, uint8_t id2,
		float speed2, int time) {
	designated_time = time;
	sys_tax = 0;
	move_AX_Wheels_Sync(id1, speed1, id2, speed2);
	ax_moving = true;
}
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 15;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 450;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

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
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 400 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 400 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */
////
	/* USER CODE END TIM9_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */
////
	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 83;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 899;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 450;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */
////
	/* USER CODE END TIM9_Init 2 */
	HAL_TIM_MspPostInit(&htim9);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 15;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 999;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 15;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 19999;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */
	HAL_TIM_MspPostInit(&htim11);

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
//
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC8 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA7 PA8 PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PC4 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_1;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
