/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PID.h"
#include "board_config.h"
#include "motors.h"
#include "SpeedProfile.h"
#include "encoders.h"
#include "sequences.h"
#include "odometry.h"

/* Timer handles -------------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

/* UART handles --------------------------------------------------------------*/
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* Motion parameters ---------------------------------------------------------*/
float des_acc = 200;
float max_vel = 400;
static bool rotate_only = false;

/* Odometry / pose -----------------------------------------------------------*/
volatile float ref_fi;
volatile float spin_target      = 0.0f;
volatile float spin_start_angle = 0.0f;
volatile float remaining;
volatile float cx, cy, cfi;
volatile float c_speedr, c_speedl;
volatile float c_speedr_old, c_speedl_old;
volatile float c_speedl_new, c_speedr_new;

/* Navigation goal (internal coordinate frame) --------------------------------*/
float global_goal_x = 0;
float global_goal_y = 0;

/* Motion state --------------------------------------------------------------*/
uint8_t          movement_phase = IDLE;
volatile int8_t  dir;
volatile float   v_ref2;

/* System tick counters ------------------------------------------------------*/
volatile int sys_t           = 0;
volatile int sys_tax         = 0;
volatile int designated_time = 0;
volatile int mechanism_timer = 0;

/* Sensor filter -------------------------------------------------------------*/
bool      debug_sensor        = false;
int       sensor_filter_counter = 0;
const int FILTER_THRESHOLD      = 10;

/* Motion start snapshot -----------------------------------------------------*/
float        start_x1, start_y1;
volatile bool ax_moving = false;

/* Stepper externals ----------------------------------------------------------*/
extern volatile float current_position, current_position_back;
extern volatile int   step_counter, step_counter_back;
extern volatile int   pwm_active, pwm_active_back;
extern float          target_pos, target_pos_back;
extern int            nmbr_of_steps, nmbr_of_steps_back;
extern volatile bool  stepper_moving, stepper_back_moving;

/* Active targets ------------------------------------------------------------*/
static CratePosition  target_crate;
static PantryPosition target_pantry;

/* Crate definitions ---------------------------------------------------------*/
CratePosition crate1  = { .approach_x = 470,  .approach_y = 1200, .x = 170,  .y = 1200, .flag = true  };
CratePosition crate2  = { .approach_x = 470,  .approach_y = 400,  .x = 170,  .y = 400,  .flag = true  };
CratePosition crate3  = { .approach_x = 1100, .approach_y = 470,  .x = 1100, .y = 170,  .flag = true  };
CratePosition crate4  = { .approach_x = 1150, .approach_y = 510,  .x = 1150, .y = 790,  .flag = false };
CratePosition crate5  = { .approach_x = 1150, .approach_y = 1090, .x = 1150, .y = 810,  .flag = false };
CratePosition crate6  = { .approach_x = 1850, .approach_y = 1090, .x = 1850, .y = 810,  .flag = false };
CratePosition crate7  = { .approach_x = 1850, .approach_y = 510,  .x = 1850, .y = 790,  .flag = false };
CratePosition crate8  = { .approach_x = 1900, .approach_y = 470,  .x = 1900, .y = 170,  .flag = true  };
CratePosition crate9  = { .approach_x = 2530, .approach_y = 400,  .x = 2820, .y = 400,  .flag = true  };
CratePosition crate10 = { .approach_x = 2530, .approach_y = 1200, .x = 2820, .y = 1200, .flag = true  };

/* Pantry definitions --------------------------------------------------------*/
PantryPosition pantry1  = { .approach_x = 230,  .approach_y = 800,  .x = 460,  .y = 800,  .flag = true  };  // +60 in +x
PantryPosition pantry2  = { .approach_x = 700,  .approach_y = 230,  .x = 700,  .y = 460,  .flag = true  };  // +60 in +y
PantryPosition pantry3  = { .approach_x = 800,  .approach_y = 800,  .x = 800,  .y = 800,  .flag = false };
PantryPosition pantry4  = { .approach_x = 1250, .approach_y = 1270, .x = 1250, .y = 1040, .flag = true  };  // +60 in -y
PantryPosition pantry5  = { .approach_x = 1500, .approach_y = 800,  .x = 1500, .y = 800,  .flag = false };
PantryPosition pantry6  = { .approach_x = 1500, .approach_y = 230,  .x = 1500, .y = 460,  .flag = true  };  // +60 in +y
PantryPosition pantry7  = { .approach_x = 1750, .approach_y = 1270, .x = 1750, .y = 1040, .flag = true  };  // +60 in -y
PantryPosition pantry8  = { .approach_x = 2200, .approach_y = 800,  .x = 2200, .y = 800,  .flag = false };
PantryPosition pantry9  = { .approach_x = 2300, .approach_y = 230,  .x = 2300, .y = 460,  .flag = true  };  // +60 in +y
PantryPosition pantry10 = { .approach_x = 2770, .approach_y = 800,  .x = 2540, .y = 800,  .flag = true  };  // +60 in -x

/* FSM states ----------------------------------------------------------------*/
int state          = -1;
int state_mechanism = 100;
int state_pickup    = 100;
int state_put_down  = 100;

/* FSM trigger flags (false = trigger reset on next tick) --------------------*/
bool started          = true;
bool started_pickup   = true;
bool started_put_down = true;
int pickup_side = 0;
/* Debug variables -----------------------------------------------------------*/
int debugaa, debuga;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);

uint8_t ss1, ss2, ss3, ss4;


/* ===========================================================================
 *  MAIN
 * ===========================================================================*/
int main(void)
{
    /* Core init */
 	MPU_Config();
    HAL_Init();
    SystemClock_Config();

    /* Peripheral init */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM7_Init();
    MX_UART5_Init();
    MX_UART4_Init();
    MX_USART3_UART_Init();

    PWM_Init();
    HAL_TIM_Base_Start_IT(&htim7);
    encoder_init();
    odometry_init(81.54, 81.54, 422.0);

    /* Servo wheel init sequence */
    set_WheelMode(1); HAL_Delay(300);
    set_WheelMode(2); HAL_Delay(100);
    set_acceleration(1, 254); HAL_Delay(300);
    set_acceleration(2, 254); HAL_Delay(300);
    set_WheelMode(1); HAL_Delay(300);
    set_WheelMode(2); HAL_Delay(300);

    /* Position init */
//    set_x_y(219, 1827, 0); // Yellow team
    set_x_y(2781, 1827, 0); // Blue team

    /* Mechanism startup sequence */
//  leadscrew_closed(2000);
//  plazma_off();
//  HAL_Delay(500);
//    HAL_Delay(500);
//  rotators_on();
    HAL_Delay(500);
    pushers_off();
    HAL_Delay(500);
    rotators_off();
//    HAL_Delay(500);


//    state = 100;
//    leadscrew_opened(1500);
//    leadscrew_closed(1500);
//    HAL_Delay(3000);
    /* Main loop */
    while (1)
    {
        bool motors_idle = !ax_moving && !stepper_back_moving && !stepper_moving;
        switch (state)
        {
            case -1:
                if (movement_phase == IDLE) {
                    navigate(2781, 1735, FORWARDS, 100, 100);
                    state++;
                }
                break;

            case 0:
                if (movement_phase == IDLE) {
                	rotate_to_point(2435, 1200, FORWARDS);
                	state++;
                }
                break;

            case 1:
            	if (movement_phase == IDLE)
            	{
					navigate(2435, 1200, FORWARDS, 1000, MAX_PHYSICAL_SPEED);
					leadscrew_opened(2000);
					move_step_motors(40.0);
					state++;
            	}
            	break;

            case 2:
                if (movement_phase == IDLE) {
                    pickup_crate(crate6, FORWARDS);
                    state++;
                }
                break;

            case 3:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 4:
                if (motors_idle) {
                    put_down_pantry(pantry8);
                    state++;
                }
                break;

            case 5:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 6:
                if (motors_idle) {
                    pickup_crate(crate8, BACKWARDS);
                    state++;
                }
                break;

            case 7:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 8:
                if (motors_idle) {
//                	put_down_pantry(pantry8);
                    state++;
                }
                break;

            case 9:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 10:
                if (motors_idle) {
//                    pickup_crate(crate6);
                    state++;
                }
                break;

            case 11:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 12:
                if (motors_idle) {
//                    pickup_crate(crate7);
                    state++;
                }
                break;

            case 13:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 14:
                if (motors_idle) {
//                    pickup_crate(crate8);
                    state++;
                }
                break;

            case 15:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 16:
                if (motors_idle) {
//                    pickup_crate(crate9);
                    state++;
                }
                break;

            case 17:
				/* Wait here for put_down or pickup FSM to advance state */
                break;

            case 18:
                if (motors_idle) {
//                    pickup_crate(crate8);
                    state++;
                }
                break;
        }

        put_down_fsm();
        pickup_fsm();
        mechanism_fsm();
    }
}


/* ===========================================================================
 *  FSMs (Finite State Machines)
 * ===========================================================================*/

void mechanism_fsm(void)
{
    bool motors_idle = !ax_moving && !stepper_back_moving && !stepper_moving;

    if (!started) {
        state_mechanism = 0;
    }
    started = true;

    switch (state_mechanism)
    {
        case 0:
            if (motors_idle) {
                pushers_on();
                move_step_motors(0.0);
                state_mechanism++;
            }
            break;

        case 1:
            if (motors_idle) {
              leadscrew_closed(1000);
              start_mechanism_wait(700);
                state_mechanism++;
            }
            break;

        case 2:
        	if (mechanism_wait_complete())
        		state_mechanism++;
            break;

        case 3:
            if (motors_idle) {
            	pushers_off();
                steppers_up(80.0);
                state++;            /* Signal main loop to continue movement */
                state_mechanism++;

            }
            break;

        case 4:
            if (motors_idle) {
                start_mechanism_wait(1000);
                rotators_on();
                state_mechanism++;
            }
            break;

        case 5:
            if (motors_idle && mechanism_wait_complete()) {
            	turn_crates_inner();
				start_mechanism_wait(1000);
				state_mechanism++;
            }
            break;

        case 6:
            if (motors_idle && mechanism_wait_complete()) {
              rotators_off();
                state_mechanism++;
                start_mechanism_wait(500);
            }
            break;

        case 7:
            if (motors_idle && mechanism_wait_complete()) {
            	steppers_up(10.0);
				state_mechanism++;
            }
            break;
        case 8:
        	if(motors_idle)
        	{
        		leadscrew_opened(1000);
        		start_mechanism_wait(1000);
        		state_mechanism++;
        	}
        case 9:
        	if (motors_idle && mechanism_wait_complete())
        	{
        		move_step_back(0.0);
        		state_mechanism++;
                /* Terminal state — waiting for external trigger */
        	}
    }
}

/* ---------------------------------------------------------------------------*/

void pickup_fsm(void)
{
    if (!started_pickup) {
        state_pickup = 0;
    }
    started_pickup = true;

    switch (state_pickup)
    {
        case 0:
            if (movement_phase == IDLE) {
            	navigate(target_crate.approach_x, target_crate.approach_y, pickup_side, 1000, MAX_PHYSICAL_SPEED);
                state_pickup++;
            }
            break;

        case 1:
            if (movement_phase == IDLE) {
            	rotate_to_point(target_crate.x, target_crate.y, FORWARDS);
                state_pickup++;
            }
            break;

        case 2:
        	if (movement_phase == IDLE)
        	{
        		take_a_picture();
        		HAL_Delay(500);
            	navigate(target_crate.x, target_crate.y, FORWARDS, 200, 200);
            	state_pickup++;
        	}
        case 3:
            if (movement_phase == IDLE) {
            	activate_mechanism();
                HAL_Delay(500);
                state_pickup++;

                if (target_crate.flag == false) {
                    state_pickup = 10;  /* Skip retreat — crate doesn't need it */
                }
            }
            break;

        case 4:
            if (movement_phase == IDLE && state_mechanism == 3) {
                navigate(target_crate.approach_x, target_crate.approach_y,
                         BACKWARDS, 1000, MAX_PHYSICAL_SPEED);
                state_pickup++;
            }
            break;
    }
}

/* ---------------------------------------------------------------------------*/

void put_down_fsm(void)
{

    bool motors_idle = !ax_moving && !stepper_back_moving && !stepper_moving;

    if (!started_put_down) {
        state_put_down = 0;
    }
    started_put_down = true;

    switch (state_put_down)
    {
        case 0:
            if (movement_phase == IDLE) {
                navigate(target_pantry.approach_x, target_pantry.approach_y,
                         FORWARDS, 1000, MAX_PHYSICAL_SPEED);
                state_put_down++;
            }
            break;

        case 1:
            if (movement_phase == IDLE && state_mechanism == 10) {
            	move_step_motors(40.0);
                state_put_down = 10;
                if (target_crate.flag == true) {
                    state_put_down = 2;
                }
            }
            break;

        case 2:
			if (movement_phase == IDLE && motors_idle) {
				rotate_to_point(target_pantry.x, target_pantry.y, BACKWARDS);
				state_put_down++;
			}
			break;

        case 3:
            if (movement_phase == IDLE && motors_idle) {
                state_put_down++;
            }
            break;
        case 4:
        	if(movement_phase == IDLE  && motors_idle)
        	{
                navigate(target_pantry.x, target_pantry.y,
                         BACKWARDS, 1000, MAX_PHYSICAL_SPEED);
                state_put_down++;
        	}
        case 5:
        	if(movement_phase == IDLE)
        	{
        		state++;
				state_put_down++;
        	}
        case 10:
        	if (motors_idle)
        	{
        		state++;
        	}
        	break;
    }
}


/* ===========================================================================
 *  CONTROL LOOP  (PID / Odometry — called from TIM7 @ 1 kHz)
 * ===========================================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* --- TIM7: 1 ms PID / odometry tick ----------------------------------- */
    if (htim->Instance == TIM7)
    {
        debug_sensor = read_sensors();
        calculate_odometry();

        cx          = get_x();
        cy          = get_y();
        cfi         = get_fi();
        c_speedl_new = get_speed_l();
        c_speedr_new = get_speed_r();

        c_speedl = c_speedl_new;
        c_speedr = c_speedr_new;

        if (!read_sensors())
        {
            /* Update heading reference */
            if (movement_phase == TRANSLATION ||
                movement_phase == ROTATION    ||
                movement_phase == IDLE)
            {
                ref_fi = atan2f(global_goal_y - cy, global_goal_x - cx);
                if (dir == BACKWARDS && movement_phase == ROTATION)
                    ref_fi -= M_PI;
            }
            else if (movement_phase == SPIN)
            {
                ref_fi = spin_start_angle + spin_target;
            }

            /* Distance remaining to goal */
            float dx  = global_goal_x - cx;
            float dy  = global_goal_y - cy;
            remaining = sqrtf(dx * dx + dy * dy);

            /* Update velocity reference every 10 ms */
            if (sys_t % 10 == 0)
            {
                if (movement_phase == ROTATION)
                {
                    v_ref2 = calculate_angular_trapezoid(50000, 180000,
                                                         cfi, ref_fi,
                                                         &movement_phase);
                    if (rotate_only && movement_phase == TRANSLATION) {
                        movement_phase = IDLE;
                        rotate_only    = false;
                        reset_PID();
                        reset_move_profiles();
                    }
                }
                else if (movement_phase == TRANSLATION)
                {
                    v_ref2 = calculate_trapezoid(des_acc, max_vel,
                                                 cx, cy,
                                                 global_goal_x, global_goal_y,
                                                 &movement_phase);
                }
                else if (movement_phase == SPIN)
                {
                    float angle_turned = get_unwrapped_fi() - spin_start_angle;
                    v_ref2 = calculate_spin_trapezoid(100000, 100000,
                                                      angle_turned, spin_target,
                                                      &movement_phase);
                }
            }
        }
        else
        {
            /* Sensor triggered — bleed off speed and reset profiles */
            if (sys_t % 10 == 0)
                v_ref2 *= 0.95f;
            reset_move_profiles();
        }

        /* Servo wheel timeout handler */
        if (ax_moving)
        {
            sys_tax++;
            if (sys_tax >= designated_time)
            {
                move_Wheels_Sync(1, 0, 2, 0);
                designated_time = 0;
                sys_tax         = 0;
                ax_moving       = false;
            }
        }

        if (mechanism_timer>0)
        {
        	mechanism_timer--;
        }

        movement_PID(v_ref2, &movement_phase, MAX_ACCEL,
                     global_goal_x, global_goal_y, ref_fi, dir);

        c_speedr_old = c_speedr;
        c_speedl_old = c_speedl;
        sys_t = (sys_t >= 2000) ? 0 : sys_t + 1;
    }

    /* --- TIM4: stepper PWM step counter ----------------------------------- */
    if (htim->Instance == TIM4)
    {
        if (pwm_active)
        {
            step_counter++;
            if (step_counter >= nmbr_of_steps)
            {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                pwm_active       = 0;
                current_position = target_pos;
                stepper_moving   = false;
                step_counter     = 0;
            }
        }

        if (pwm_active_back)
        {
            step_counter_back++;
            if (step_counter_back >= nmbr_of_steps_back)
            {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
                pwm_active_back       = 0;
                current_position_back = target_pos_back;
                stepper_back_moving   = false;
            }
        }

        if (!pwm_active && !pwm_active_back)
            HAL_TIM_Base_Stop_IT(&htim4);
    }
}


/* ===========================================================================
 *  HELPER FUNCTIONS
 * ===========================================================================*/

/**
 * @brief  Set a navigation goal and start the ROTATION → TRANSLATION sequence.
 *
 * Coordinate mapping: global_goal_x = -ty, global_goal_y = -tx
 */
void navigate(float tx, float ty, int8_t direction, float acc, float vel)
{
    reset_PID();
    reset_move_profiles();

    start_x1 = cx;
    start_y1 = cy;

    global_goal_x = -ty;
    global_goal_y = -tx;
    dir           = direction;
    des_acc       = acc;
    max_vel       = vel;
    movement_phase = ROTATION;
}

/* ---------------------------------------------------------------------------*/

/**
 * @brief  Set a rotation goal and start the ROTATION sequence.
 *
 * Coordinate mapping: global_goal_x = -ty, global_goal_y = -tx
 */
void rotate_to_point(float tx, float ty, int direction)
{
    reset_PID();
    reset_move_profiles();

    global_goal_x  = -ty;
    global_goal_y  = -tx;
    dir            = direction;
    des_acc        = 1000;
    max_vel        = MAX_PHYSICAL_SPEED;
    movement_phase = ROTATION;
    rotate_only    = true;
}

/* ---------------------------------------------------------------------------*/

/**
 * @brief  Spin in place by a given number of full circles.
 */
void spin_robot(float num_circles)
{
    reset_PID();
    reset_move_profiles();
    spin_start_angle = get_unwrapped_fi();
    spin_target      = num_circles * 2.0f * M_PI;
    movement_phase   = SPIN;
}

/* ---------------------------------------------------------------------------*/

/**
 * @brief  Drive AX or Waveshare servo wheels for a fixed duration (ms).
 */
void move_Wheels_SyncTime(uint8_t id1, float speed1,
                          uint8_t id2, float speed2,
                          int time, int type)
{
    designated_time = time;
    sys_tax         = 0;
    ax_moving       = true;

    if (type == AX)
        move_AX_Wheels_Sync(id1, speed1, id2, speed2);
    else if (type == WAVESHARE)
        move_Wheels_Sync(id1, speed1, id2, speed2);
}

/* ---------------------------------------------------------------------------*/

bool read_sensors(void)
{
    return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) ? true : false;
}

/* ---------------------------------------------------------------------------*/

void start_mechanism_wait(int time_ms) {
    mechanism_timer = time_ms;
}

bool mechanism_wait_complete() {
    return (mechanism_timer == 0);
}

/* Raspberry Pi command bytes ------------------------------------------------*/
void take_a_picture(void)  { uint8_t msg = 1; HAL_UART_Transmit(&huart3, &msg, 1, 10); }
void turn_crates_outer(void) { uint8_t msg = 2; HAL_UART_Transmit(&huart3, &msg, 1, 10); }
void turn_crates_inner(void) { uint8_t msg = 3; HAL_UART_Transmit(&huart3, &msg, 1, 10); }
void finalize_crates(void)   { uint8_t msg = 4; HAL_UART_Transmit(&huart3, &msg, 1, 10); }

/* FSM trigger helpers -------------------------------------------------------*/
void pickup_crate(CratePosition pos, int side)    { started_pickup   = false; target_crate  = pos; pickup_side = side;}
void put_down_pantry(PantryPosition pos) { started_put_down = false; target_pantry = pos; }
void activate_mechanism(void)            { started          = false; }


/* ===========================================================================
 *  PERIPHERAL INITIALIZATION
 * ===========================================================================*/
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR1_SLP_Pin|MOTOR2_SLP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR1_DIR_Pin|MOTOR2_DIR_Pin|MS1_1_MICROSTEP_Pin|STEP1_DIR_Pin
                          |STEPDRIVER1_EN_Pin|MS1_2_MICROSTEP_Pin|STEPDRIVER2_EN_Pin|STEP2_DIR_Pin
                          |MS2_1_MICROSTEP_Pin|MS2_2_MICROSTEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG4 PG5 PG6
                           PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
