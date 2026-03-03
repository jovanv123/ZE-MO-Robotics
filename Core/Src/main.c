#include "main.h"
#include "../Inc/SpeedProfile.h"
#include "board_config.h"
#include "../Inc/motors.h"

UART_HandleTypeDef huart6, huart2;
TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim9, htim10, htim11;
DMA_HandleTypeDef hdma_usart6_rx;

float des_acc = 200;
float max_vel = 400;

volatile float ref_fi;
volatile float spin_target      = 0.0f;
volatile float spin_start_angle = 0.0f;
volatile float remaining;
volatile float cx, cy, cfi, c_speedr, c_speedl;
float global_goal_x, global_goal_y;
uint8_t  movement_phase;
volatile int8_t  dir;
volatile float   v_ref2;

int sys_t            = 0;
int sys_tax          = 0;
int designated_time  = 0;

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

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);

bool read_sensors(void);
void turn_crates(void);
void navigate(float tx, float ty, int8_t direction, float acc, float vel);
void spin_robot(float num_circles);
void move_AX_Wheels_SyncTime(uint8_t id1, float speed1, uint8_t id2, float speed2, int time);

uint32_t freq;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

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

    init_robot(3000);

    int state = 0;

    while (1) {
        bool motors_idle = !stepper_moving && !stepper_back_moving && !ax_moving;

        switch (state) {
        case 0:
            if (motors_idle) {
                pushers_on();
                leadscrew_closed(3000);
            }
            break;
        case 1:
            if (motors_idle) {
                pushers_off();
                steppers_up();
                state++;
            }
            break;
        case 2:
            if (motors_idle) {
                plazma_on();
                HAL_Delay(1500);
                storage_on();
                HAL_Delay(1500);
                state++;
            }
            break;
        case 3:
            if (motors_idle) {
                leadscrew_opened(3000);
                state++;
            }
            break;
        case 4:
            if (motors_idle) {
                move_step_back(0.0);
                HAL_Delay(15000);
                move_step_motors(0.0);
                state++;
            }
            break;
        case 5:
            if (motors_idle) {
                state++;
            }
            break;
        case 6:
            if (motors_idle) {
                move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OFF,
                                   RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
                state++;
            }
            break;
        case 7:
            if (motors_idle) {
                state++;
            }
            break;
        case 8:
            if (motors_idle) {
                state++;
            }
            break;
        default:
            break;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        calculate_odometry();
        cx       = get_x();
        cy       = get_y();
        cfi      = get_fi();
        c_speedl = get_speed_l();
        c_speedr = get_speed_r();

        if (movement_phase == TRANSLATION || movement_phase == ROTATION || movement_phase == IDLE) {
            ref_fi = atan2f(global_goal_y - cy, global_goal_x - cx);
            if (dir == BACKWARDS)
                ref_fi -= M_PI;
        } else if (movement_phase == SPIN) {
            ref_fi = spin_start_angle + spin_target;
        }

        float dx = global_goal_x - cx;
        float dy = global_goal_y - cy;
        remaining = sqrtf(dx * dx + dy * dy);

        if (sys_t % 10 == 0) {
            if (movement_phase == ROTATION) {
                v_ref2 = calculate_angular_trapezoid(600, MAX_ANG_ACCEL, cfi, ref_fi, &movement_phase);
            } else if (movement_phase == TRANSLATION) {
                v_ref2 = calculate_trapezoid(des_acc, max_vel, cx, cy, global_goal_x, global_goal_y, &movement_phase);
            } else if (movement_phase == SPIN) {
                float angle_turned = get_unwrapped_fi() - spin_start_angle;
                v_ref2 = calculate_spin_trapezoid(400, MAX_ANG_ACCEL, angle_turned, spin_target, &movement_phase);
            }
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
        sys_t = (sys_t >= 2000) ? 0 : sys_t + 1;
    }

    if (htim->Instance == TIM1) {
        if (pwm_active) {
            step_counter++;
            if (step_counter >= nmbr_of_steps) {
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
                pwm_active       = 0;
                current_position = target_pos;
                stepper_moving   = false;
                step_counter     = 0;
            }
        }
        if (pwm_active_back) {
            step_counter_back++;
            if (step_counter_back >= nmbr_of_steps_back) {
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
                pwm_active_back       = 0;
                current_position_back = target_pos_back;
                stepper_back_moving   = false;
            }
        }
        if (pwm_active == 0 && pwm_active_back == 0)
            HAL_TIM_Base_Stop_IT(&htim1);
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
    if (GPIOC->IDR & GPIO_PIN_8) {
        sensor_filter_counter++;
        if (sensor_filter_counter >= FILTER_THRESHOLD)
            return true;
    } else {
        sensor_filter_counter = 0;
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

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef       sClockSourceConfig  = { 0 };
    TIM_MasterConfigTypeDef      sMasterConfig       = { 0 };
    TIM_OC_InitTypeDef           sConfigOC           = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    htim1.Instance                 = TIM1;
    htim1.Init.Prescaler           = 15;
    htim1.Init.CounterMode         = TIM_COUNTERMODE_UP;
    htim1.Init.Period              = 999;
    htim1.Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter   = 0;
    htim1.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 450;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef      sConfigOC     = { 0 };

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 400 - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef      sConfigOC     = { 0 };

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 400 - 1;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

static void MX_TIM5_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 0;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 4294967295;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;
    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM9_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = { 0 };

    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 83;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = 899;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 450;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim9);
}

static void MX_TIM10_Init(void)
{
    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 15;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 999;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) Error_Handler();
}

static void MX_TIM11_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = { 0 };

    htim11.Instance               = TIM11;
    htim11.Init.Prescaler         = 15;
    htim11.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim11.Init.Period            = 19999;
    htim11.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim11)  != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim11)   != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim11);
}

static void MX_USART6_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 9600;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_HalfDuplex_Init(&huart6) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_1 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
