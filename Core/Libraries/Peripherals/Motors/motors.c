/*
 * pwm.c
 *
 *  Created on: Dec 22, 2025
 *      Author: root
 */

#include "motors.h"
int debug;
volatile uint32_t debug_ODR_A;
volatile uint32_t ccr_val;
void PWM_Init()
{
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_4, 0);
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER1, TIM_CHANNEL_1);//Levi tocak
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER2, TIM_CHANNEL_2);//Desni tocak
//	HAL_TIM_PWM_Start(&SERVO_TIMER, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&SERVO_TIMER, TIM_CHANNEL_2);

//	__HAL_TIM_MOE_ENABLE(&SERVO_TIMER);
}

void PWM_SetSpeed_Left(float speed)
{

    if (speed >= 0)
    {
        GPIOA->BSRR = (1 << 9);
    }
    else
    {
        GPIOA->BSRR = (1 << (9 + 16));
        speed = -speed;
    }


    if (speed > MAX_PHYSICAL_SPEED) speed = (float)MAX_PHYSICAL_SPEED;

    uint32_t ccr_val = (uint32_t)((speed * (float)PWM_MAX_VALUE) / (float)MAX_PHYSICAL_SPEED);
    MOTOR_PWM_TIMER1.Instance->CCR1 = ccr_val;
}


void PWM_SetSpeed_Right(float speed)
{

    if (speed >= 0)
    {
        GPIOA->BSRR = (1 << 8);
    }
    else
    {
        GPIOA->BSRR = (1 << (8 + 16));
        speed = -speed;
    }


    if (speed > MAX_PHYSICAL_SPEED) speed = (float)MAX_PHYSICAL_SPEED;

    uint32_t ccr_val = (uint32_t)((speed * (float)PWM_MAX_VALUE) / (float)MAX_PHYSICAL_SPEED);
    MOTOR_PWM_TIMER2.Instance->CCR2 = ccr_val;
}


void PWM_SetServo_Position(uint32_t servo_number, uint16_t angle)
{
	if (angle>180) angle = 180;
	uint32_t angle_to_pwm = 250 + (angle*(1000))/180;
	switch (servo_number)
	{
	case 1:
		SERVO_TIMER.Instance->CCR1 = angle_to_pwm;
		break;
	case 2:
		SERVO_TIMER.Instance->CCR2 = angle_to_pwm;
		break;
	case 3:
		SERVO_TIMER.Instance->CCR3 = angle_to_pwm;
		break;
	case 4:
		SERVO_TIMER.Instance->CCR4 = angle_to_pwm;
		debug = 1;
		break;
	default:
		break;
	}
}

void move_AX(uint8_t id, float degrees, float speed_percent)
{

    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 300.0f) degrees = 300.0f;
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;

    uint16_t raw_position = (uint16_t)((degrees / 300.0f) * 1023.0f);
    uint16_t raw_speed = (uint16_t)((speed_percent / 100.0f) * 1023.0f);

    if (raw_speed == 0 && speed_percent > 0) raw_speed = 1;

    uint8_t packet[11];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x07;
    packet[4] = 0x03;
    packet[5] = 0x1E;

    packet[6] = (uint8_t)(raw_position & 0xFF);
    packet[7] = (uint8_t)((raw_position >> 8) & 0xFF);

    packet[8] = (uint8_t)(raw_speed & 0xFF);
    packet[9] = (uint8_t)((raw_speed >> 8) & 0xFF);

    uint32_t checksum_sum = packet[2] + packet[3] + packet[4] + packet[5] +
                            packet[6] + packet[7] + packet[8] + packet[9];
    packet[10] = (uint8_t)(~(checksum_sum) & 0xFF);

    HAL_UART_Transmit(&huart6, packet, 11, 100);
}

void Stop_Motors()
{
	MOTOR_PWM_TIMER1.Instance->CCR1 = 0;
	MOTOR_PWM_TIMER2.Instance->CCR2 = 0;
}
//Potreban jos jedan 4-kanalni PWM
