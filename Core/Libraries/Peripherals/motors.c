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
extern volatile bool ax_moving;

volatile int debug_pin1, debug_pin2;

volatile float current_position = 0, current_position_back = 0;
volatile int step_counter = 0, step_counter_back = 0;
volatile int pwm_active = 0, pwm_active_back = 0;
float target_pos = 0, target_pos_back = 0;
int nmbr_of_steps = 0, nmbr_of_steps_back = 0;
volatile bool stepper_moving = false, stepper_back_moving = false;

void PWM_Init() {
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_1); //Levi tocak
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_2); //Desni tocak
}

void PWM_SetSpeed_Left(float speed) {

	if (speed >= 0) {
		debug_pin1 = 1;
		GPIOF->BSRR = (1 << 6);
	} else {
		GPIOF->BSRR = (1 << (6 + 16));
		debug_pin1 = 0;
		speed = -speed;
	}

	if (speed > MAX_PHYSICAL_SPEED)
		speed = (float) MAX_PHYSICAL_SPEED;

	uint32_t ccr_val = (uint32_t) ((speed * (float) PWM_MAX_VALUE)
			/ (float) MAX_PHYSICAL_SPEED);
	MOTOR_PWM_TIMER.Instance->CCR1 = ccr_val;
}

void PWM_SetSpeed_Right(float speed) {

	if (speed >= 0) {
		debug_pin2 = 1;
		GPIOF->BSRR = (1 << 5);
	} else {
		GPIOF->BSRR = (1 << (5 + 16));
		debug_pin2 = 0;
		speed = -speed;
	}

	if (speed > MAX_PHYSICAL_SPEED)
		speed = (float) MAX_PHYSICAL_SPEED;

	uint32_t ccr_val = (uint32_t) ((speed * (float) PWM_MAX_VALUE)
			/ (float) MAX_PHYSICAL_SPEED);
	MOTOR_PWM_TIMER.Instance->CCR2 = ccr_val;
}

void move_step_motors(float target) {
	target_pos = target;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8,
			(target_pos > current_position) ? 0 : 1);

	float delta = fabs(target_pos - current_position);
	nmbr_of_steps = (int) ((delta / 8) * 200);

	step_counter = 0;
	pwm_active = 1;
	stepper_moving = true;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
}

void move_step_back(float target_back) {
	target_pos_back = target_back;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13,
			(target_pos_back > current_position_back) ? 0 : 1);

	float delta_back = fabs(target_pos_back - current_position_back);
	nmbr_of_steps_back = (int) ((delta_back / 8) * 200);

	step_counter_back = 0;
	pwm_active_back = 1;
	stepper_back_moving = true;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
}

void set_acceleration(uint8_t id, uint8_t acceleration)
{
	static uint8_t packet[8];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = 0x04;
	packet[4] = 0x03;
	packet[5] = 0x29;
	packet[6] = acceleration;

	uint32_t checksum = packet[2] + packet[3] + packet[4] + packet[5]
			+ packet[6];
	packet[7] = (uint8_t) (~checksum & 0xFF);

	HAL_UART_Transmit_IT(&huart4, packet, 8);
}

void set_WheelMode(uint8_t id) {
	static uint8_t packet[8];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = 0x04;
	packet[4] = 0x03;
	packet[5] = 0x21;
	packet[6] = 0x01;

	uint32_t checksum = packet[2] + packet[3] + packet[4] + packet[5]
			+ packet[6];
	packet[7] = (uint8_t) (~checksum & 0xFF);

	HAL_UART_Transmit_IT(&huart4, packet, 8);
}

void move_Wheels_Sync(uint8_t id1, float speed1, uint8_t id2, float speed2) {
    float speeds[2] = {speed1, speed2};
    uint8_t ids[2] = {id1, id2};
    uint16_t raw_speeds[2];

    for (int i = 0; i < 2; i++) {
        if (speeds[i] > 7000.0f)
            speeds[i] = 7000.0f;
        if (speeds[i] < -7000.0f)
            speeds[i] = -7000.0f;

        uint16_t speed_val = (uint16_t) fabs(speeds[i]);
        if (speeds[i] < 0) {
            raw_speeds[i] = speed_val | 0x8000;
        } else {
            raw_speeds[i] = speed_val;
        }
    }

    static uint8_t packet[16];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFE;
    packet[3] = 0x0A;
    packet[4] = 0x83;
    packet[5] = 0x2E;
    packet[6] = 0x02;

    packet[7] = ids[0];
    packet[8] = (uint8_t) (raw_speeds[0] & 0xFF);
    packet[9] = (uint8_t) ((raw_speeds[0] >> 8) & 0xFF);

    packet[10] = ids[1];
    packet[11] = (uint8_t) (raw_speeds[1] & 0xFF);
    packet[12] = (uint8_t) ((raw_speeds[1] >> 8) & 0xFF);

    uint32_t checksum = 0;
    for (int i = 2; i < 13; i++)
        checksum += packet[i];
    packet[13] = (uint8_t) (~checksum & 0xFF);

    HAL_UART_Transmit_IT(&huart4, packet, 14);
}

void move_AX_Wheels_Sync(uint8_t id1, float speed1, uint8_t id2, float speed2) {
	float speeds[2] = { speed1, speed2 };
	uint8_t ids[2] = { id1, id2 };
	uint16_t raw_speeds[2];

	for (int i = 0; i < 2; i++) {
		if (speeds[i] > 100.0f)
			speeds[i] = 100.0f;
		if (speeds[i] < -100.0f)
			speeds[i] = -100.0f;

		if (speeds[i] >= 0) {
			raw_speeds[i] = (uint16_t) ((speeds[i] / 100.0f) * 1023.0f);
		} else {
			raw_speeds[i] = (uint16_t) ((-speeds[i] / 100.0f) * 1023.0f);
			raw_speeds[i] |= 0x400;
		}
	}

	static uint8_t packet[14];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0xFE;
	packet[3] = 0x0A;
	packet[4] = 0x83;
	packet[5] = 0x20;
	packet[6] = 0x02;

	// Motor 1 Data
	packet[7] = ids[0];
	packet[8] = (uint8_t) (raw_speeds[0] & 0xFF);
	packet[9] = (uint8_t) ((raw_speeds[0] >> 8) & 0xFF);

	// Motor 2 Data
	packet[10] = ids[1];
	packet[11] = (uint8_t) (raw_speeds[1] & 0xFF);
	packet[12] = (uint8_t) ((raw_speeds[1] >> 8) & 0xFF);

	uint32_t checksum = 0;
	for (int i = 2; i < 13; i++)
		checksum += packet[i];
	packet[13] = (uint8_t) (~(checksum) & 0xFF);

	HAL_UART_Transmit_IT(&huart5, packet, 14);
}

void move_wheel(uint8_t id, float speed)
{
    static uint8_t packet[9];

    uint16_t speed_val = (uint16_t) fabs(speed);
    if (speed_val > 10000)
        speed_val = 10000;

    if (speed_val < -10000)
        speed_val = -10000;

    uint16_t raw_speed = speed_val;
    if (speed < 0)
        raw_speed |= 0x8000;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x05;
    packet[4] = 0x03;
    packet[5] = 0x2E;
    packet[6] = (uint8_t)(raw_speed & 0xFF);
    packet[7] = (uint8_t)((raw_speed >> 8) & 0xFF);

    uint32_t checksum = 0;
    for (int i = 2; i < 8; i++)
        checksum += packet[i];
    packet[8] = (uint8_t) (~(checksum) & 0xFF);

    HAL_UART_Transmit_IT(&huart4, packet, 9);
}
void move_AX_Servo_Sync(uint8_t id1, float deg1, uint8_t id2, float deg2,
		float speed_percent) {
	float degrees[2] = { deg1, deg2 };
	uint8_t ids[2] = { id1, id2 };
	uint16_t raw_pos[2];

	if (speed_percent < 0.0f)
		speed_percent = 0.0f;

	if (speed_percent > 100.0f)
		speed_percent = 100.0f; // saturacija
	uint16_t raw_speed = (uint16_t) ((speed_percent / 100.0f) * 1023.0f);
	if (raw_speed == 0 && speed_percent > 0)
		raw_speed = 1;

	for (int i = 0; i < 2; i++) {
		if (degrees[i] < 0.0f)
			degrees[i] = 0.0f;
		if (degrees[i] > 300.0f)
			degrees[i] = 300.0f;
		raw_pos[i] = (uint16_t) ((degrees[i] / 300.0f) * 1023.0f);
	}

	static uint8_t packet[18];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0xFE;
	packet[3] = 0x0E;
	packet[4] = 0x83;
	packet[5] = 0x1E;
	packet[6] = 0x04;

	packet[7] = ids[0];
	packet[8] = (uint8_t) (raw_pos[0] & 0xFF);
	packet[9] = (uint8_t) ((raw_pos[0] >> 8) & 0xFF);
	packet[10] = (uint8_t) (raw_speed & 0xFF);
	packet[11] = (uint8_t) ((raw_speed >> 8) & 0xFF);

	packet[12] = ids[1];
	packet[13] = (uint8_t) (raw_pos[1] & 0xFF);
	packet[14] = (uint8_t) ((raw_pos[1] >> 8) & 0xFF);
	packet[15] = (uint8_t) (raw_speed & 0xFF);
	packet[16] = (uint8_t) ((raw_speed >> 8) & 0xFF);

	uint32_t checksum = 0;
	for (int i = 2; i < 17; i++)
		checksum += packet[i];
	packet[17] = (uint8_t) (~(checksum) & 0xFF);

	HAL_UART_Transmit_IT(&huart5, packet, 18);
}



