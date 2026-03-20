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
//	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_1, 500);
//	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_2, 500);
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_1); //Levi tocak
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, TIM_CHANNEL_2); //Desni tocak
//	HAL_TIM_PWM_Start(&SERVO_TIMER, TIM_CHANNEL_1);

//	__HAL_TIM_MOE_ENABLE(&htim1);
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

//void PWM_SetServo_Position(uint32_t servo_number, uint16_t angle) {
//	if (angle > 180)
//		angle = 180;
//	uint32_t angle_to_pwm = 500
//			+ (uint32_t) (angle * (2500.0f - 500.0f) / 180.0f);
//	switch (servo_number) {
//	case 1:
//		SERVO_TIMER.Instance->CCR1 = angle_to_pwm;
//		break;
//	case 2:
////		SERVO_TIMER.Instance?->CCR2 = angle_to_pwm
//		break;
//	case 3:
////		SERVO_TIMER.Instance->CCR3 = angle_to_pwm;
//		break;
//	case 4:
////		SERVO_TIMER.Instance->CCR4 = angle_to_pwm;
//		break;
//	default:
//		break;
//	}
//}
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
//
//void move_AX(uint8_t id, float degrees, float speed_percent) {
//
//	if (degrees < 0.0f)
//		degrees = 0.0f;
//	if (degrees > 300.0f)
//		degrees = 300.0f;
//	if (speed_percent < 0.0f)
//		speed_percent = 0.0f;
//	if (speed_percent > 100.0f)
//		speed_percent = 100.0f;
//
//	uint16_t raw_position = (uint16_t) ((degrees / 300.0f) * 1023.0f);
//	uint16_t raw_speed = (uint16_t) ((speed_percent / 100.0f) * 1023.0f);
//
//	if (raw_speed == 0 && speed_percent > 0)
//		raw_speed = 1;
//
//	uint8_t packet[11];
//	packet[0] = 0xFF;
//	packet[1] = 0xFF;
//	packet[2] = id;
//	packet[3] = 0x07;
//	packet[4] = 0x03;
//	packet[5] = 0x1E;
//
//	packet[6] = (uint8_t) (raw_position & 0xFF);
//	packet[7] = (uint8_t) ((raw_position >> 8) & 0xFF);
//
//	packet[8] = (uint8_t) (raw_speed & 0xFF);
//	packet[9] = (uint8_t) ((raw_speed >> 8) & 0xFF);
//
//	uint32_t checksum_sum = packet[2] + packet[3] + packet[4] + packet[5]
//			+ packet[6] + packet[7] + packet[8] + packet[9];
//	packet[10] = (uint8_t) (~(checksum_sum) & 0xFF);
//
//	HAL_UART_Transmit(&huart6, packet, 11, 100);
//}
//
void set_AX_WheelMode(uint8_t id, uint8_t enable) {
	uint16_t ccw_limit = enable ? 0 : 1023;
	uint8_t packet[9];
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = id;
	packet[3] = 0x05;
	packet[4] = 0x03;
	packet[5] = 0x08;
	packet[6] = (uint8_t) (ccw_limit & 0xFF);
	packet[7] = (uint8_t) ((ccw_limit >> 8) & 0xFF);

	uint32_t checksum = packet[2] + packet[3] + packet[4] + packet[5]
			+ packet[6] + packet[7];
	packet[8] = (uint8_t) (~(checksum) & 0xFF);

	HAL_UART_Transmit(&huart1, packet, 9, 100);
}

//void set_AX_ServoMode(uint8_t id)
//{
//    uint16_t ccw_limit = 1023;
//    uint8_t packet[9];
//    packet[0] = 0xFF;
//    packet[1] = 0xFF;
//    packet[2] = id;
//    packet[3] = 0x05;
//    packet[4] = 0x03;
//    packet[5] = 0x08;
//    packet[6] = (uint8_t)(ccw_limit & 0xFF);
//    packet[7] = (uint8_t)((ccw_limit >> 8) & 0xFF);
//
//    uint32_t checksum = packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7];
//    packet[8] = (uint8_t)(~(checksum) & 0xFF);
//
//    HAL_UART_Transmit(&huart6, packet, 9, 100);
//
//    HAL_Delay(5);
//}
//
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

	uint8_t packet[14];
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

	HAL_UART_Transmit(&huart1, packet, 14, 100);
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

	uint8_t packet[18];
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

	HAL_UART_Transmit(&huart1, packet, 18, 100);
}
//
//void move_AX_Wheels_Sync_Limited(uint8_t id1, float speed1, uint8_t id2,
//		float speed2, float limit_percent) {
//	float speeds[2] = { speed1, speed2 };
//	uint8_t ids[2] = { id1, id2 };
//	uint16_t raw_speeds[2];
//
//	// Scale Torque Limit (0 - 1023)
//	if (limit_percent > 100.0f)
//		limit_percent = 100.0f;
//	uint16_t raw_limit = (uint16_t) ((limit_percent / 100.0f) * 1023.0f);
//
//	for (int i = 0; i < 2; i++) {
//		if (speeds[i] > 100.0f)
//			speeds[i] = 100.0f;
//		if (speeds[i] < -100.0f)
//			speeds[i] = -100.0f;
//
//		if (speeds[i] >= 0) {
//			raw_speeds[i] = (uint16_t) ((speeds[i] / 100.0f) * 1023.0f);
//		} else {
//			raw_speeds[i] = (uint16_t) ((-speeds[i] / 100.0f) * 1023.0f);
//			raw_speeds[i] |= 0x400; // Bit 10 is direction for wheel mode
//		}
//	}
//
//	uint8_t packet[18];
//	packet[0] = 0xFF;
//	packet[1] = 0xFF;
//	packet[2] = 0xFE;  // Broadcast ID
//	packet[3] = 0x0E;  // Length: (4 bytes data + 1 ID) * 2 motors + 4
//	packet[4] = 0x83;  // Sync Write
//	packet[5] = 0x20;  // Start Address: 32 (Moving Speed)
//	packet[6] = 0x04;  // Data Length per motor: 2 (Speed) + 2 (Torque Limit)
//
//	// Motor 1
//	packet[7] = ids[0];
//	packet[8] = (uint8_t) (raw_speeds[0] & 0xFF);
//	packet[9] = (uint8_t) ((raw_speeds[0] >> 8) & 0xFF);
//	packet[10] = (uint8_t) (raw_limit & 0xFF);
//	packet[11] = (uint8_t) ((raw_limit >> 8) & 0xFF);
//
//	// Motor 2
//	packet[12] = ids[1];
//	packet[13] = (uint8_t) (raw_speeds[1] & 0xFF);
//	packet[14] = (uint8_t) ((raw_speeds[1] >> 8) & 0xFF);
//	packet[15] = (uint8_t) (raw_limit & 0xFF);
//	packet[16] = (uint8_t) ((raw_limit >> 8) & 0xFF);
//
//	uint32_t checksum = 0;
//	for (int i = 2; i < 17; i++)
//		checksum += packet[i];
//	packet[17] = (uint8_t) (~(checksum) & 0xFF);
//
//	HAL_UART_Transmit(&huart6, packet, 18, 100);
//}
//
//void receive_message(uint8_t id, uint8_t *rx_dma_buffer) {
//	uint8_t packet[8] = { 0xFF, 0xFF, id, 0x04, 0x02, 0x28, 0x02, 0x00 };
//
//	uint32_t checksum = packet[2] + packet[3] + packet[4] + packet[5]
//			+ packet[6];
//	packet[7] = (uint8_t) (~(checksum) & 0xFF);
//
//	__HAL_UART_CLEAR_OREFLAG(&huart6);
//	HAL_UART_Receive_DMA(&huart6, rx_dma_buffer, 8);
//
//	HAL_UART_Transmit(&huart6, packet, 8, 10);
//	HAL_HalfDuplex_EnableReceiver(&huart6);

//}


