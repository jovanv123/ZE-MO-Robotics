/*
 * pwm.h
 *
 *  Created on: Dec 22, 2025
 *      Author: root
 */

#ifndef LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_
#define LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_
#include "board_config.h"
#define PWM_MAX 1049


void PWM_Init(void);
void PWM_SetSpeed_Left(float speed);
void PWM_SetSpeed_Right(float speed);
void PWM_SetServo_Position(uint32_t servo_number, uint16_t angle);
void move_AX(uint8_t id, float position, float speed);
void Stop_Motors(void);

#endif /* LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_ */
