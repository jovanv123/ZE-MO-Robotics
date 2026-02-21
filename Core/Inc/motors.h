/*
 * pwm.h
 *
 *  Created on: Dec 22, 2025
 *      Author: root
 */

#ifndef LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_
#define LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_
#include "board_config.h"
#include <stdbool.h>
#define PWM_MAX 1049

extern volatile bool ax_moving;
void PWM_Init(void);
void PWM_SetSpeed_Left(float speed);
void PWM_SetSpeed_Right(float speed);
void PWM_SetServo_Position(uint32_t servo_number, uint16_t angle);
void move_step_motors(float target);
void move_step_back(float target);
void move_AX(uint8_t id, float position, float speed);
void set_AX_WheelMode(uint8_t id, uint8_t enable);
//void set_AX_ServoMode(uint8_t id);
void move_AX_Wheels_Sync(uint8_t id1, float speed1, uint8_t id2, float speed2);
void move_AX_Servo_Sync(uint8_t id1, float deg1, uint8_t id2, float deg2, float speed_percent);
void move_AX_Wheels_Sync_Limited(uint8_t id1, float speed1, uint8_t id2, float speed2, float limit_percent);
void Stop_Motors(void);
void receive_message(uint8_t id, uint8_t* rx_dma_buffer);

#endif /* LIBRARIES_PERIPHERALS_PERIPHERALS_PWM_PWM_H_ */
