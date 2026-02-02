/*
 * board_config.h
 *
 *  Created on: Dec 21, 2025
 *      Author: root
 */

#ifndef INC_BOARD_CONFIG_H_
#define INC_BOARD_CONFIG_H_
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim5, htim4, htim3, htim2, htim1, htim10;
extern UART_HandleTypeDef huart6;
#define FRONT 1
#define BACK 2
#define TRANSLATION 2
#define ROTATION 1
#define IDLE 3
#define MAX_JERK  8000.0f
#define MAX_ACCEL 1000.0f
#define MAX_ANG_ACCEL 150000.0f
#define TRAPEZOID 1
#define S_CURVE 2
#define FORWARDS 1
#define BACKWARDS -1

	//Main Motors
	#define MAX_PHYSICAL_SPEED 1000.0f

	#define MOTOR_PWM_TIMER1 	htim4
	#define MOTOR_PWM_TIMER2 	htim3
	#define PWM_MAX_VALUE 		1049

	#define MOTOR_LEFT_DIR_PIN		GPIO_PIN_9
  	#define MOTOR_LEFT_DIR_PORT		GPIOA

	#define MOTOR_RIGHT_DIR_PIN     GPIO_PIN_8
	#define MOTOR_RIGHT_DIR_PORT    GPIOA

	//Step Motors (Lift)

	#define STEP_MOTOR_PWM_TIMER htim9

	#define STEP_MOTOR1_PWM_PIN
	#define STEP_MOTOR1_PWM_PORT

	#define STEP_MOTOR2_PWM_PIN
	#define STEP_MOTOR2_PWM_PORT

	//Servo Motors
	#define SERVO_TIMER		htim1

	//Encoder Wheels
	#define ENC_LEFT_TIMER 		htim2
	#define ENC_RIGHT_TIMER 	htim5
	#define ENCODER_TICKS_PER_REV 		2048

	//Sensors

	//Lifts

	//UART

	#define SYS_CLOCK_MHZ 84

#endif /* INC_BOARD_CONFIG_H_ */
