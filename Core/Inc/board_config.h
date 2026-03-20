/*
 * board_config.h
 *
 *  Created on: Dec 21, 2025
 *      Author: root
 */

#ifndef INC_BOARD_CONFIG_H_
#define INC_BOARD_CONFIG_H_
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim5, htim4, htim7, htim3, htim2, htim1, htim9, htim10, htim11;
extern UART_HandleTypeDef huart1;

#define FRONT 1
#define BACK 2
#define TRANSLATION 2
#define ROTATION 1
#define SPIN 4
#define IDLE 3
#define MAX_JERK  8000.0f
#define MAX_ACCEL 200.0f
#define MAX_ANG_ACCEL 200.0f
#define TRAPEZOID 1
#define S_CURVE 2
#define FORWARDS 1
#define BACKWARDS -1

//Main Motors
#define MAX_PHYSICAL_SPEED 800.0f

#define MOTOR_PWM_TIMER 	htim3
#define PWM_MAX_VALUE 		2099

#define MOTOR_LEFT_DIR_PIN		GPIO_PIN_9
#define MOTOR_LEFT_DIR_PORT		GPIOA

#define MOTOR_RIGHT_DIR_PIN     GPIO_PIN_8
#define MOTOR_RIGHT_DIR_PORT    GPIOA

//Step Motors (Lift)

//#define STEP_MOTOR_PWM_TIMER htim1

#define STEP_MOTOR1_PWM_PIN GPIO_PIN_4
#define STEP_MOTOR1_PWM_PORT GPIOC

#define STEP_MOTOR2_PWM_PIN GPIO_PIN_5
#define STEP_MOTOR2_PWM_PORT GPIOC

//Servo Motors + AX servo motors

#define SERVO_TIMER		htim11

#define FRONT_ROTATOR_AX 21 // id ax-eva
#define BACK_ROTATOR_AX 20

#define RIGHT_PUSHER_AX 60
#define LEFT_PUSHER_AX 61

#define RIGHT_LEADSCREW_AX 51
#define LEFT_LEADSCREW_AX 50

#define RIGHT_STORAGE_AX 71
#define LEFT_STORAGE_AX 70

#define RIGHT_LEADSCREW_ON 0
#define RIGHT_LEADSCREW_OFF 140

#define LEFT_LEADSCREW_ON 150
#define LEFT_LEADSCREW_OFF 295

#define BACK_ROTATOR_ON 125
#define BACK_ROTATOR_OFF 60

#define FRONT_ROTATOR_ON 120
#define FRONT_ROTATOR_OFF 55

#define LEFT_PUSHER_OFF 130//142 //60
#define LEFT_PUSHER_ON 190 //217 //0

#define RIGHT_PUSHER_OFF 70//67 // 150
#define RIGHT_PUSHER_ON  10 //11  // 210

#define LEFT_STORAGE_ON 0
#define LEFT_STORAGE_OFF 300

#define RIGHT_STORAGE_ON 0
#define RIGHT_STORAGE_OFF 300
//Encoder Wheels
#define ENC_LEFT_TIMER 		htim1
#define ENC_RIGHT_TIMER 	htim2
#define ENCODER_TICKS_PER_REV 		2048

//Sensors

//Lifts

//UART

#define SYS_CLOCK_MHZ 84

#endif /* INC_BOARD_CONFIG_H_ */
