#include "sequences.h"

void init_robot(int time)
{
	move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OFF, RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
	HAL_Delay(10);
	move_AX_Servo_Sync(LEFT_STORAGE_AX, 300, RIGHT_STORAGE_AX, 300, 100);
	HAL_Delay(10);
	move_AX_Servo_Sync(1,189,2,92,100);
	HAL_Delay(10);
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_OFF, BACK_ROTATOR_AX, BACK_ROTATOR_OFF, 100);
	HAL_Delay(10);
	move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, -100, RIGHT_LEADSCREW_AX, -100, time);
}

void pushers_on()
{
	move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_ON, RIGHT_PUSHER_AX, RIGHT_PUSHER_ON, 100);
}

void pushers_off()
{
	move_AX_Servo_Sync(LEFT_PUSHER_AX, LEFT_PUSHER_OFF, RIGHT_PUSHER_AX, RIGHT_PUSHER_OFF, 100);
}

void rotators_on()
{
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_ON, BACK_ROTATOR_AX, BACK_ROTATOR_ON, 100);
}

void rotators_off()
{
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_OFF, BACK_ROTATOR_AX, BACK_ROTATOR_OFF, 100);
}

void storage_on()
{
    move_AX_Wheels_SyncTime(LEFT_STORAGE_AX, -100, RIGHT_STORAGE_AX, -100, 2500);
}

void storage_off()
{
    move_AX_Wheels_SyncTime(LEFT_STORAGE_AX, 100, RIGHT_STORAGE_AX, 100, 2500);
}

void leadscrew_closed(int time)
{
	int move_time = time;
	move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, 100, RIGHT_LEADSCREW_AX, 100, move_time);
}

void leadscrew_opened(int time)
{
	int move_time = time;
	move_AX_Wheels_SyncTime(LEFT_LEADSCREW_AX, -100, RIGHT_LEADSCREW_AX, -100, move_time);
}

void plazma_on()
{
	move_AX_Servo_Sync(1,1,2,280,100);
}

void plazma_off()
{
	move_AX_Servo_Sync(1,189,2,92,100);
}

void steppers_up()
{
	move_step_back(90.0);
	move_step_motors(90.0);
}

