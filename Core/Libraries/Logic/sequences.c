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

void rotators_storage()
{
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_STORAGE, BACK_ROTATOR_AX, BACK_ROTATOR_STORAGE, 100);
}

void rotators_mid()
{
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_MID, BACK_ROTATOR_AX, BACK_ROTATOR_MID, 100);
}

void rotators_off()
{
	move_AX_Servo_Sync(FRONT_ROTATOR_AX, FRONT_ROTATOR_OFF, BACK_ROTATOR_AX, BACK_ROTATOR_OFF, 100);
}

void storage_off()
{
	  move_Wheels_SyncTime(LEFT_STORAGE_AX, 100, RIGHT_STORAGE_AX, 100, 1500, AX);
}

void storage_on()
{
	  move_Wheels_SyncTime(LEFT_STORAGE_AX, -100, RIGHT_STORAGE_AX, -100, 1500, AX);
}

void leadscrew_closed(int time)
{
	int move_time = time;
	move_Wheels_SyncTime(1, -10000, 2, -10000, move_time, WAVESHARE);
}

void leadscrew_opened(int time)
{
	int move_time = time;
	move_Wheels_SyncTime(1, 10000, 2, 10000, move_time, WAVESHARE);
}

void plazma_on()
{
	move_AX_Servo_Sync(LEFT_PLAZMA_AX, LEFT_PLAZMA_ON, RIGHT_PLAZMA_AX, RIGHT_PLAZMA_ON, 100);
}

void plazma_off()
{
	move_AX_Servo_Sync(LEFT_PLAZMA_AX, LEFT_PLAZMA_OFF, RIGHT_PLAZMA_AX, RIGHT_PLAZMA_OFF, 100);
}

void steppers_up(float position)
{
	move_step_back(position);
	move_step_motors(position);
}

