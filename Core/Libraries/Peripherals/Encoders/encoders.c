#include "encoders.h"
volatile int32_t debug1, debug2;

void encoder_init(void)
{
	HAL_TIM_Encoder_Start(&ENC_LEFT_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENC_RIGHT_TIMER, TIM_CHANNEL_ALL);
	ENC_LEFT_TIMER.Instance->CNT = 0;
	ENC_RIGHT_TIMER.Instance->CNT = 0;
}
//Namestiti drugi enkoder + proveriti na nukleu
int32_t encoder_get_count_left_motor(void)
{
	int32_t count = ENC_LEFT_TIMER.Instance->CNT;
	debug1 = count;
	return debug1;
}

int32_t encoder_get_count_right_motor(void)
{
	int32_t count = ENC_RIGHT_TIMER.Instance->CNT;
	debug2 = count;
	return debug2;
}
