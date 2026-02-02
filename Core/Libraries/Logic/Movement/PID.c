#include "PID.h"

float v_ref, v_target;
float ref_speed_l = 0.0f, err_speed_l = 0.0f, err_speed_l_previous = 0.0f, u_speed_l = 0.0f, Kp_l = 1.0f, Ki_l = 0.01f;
float ref_speed_r = 0.0f, err_speed_r = 0.0f, err_speed_r_previous = 0.0f, u_speed_r = 0.0f, Kp_r = 1.0f, Ki_r = 0.01f;
float err_fi = 0.0f, err_fi_previous = 0.0f;
float Kp_correction = 55.0f, Ki_correction = 0.01f, u_correction = 0.0f;
extern volatile float cx, cy, cfi, c_speedl, c_speedr;
float final_out_r = 0.0f, final_out_l = 0.0f;

void movement_PID(float v_ref, uint8_t *movement_phase, float acceleration, float target_x, float target_y, float ref_fi, int8_t dir)
{
    switch (*movement_phase)
    {
    case ROTATION:
        speed_PID(v_ref, ROTATION, target_x, target_y, cx, cy, cfi, ref_fi, dir);
        break;

    case TRANSLATION:
        speed_PID(v_ref, TRANSLATION, target_x, target_y, cx, cy, cfi, ref_fi, dir);
        break;

    case IDLE:
        speed_PID(0, IDLE, target_x, target_y, cx, cy, cfi, ref_fi, dir);
        break;
    }
}

void speed_PID(float ref_speed, uint8_t movement_phase, float x_ref, float y_ref, float current_x, float current_y, float current_fi, float ref_fi, int8_t dir)
{
    err_fi = ref_fi - current_fi;
    while (err_fi >  M_PI) err_fi -= 2.0f * M_PI;
    while (err_fi < -M_PI) err_fi += 2.0f * M_PI;

    float correction = 0;

    if (movement_phase == ROTATION)
    {
			if (err_fi < 0) {
				ref_speed_r = -ref_speed;
				ref_speed_l =  ref_speed;
			} else {
				ref_speed_r =  ref_speed;
				ref_speed_l = -ref_speed;
			}
    } else {
        if(movement_phase == TRANSLATION)
        {
        	correction = err_fi * Kp_correction;
        	if (get_remaining()<10.0f)
        	{
        		correction = 0.0f;
        	}
        }
        ref_speed_r = ref_speed + dir*correction;
        ref_speed_l = ref_speed - dir*correction;
    }

    if (movement_phase == TRANSLATION || movement_phase == IDLE)
    {
    	err_speed_r = ref_speed_r - dir*c_speedr;
        err_speed_l = ref_speed_l - dir*c_speedl;
    }
    else if (movement_phase == ROTATION)
    {
    	err_speed_r = ref_speed_r - c_speedr;
        err_speed_l = ref_speed_l - c_speedl;
    }

    float du_r = Kp_r * (err_speed_r - err_speed_r_previous) + Ki_r * err_speed_r;
    u_speed_r += du_r;
    u_speed_r = fminf(fmaxf(u_speed_r, -1000.0f), 1000.0f);
    err_speed_r_previous = err_speed_r;

    float du_l = Kp_l * (err_speed_l - err_speed_l_previous) + Ki_l * err_speed_l;
    u_speed_l += du_l;
    u_speed_l = fminf(fmaxf(u_speed_l, -1000.0f), 1000.0f);
    err_speed_l_previous = err_speed_l;

    err_fi_previous = err_fi;

    final_out_l = fminf(fmaxf(u_speed_l, -1000.0f), 1000.0f);
    final_out_r = fminf(fmaxf(u_speed_r, -1000.0f), 1000.0f);

    if (movement_phase == TRANSLATION) {
        PWM_SetSpeed_Left(dir * final_out_l);
        PWM_SetSpeed_Right(dir * final_out_r);
    } else if (movement_phase == ROTATION) {
        PWM_SetSpeed_Left(final_out_l);
        PWM_SetSpeed_Right(final_out_r);
    } else {
        PWM_SetSpeed_Left(dir*final_out_l);
        PWM_SetSpeed_Right(dir*final_out_r);
    }
}

void reset_PID() {
    u_speed_l = 0.0f;
    u_speed_r = 0.0f;
    err_speed_l_previous = 0.0f;
    err_speed_r_previous = 0.0f;
    final_out_l = 0.0f;
    final_out_r = 0.0f;
}
