#include "SpeedProfile.h"


float rots1, rots2, s1, s2, s3, total_path, distance_traveled, x_current, y_current, x_beginning, y_beginning;
//static float profile_v = 0.0f;
//static float profile_a = 0.0f;
//static float jerk = 0.0f;
float peak_vel, peak_ang_vel;

static bool is_rotating = false;
static bool is_braking = false;
static bool is_moving = false;

float target_speed, target_v;
float start_x, start_y;
float start_fi;
float total_angle, path_so_far;
float remaining2;
extern volatile int8_t dir;


float calculate_trapezoid(float max_vel, float max_accel, float cx, float cy, float target_x, float target_y, uint8_t *movement_phase)
{
    if (!is_moving)
    {
        start_x = cx;
        start_y = cy;
        float dx = target_x - cx;
        float dy = target_y - cy;
        total_path = sqrtf(dx * dx + dy * dy);

        s1 = (max_vel * max_vel) / (2.0f * max_accel);
        if (2.0f * s1 > total_path) {
            s1 = total_path / 2.0f;
            s2 = 0.0f;
            peak_vel = sqrtf(2.0f * max_accel * s1);
        } else {
            s2 = total_path - 2.0f * s1;
            peak_vel = max_vel;
        }
        is_moving = true;
    }

    float dx_now = cx - start_x;
    float dy_now = cy - start_y;
    path_so_far = sqrtf(dx_now * dx_now + dy_now * dy_now);

    remaining2 = total_path - path_so_far;

    if (path_so_far >= total_path) {
            is_moving = false;
            *movement_phase = IDLE;
            reset_PID();
            reset_move_profiles();
            return 0.0f;
        }

    if (path_so_far < s1) {
        target_v = sqrtf(2.0f * max_accel * path_so_far) + 10.0f;
    } else if (path_so_far > (s1 + s2)) {

        target_v = sqrtf(pow(peak_vel, 2) - 2.0f * max_accel*(path_so_far - s1 - s2));
    } else {
        target_v = peak_vel;
    }

    if (target_v > peak_vel) target_v = peak_vel;

    return target_v;
}

float calculate_angular_trapezoid(float max_ang_vel, float ang_accel, float current_fi, float target_fi, uint8_t *movement_phase)
{
    if (!is_rotating)
    {
        start_fi = current_fi;
        float diff = target_fi - current_fi;
        while (diff > M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        total_angle = fabsf(diff);

        rots1 = (max_ang_vel * max_ang_vel) / (2.0f * ang_accel);
        if (2.0f * rots1 > total_angle) {
            rots1 = total_angle / 2.0f;
            rots2 = 0.0f;
            peak_ang_vel = sqrtf(2.0f * ang_accel * rots1);
        } else {
            rots2 = total_angle - 2.0f * rots1;
            peak_ang_vel = max_ang_vel;
        }
        is_rotating = true;
    }

    float current_diff = current_fi - start_fi;
    while (current_diff > M_PI) current_diff -= 2.0f * M_PI;
    while (current_diff < -M_PI) current_diff += 2.0f * M_PI;
    float angle_so_far = fabsf(current_diff);

    if (angle_so_far >= total_angle) {
        is_rotating = false;
        *movement_phase = TRANSLATION;
        reset_PID();
        reset_move_profiles();
        return 0.0f;
    }
    if (angle_so_far < rots1) {
    	target_speed = sqrtf((2.0f * ang_accel) * angle_so_far) + 10.0f;
    } else if (angle_so_far > (rots1 + rots2)) {

        target_speed = sqrtf(pow(peak_ang_vel, 2) - 2.0f * ang_accel*(angle_so_far - rots1 - rots2));
    } else {
        target_speed = peak_ang_vel;
    }

    if (target_speed > peak_ang_vel) target_speed = peak_ang_vel;

    return target_speed;
}

float get_remaining()
{
	return remaining2;
}

void reset_move_profiles() {
    is_rotating = false;
    is_moving = false;
    is_braking = false;
}
