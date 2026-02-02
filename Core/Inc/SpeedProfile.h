/*
 * Move.h
 *
 *  Created on: Dec 23, 2025
 *      Author: root
 */

#ifndef LIBRARIES_LOGIC_MOVEMENT_SPEEDPROFILE_H_
#define LIBRARIES_LOGIC_MOVEMENT_SPEEDPROFILE_H_
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "odometry.h"
#include "PID.h"
float calculate_trapezoid(float max_speed, float acceleration, float current_x, float current_y, float x_goal, float y_goal, uint8_t *movement_phase);
float calculate_angular_trapezoid(float max_ang_vel, float ang_accel, float current_fi, float target_fi, uint8_t *movement_phase);
void reset_move_profiles();
float get_remaining();

#endif /* LIBRARIES_LOGIC_MOVEMENT_SPEEDPROFILE_H_ */
