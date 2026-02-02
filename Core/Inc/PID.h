/* movement_phase = IDLE;
        re
 * PID.h
 *
 *  Created on: Dec 23, 2025
 *      Author: root
 */

#ifndef LIBRARIES_LOGIC_MOVEMENT_PID_H_
#define LIBRARIES_LOGIC_MOVEMENT_PID_H_

#include "motors.h"
#include "odometry.h"
#include "SpeedProfile.h"

void movement_PID(float v_ref, uint8_t *movement_phase, float acceleration, float target_x, float target_y, float target_fi, int8_t dir);
void speed_PID(float ref_speed, uint8_t movement_phase, float x_ref, float y_ref, float current_x, float current_y, float current_fi, float ref_fi, int8_t dir);
void reset_PID();

#endif /* LIBRARIES_LOGIC_MOVEMENT_PID_H_ */
