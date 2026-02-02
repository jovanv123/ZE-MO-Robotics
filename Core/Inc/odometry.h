/*
 * odometry.h
 *
 *  Created on: Dec 21, 2025
 *      Author: root
 */

#ifndef LIBRARIES_LOGIC_ODOMETRY_ODOMETRY_H_
#define LIBRARIES_LOGIC_ODOMETRY_ODOMETRY_H_

#include "encoders.h"
#include <math.h>

void odometry_init(float d_r, float d_l, float base);

void calculate_odometry();

float get_speed_r();

float get_speed_l();

float get_x();

float get_y();

float get_fi();

float get_w_r();

float get_w_l();

void set_x_y(float x_p, float y_p, float fi_p);

#endif /* LIBRARIES_LOGIC_ODOMETRY_ODOMETRY_H_ */
