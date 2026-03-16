/*
 * Encoders.h
 *
 *  Created on: Dec 21, 2025
 *      Author: root
 */

#ifndef LIBRARIES_PERIPHERALS_TIMERS_ENCODERS_ENCODERS_H_
#define LIBRARIES_PERIPHERALS_TIMERS_ENCODERS_ENCODERS_H_
#include "board_config.h"
#include <stdint.h>
void encoder_init(void);
int32_t encoder_get_count_left_motor(void);
int32_t encoder_get_count_right_motor(void);
void encoder_reset_left(void);
void encoder_reset_right(void);

#endif /* LIBRARIES_PERIPHERALS_TIMERS_ENCODERS_ENCODERS_H_ */
