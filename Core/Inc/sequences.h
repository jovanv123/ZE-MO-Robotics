/*
 * sequences.h
 *
 *  Created on: Mar 3, 2026
 *      Author: root
 */

#ifndef INC_SEQUENCES_H_
#define INC_SEQUENCES_H_
#include "main.h"
#include "motors.h"

void init_robot(int time);
void pushers_on();
void pushers_off();
void rotators_on();
void rotators_off();
void storage_on();
void storage_off();
void leadscrew_closed(int time);
void leadscrew_opened(int time);
void plazma_on();
void plazma_off();
void steppers_up();

#endif /* INC_SEQUENCES_H_ */
