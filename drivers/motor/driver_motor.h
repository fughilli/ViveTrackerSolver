/*
 * motor.h
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t
	servo_max_highband,
	servo_min_highband;

void motor_init();
void motor_setSpeedi(uint32_t speed);
void motor_setSpeedf(float speed);
uint32_t motor_getMaxSpeed();
void motor_brake();
void motor_releaseBrake();

void servo_setPosf(float pos);
void servo_setPosi(uint32_t pos);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
