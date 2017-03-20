/*
 * motor.c
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#include "driver_motor.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"

#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#define min(_val1_, _val2_) (((_val1_) < (_val2_)) ? (_val1_) : (_val2_))
#define max(_val1_, _val2_) (((_val1_) > (_val2_)) ? (_val1_) : (_val2_))
#define clamp(_val_, _min_, _max_) (max((_min_), min((_val_), (_max_))))

uint32_t motor_PWMClockFreq;

/********************************************
 * 			MOTOR CONTROL CONSTANTS			*
 ********************************************
 */

// The clock divider for the PWM module
#define MOTOR_CLOCK_DIV (32)
#define MOTOR_SYSCTL_CLOCK_DIV (SYSCTL_PWMDIV_32)

#define MOTOR_PWM_FREQ (1000)

uint32_t
	motor_period,
	motor_highband;

bool motor_braking;

/********************************************
 * 			SERVO CONTROL CONSTANTS			*
 ********************************************
 */
#define SERVO_PWM_FREQ (50ul)

#define SERVO_PERIOD_NS (1000000ul/SERVO_PWM_FREQ)
#define SERVO_MAX_BAND_NS (2500ul)
#define SERVO_MIN_BAND_NS (1000ul)
#define SERVO_MID_BAND_NS ((SERVO_MAX_BAND_NS + SERVO_MIN_BAND_NS)/2)

#define SERVO_OVERRIDE_BOUNDS

#define SERVO_OVERRIDE_SMAXB (2400)
#define SERVO_OVERRIDE_SMINB (1000)

uint32_t
	servo_period,
	servo_highband,
	servo_max_highband,
	servo_min_highband;

void servo_init();

void motor_init()
{
	// Calculate the PWM clock frequency
	motor_PWMClockFreq = SysCtlClockGet() / MOTOR_CLOCK_DIV;

	// Set the clock source divider for the PWM module
	SysCtlPWMClockSet(MOTOR_SYSCTL_CLOCK_DIV);

	// Enable the PWM peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	// Enable the GPIO port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure PD0 as the PWM output for the drive motor
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);

	// Configure PD2 as the brake pin (fast stop)
	motor_braking = false;
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

	// Calculate and set the motor period
	motor_period = (motor_PWMClockFreq / MOTOR_PWM_FREQ) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, motor_period);

	// Calculate and set the motor highband
	motor_highband = 1;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, motor_highband);

	// Enable the PWM output
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	motor_setSpeedi(0);

	// Chain initializer for servo
	servo_init();
}

void servo_init()
{
	// Enable the GPIO port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Configure PD1 as the PWM output for the steering servo
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);

	// Calculate and set the servo period
	servo_period = (motor_PWMClockFreq / SERVO_PWM_FREQ) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, servo_period);

#ifndef SERVO_OVERRIDE_BOUNDS
	// Calculate and set the servo highband
	servo_max_highband = ((motor_PWMClockFreq / SERVO_PWM_FREQ)
			* (SERVO_MAX_BAND_NS / 10ul)) / (SERVO_PERIOD_NS / 10ul);
	servo_min_highband = ((motor_PWMClockFreq / SERVO_PWM_FREQ)
			* (SERVO_MIN_BAND_NS / 10ul)) / (SERVO_PERIOD_NS / 10ul);

#else
	servo_max_highband = SERVO_OVERRIDE_SMAXB;
	servo_min_highband = SERVO_OVERRIDE_SMINB;
#endif

	servo_highband = (servo_max_highband + servo_min_highband)/2;

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, servo_highband);

	// Enable the PWM output
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void motor_setSpeedi(uint32_t speed)
{
	if (motor_braking)
	{
		motor_releaseBrake();
	}

	if(speed == 0 && motor_highband != 0)
	{
		motor_highband = 0;
		PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
		return;
	}

	if(motor_highband == 0)
	{
		PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	}

	motor_highband = clamp(speed, 1, motor_period - 1);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, motor_highband);
}

void motor_setSpeedf(float speed)
{
	motor_setSpeedi(clamp((uint32_t )(motor_period * speed), 1, motor_period - 1));
}

uint32_t motor_getMaxSpeed()
{
	return motor_period;
}

void motor_brake()
{
	motor_highband = 1;
	motor_braking = true;
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void motor_releaseBrake()
{
	motor_braking = false;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
}

void servo_setPosf(float pos)
{
	servo_highband =
			clamp(
					(uint32_t)((servo_max_highband-servo_min_highband)*pos + servo_min_highband),
					servo_min_highband, servo_max_highband);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, servo_highband);
}

void servo_setPosi(uint32_t pos)
{
	servo_highband = clamp(pos, 0, servo_highband);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, servo_highband);
}
