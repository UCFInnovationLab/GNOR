/*
 * servo.c
 *
 *  Created on: Mar 20, 2016
 *      Author: harper
 */

//#include "inc/hw_memmap.h"
//
//#ifdef __MSP430_HAS_TxA7__
//#include "timer_a.h"
//
//#include <assert.h>

#include "servo.h"
#include "driverlib.h"

// Connect the servo SIGNAL wire to P1.2 through a 1K resistor.

#define MCU_CLOCK			12000000
#define MCU_CLOCK_DIV		8
#define SERVO_CLOCK			1500000 // MCU_CLOCK / MCU_CLOCK_DIV
#define PWM_FREQUENCY		50		// In Hertz, ideally 50Hz.
#define TIMER_PERIOD 		30000	// SERVO_CLOCK/PWM_FREQUENCY

#define SERVO_STEPS			181		// Maximum amount of steps in degrees (180 is common)
#define SERVO_MIN			1200	// The minimum duty cycle (800us / 1000000) / (1/1500000)
#define SERVO_MAX			3300	// The maximum duty cycle (2200us / 1000000) / (1/1500000)

#define CENTER_DUTY_CYCLE  2250

unsigned int servo_lut[ SERVO_STEPS+1 ];

void servo_init(void)
{
    //P2.4 and p2.5 as PWM outputs
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P2,
        GPIO_PIN4
        );

    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P2,
        GPIO_PIN5
        );

    //Generate PWM - Timer runs in Up mode
    Timer_A_outputPWMParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8;
    param.timerPeriod = TIMER_PERIOD;
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = CENTER_DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A2_BASE, &param);

    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8;
    param.timerPeriod = TIMER_PERIOD;
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = CENTER_DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A2_BASE, &param);

	unsigned int servo_stepval, servo_stepnow;
	unsigned int i;

	// Calculate the step value and define the current step, defaults to minimum.
	servo_stepval 	= ( (SERVO_MAX - SERVO_MIN) / SERVO_STEPS );
	servo_stepnow	= SERVO_MIN;

	// Fill up the LUT
	for (i = 0; i < SERVO_STEPS; i++) {
		servo_stepnow += servo_stepval;
		servo_lut[i] = servo_stepnow;
	}
}

void Timer_A_change_duty_cycle(uint16_t baseAddress,
					uint16_t compareRegister,
                    uint16_t duty_cycle)
{
    HWREG16(baseAddress + compareRegister + OFS_TAxR) = duty_cycle;
}

void servo1_move_to_angle(int angle) {

    int pulse_width;

    // make sure we are within 0-180 degrees
    if (angle>180) angle = 180;
    if (angle<0) angle = 0;

    pulse_width = servo_lut[angle];

    Timer_A_change_duty_cycle(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,pulse_width);
}

void servo2_move_to_angle(int angle) {

	int pulse_width;

	// make sure we are within 0-180 degrees
	if (angle>180) angle = 180;
	if (angle<0) angle = 0;

	pulse_width = servo_lut[angle];

	Timer_A_change_duty_cycle(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,pulse_width);
}




