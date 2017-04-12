/*
 * pwm.c
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

#include "pwm.h"
#include "driverlib.h"

#define TIMER_PERIOD 511
#define DUTY_CYCLE  1

// P1.4 (TA0.3)
// P1.5 (TA0.4)
void pwm_init(void)
{
    //P1.5 as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(
      GPIO_PORT_P1,
      GPIO_PIN5
    );

    //Generate PWM - Timer runs in Up mode
    Timer_A_outputPWMParam tparam = {0};
    tparam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    tparam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    tparam.timerPeriod = TIMER_PERIOD;
    tparam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    tparam.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    tparam.dutyCycle = DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A0_BASE, &tparam);
}

// Set PWM duty cycle.
// duty: max value 512
//
void set_pwm_duty_cycle_1(uint16_t duty) {
    HWREG16(TIMER_A0_BASE + TIMER_A_CAPTURECOMPARE_REGISTER_4 + OFS_TAxR) = duty;
}

