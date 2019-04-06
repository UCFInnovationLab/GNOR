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

#define TIMER_PERIOD 15000
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

    //P1.4 as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(
      GPIO_PORT_P1,
      GPIO_PIN4
    );

    //Generate PWM - Timer runs in Up mode
    Timer_A_outputPWMParam tparam1 = {0};
    tparam1.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    tparam1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8;
    tparam1.timerPeriod = TIMER_PERIOD;
    tparam1.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    tparam1.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    tparam1.dutyCycle = DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A0_BASE, &tparam1);

    //Generate PWM - Timer runs in Up mode
    Timer_A_outputPWMParam tparam2 = {0};
    tparam2.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    tparam2.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_8;
    tparam2.timerPeriod = TIMER_PERIOD;
    tparam2.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    tparam2.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    tparam2.dutyCycle = DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A0_BASE, &tparam2);
}

// Set PWM duty cycle.
// duty: max value 512
//
void set_pwm_duty_cycle_1(double duty) {
    uint16_t duty_int;

    if (duty < 0.0) duty = 0.0;
    else if (duty > 1.0) duty = 1.0;

    duty_int = duty * TIMER_PERIOD;

    HWREG16(TIMER_A0_BASE + TIMER_A_CAPTURECOMPARE_REGISTER_3 + OFS_TAxR) = duty_int;
}


// Set PWM duty cycle.
// duty: max value 512
//
void set_pwm_duty_cycle_2(double duty) {
    uint16_t duty_int;

    if (duty < 0.0) duty = 0.0;
    else if (duty > 1.0) duty = 1.0;

    duty_int = duty * TIMER_PERIOD;

    HWREG16(TIMER_A0_BASE + TIMER_A_CAPTURECOMPARE_REGISTER_4 + OFS_TAxR) = duty_int;
}



