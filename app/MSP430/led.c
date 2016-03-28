/*
 * led.c
 *
 *  Created on: Mar 20, 2016
 *      Author: harper
 */

#include "led.h"
#include "driverlib.h"

void led_init()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);		// LaunchPad LED1 OUTPUT
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);	// LaunchPad LED1 Off

    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);		// LaunchPad LED1 OUTPUT
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);	// LaunchPad LED1 off

    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN2);		// SensorHub LED OUTPUT
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2);	// SensorHub LED off
}

void blink_sensorhub_led()
{
	static int flag=0;

	if (flag==0) {
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2);
		flag=1;
	} else {
	    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2);
		flag=0;
	}
}

void set_led1(int flag)
{
	if (flag)
		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
	else
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void set_led2(int flag)
{
	if (flag)
		GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
	else
		GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
}

void set_sensorhub_led(int flag)
{
	if (flag)
		GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2);
	else
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2);
}

