/*
 * button.c
 *
 *  Created on: Mar 27, 2016
 *      Author: harper
 */
#include "driverlib.h"

void button_init()
{
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);	// LaunchPad Button2
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);	// LaunchPad Button1

    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN3);		// SernorHub push button 1
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN4);		// SernorHub push button 2
}

int launchpad_button_1_pressed()
{
	if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == GPIO_INPUT_PIN_LOW)
		return 1;
	else
		return 0;
}

int launchpad_button_2_pressed()
{
	if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == GPIO_INPUT_PIN_LOW)
		return 1;
	else
		return 0;
}

int sensorhub_button_1_pressed()
{
	if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN3) == GPIO_INPUT_PIN_LOW)
		return 1;
	else
		return 0;
}

int sensorhub_button_2_pressed()
{
	if (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN4) == GPIO_INPUT_PIN_LOW)
		return 1;
	else
		return 0;
}


