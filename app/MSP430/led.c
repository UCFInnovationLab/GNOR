/*
 * led.c
 *
 *  Created on: Mar 20, 2016
 *      Author: harper
 */

#include "led.h"
#include "driverlib.h"

void blink_sensorhub_led() {
	static int flag=0;

	if (flag==0) {
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2);
		flag=1;
	} else {
	    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2);
		flag=0;
	}
}

