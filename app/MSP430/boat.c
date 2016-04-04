/*
 * boat.c
 *
 *  Created on: Apr 4, 2016
 *      Author: harper
 */
#include "boat.h"
#include "led.h"
#include "servo.h"
#include "log.h"


void boat_loop(unsigned long timestamp, double heading) {

	static double old_heading=0.0;
	static unsigned long next_time = 0;
	static int servo_degrees=0;

	double heading_rate;

	heading_rate = heading - old_heading;
	old_heading = heading;

	MPL_LOGE("Heading: %lf, %lf\n", heading, heading_rate);

	// Turn on LED if heading +- 5 degrees of 0
	if (abs(calculateDifferenceBetweenAngles(heading, 0)) < 5.0) {
		set_led1(1);
	} else {
		set_led1(0);
	}

	if (timestamp>next_time) {

		servo_degrees = (servo_degrees + 10) % 180;
		servo1_move_to_angle(servo_degrees);
		//MPL_LOGE("Timestamp %ld\n", timestamp);
		next_time += 1000;
	}

}

/*
 * Return the difference between two angles in a 0-360 system
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
	int delta;

	delta = (angle1 - angle2 + 360) % 360;
	if (delta > 180) delta = delta - 360;

//	private double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
//	  {
//	        double difference = secondAngle - firstAngle;
//	        while (difference < -180) difference += 360;
//	        while (difference > 180) difference -= 360;
//	        return difference;
//	 }

//	if(angle1 > angle2) delta = 360 - angle2 - angle1;
//	else delta = angle2 - angle1;
	return delta;
}


