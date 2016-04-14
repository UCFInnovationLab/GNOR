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

/*
 * boat_loop
 * ----------------------------
 * This routine is called in the main loop at a rate of ~100 times/sec.
 * The current timestamp in milliseconds and the current heading (0-359) is passed in.
 * Note: must use static variables if you need a persistance between calls.
 */
void boat_loop(unsigned long timestamp, double heading) {

	// Static variables.  Keep their values between calls to "boat_loop"
    static double old_heading=0.0;			// used to calculate the delta heading
    static unsigned long start_time;		// actual time the boat started.  Used as an offset to calculate elapsed time
    static int start=0;						// has the boat started, 1 of started
    static double heading_zero;				// heading offset.  Used to zero heading when button is pressed
    
    unsigned long running_time;				// elapsed time since the mission started
    
    double P = 1.0;							// P constant from the PID algorithm
    double I = 0.01;						// I constant from the PID algorithm
    int target = 0;							// Current heading target that the boat should seek
    int error = 0;							// error between current heading and target heading
    int rudder = 0;							// calculated rudder angle
    
    double heading_rate;					// calcuated delta between current and last reading of heading

    //--------------------------------------------------------------------------------
    // pre-start
    //--------------------------------------------------------------------------------

    // calculate heading turn rate.  this can be used to give a measure of how fast the boat's heading is drifting when the
    // boat is still.  If good, stable rate should be less than .005.
    heading_rate = heading - old_heading;
    old_heading = heading;
    
    // if the heading rate is less than some constant then turn on the Green LED
    if (fabs(heading_rate) < .005)
        set_led2(1);
    else
        set_led2(0);

    // Reset heading if button 1 is pressed
    if (launchpad_button_1_pressed() == 1) {
        heading_zero = heading;
    }
    
    // Calcuate the corrected heading taking into account the heading_zero value
    heading = heading - heading_zero;
    if (heading > 180) 
        heading = heading - 360;
    else if (heading < 0.0)
        heading = heading + 360;

    // Turn on LED if heading +- 5 degrees of 0
    if (abs(calculateDifferenceBetweenAngles(heading, target)) < 5.0) {
		set_led1(1);
	} else {
		set_led1(0);
    }
    
    // check for boat start.
    if ((abs(calculateDifferenceBetweenAngles(heading, 90)) < 5.0) && (start==0)) {
        start = 1;
        start_time = timestamp;
    }

    // handle orange running LED
    // blinking: pre-start
    // solid: boat is being controlled by program
    if (start==1) 
        set_sensorhub_led(1);
    else
        blink_sensorhub_led();

    //--------------------------------------------------------------------------------
    // Main routine that runs after boat has started
    //--------------------------------------------------------------------------------
    if (start==1) {
        running_time = timestamp - start_time;			// calculate elapsed time
        
        // place timed events here
        if (running_time < 20000) 
            target = 0;
        else if ((running_time > 20000) && (running_time < 40000))
            target = 270;
        else if (running_time > 40000)
            target = 180;
        
        // PID routine
        // bigger P causes boat to more reaction in if there is heading errors
        error = calculateDifferenceBetweenAngles(heading, target);
        rudder = P * error;
    
        // set limits on rudder movement
        if (rudder > 90) rudder  = 90;
        if (rudder < -90) rudder = -90;

        // set servo angles in response to PID
        servo1_move_to_angle(90 - rudder);
        

        // Log debugging information
        // %lf: double
        // %f: float
        // %d: integer
        //MPL_LOGE("Heading: %lf, %lf\n", heading, heading_rate);
      
    }


}

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the difference between two angles in a 0-360 system
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
   int delta;

    delta = (angle1 - angle2 + 360) % 360;
       if (delta > 180) delta = delta - 360;

// private double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
// {
// 	double difference = secondAngle - firstAngle;
//  while (difference < -180) difference += 360;
//  while (difference > 180) difference -= 360;
//  return difference;
//  }

//       if(angle1 > angle2) delta = 360 - angle2 - angle1;
//       else delta = angle2 - angle1;
     return delta;
}



