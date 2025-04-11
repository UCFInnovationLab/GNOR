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
#include "stdio.h"
#include "button.h"
#include "pwm.h"

unsigned long last_time = 0;     // last time through the loop

#define P 2.0
#define MOTOR_BASE_SPEED 90

/*
 * boat_loop
 * ----------------------------
 * This routine is called in the main loop at a rate of ~100 times/sec.
 * The current timestamp in milliseconds and the current heading (0-359) is passed in.
 * Note: must use static variables if you need a persistence between calls.
 */
void boat_loop(unsigned long timestamp, double heading) {

	// Static variables.  Keep their values between calls to "boat_loop"
    static double old_heading=0.0;			// used to calculate the delta heading
    static unsigned long start_time;		// actual time the boat started.  Used as an offset to calculate elapsed time
    static int start=-1;					// has the boat started, -1=not ready, 0=ready, 1=started
    static double heading_zero;				// heading offset.  Used to zero heading when button is pressed
    static int first_time=1;                // flag to run one time routines
    static int calibrate_time=1;            // Should we calibrate
    
    unsigned long running_time;				// elapsed time since the mission started
    
    int target = 0;							// Current heading target that the boat should seek
    int error = 0;							// error between current heading and target heading
    int rudder = 0;							// calculated rudder angle
    int diff = 0;                           // diff for dual motor drive
    
    double heading_rate;			            // Calculated delta between current and last reading of heading

    //--------------------------------------------------------------------------------
    // pre-start
    //--------------------------------------------------------------------------------
    if (calibrate_time == 1) {
        if (launchpad_button_2_pressed() == 1) {
            servo2_move_to_angle(180);
        }

        while (launchpad_button_2_pressed() == 1)  {

        }

        servo2_move_to_angle(0);

        calibrate_time =0;
    }
    // Run one time initialization routines.
    if (first_time ==1) {
        //--------------------------------------------------
        // Rudder & Brushless motor
        // Servo1: Rudder, Servo2: speed controller
        //--------------------------------------------------
        servo1_move_to_angle(90);   // set rudder straight = 90 degrees
        servo2_move_to_angle(0);    // start motor a zero speed. range 0-180
        //--------------------------------------------------
        // Dual brushless motors
        // Servo1: left motor, Servo2: right motor
        //--------------------------------------------------
            // servo1_move_to_angle(0);    // start left motor a zero speed. range 0-180
            // servo2_move_to_angle(0);    // start right motor a zero speed. range 0-180
        //--------------------------------------------------
        // Dual brush motors
        //--------------------------------------------------
            // set_pwm_duty_cycle_1(0.0);
            // set_pwm_duty_cycle_2(0.0);
        first_time = 0;
    }

    // calculate heading turn rate.  this can be used to give a measure of how fast the boat's heading is drifting when the
    // boat is still.  If good, stable rate should be less than .005.
    heading_rate = heading - old_heading;
    old_heading = heading;

    // print heading every .5 seconds
    if ((timestamp - last_time) > 500) {
        printf("Heading: %lf, %lf\n", heading, heading_rate);
        last_time = timestamp;
    }
    
    // if the heading rate is less than some constant then turn on the Green LED
    if (fabs(heading_rate) < .005)
        set_led2(1);
    else
        set_led2(0);

    // Restart when button 1 is pressed
    // - reset heading
    // - reset start
    // - reset first_time
    if ((launchpad_button_1_pressed() == 1) || (green_board_button_pressed()== 1)) {
        heading_zero = heading;
        start=0;
        first_time=1;
    }
    
    // Calculate the corrected heading taking into account the heading_zero value
    heading = heading - heading_zero;
    if (heading > 180) 
        heading = heading - 360;
    else if (heading < 0.0)
        heading = heading + 360;

    // check for boat start.  (currently rotate boat 90 degrees
    if (((calculateDifferenceBetweenAngles(heading, 90)) > 0.0) && (start==0)) {
        start = 1;
        start_time = timestamp;
        printf("Started\n");

        //
        // Start the boat's motor(s)
        //

        //--------------------------------------------------
        // Rudder & Brushless motor
        // Servo1: Rudder, Servo2: speed controller
        //--------------------------------------------------
        servo2_move_to_angle(MOTOR_BASE_SPEED);    // start motor. range 0-180
        //--------------------------------------------------
        // Dual brushless motors
        // Servo1: left motor, Servo2: right motor
        //--------------------------------------------------
            // servo1_move_to_angle(MOTOR_BASE_SPEED);    // start left motor at base speed. range 0-180
            // servo2_move_to_angle(MOTOR_BASE_SPEED);    // start right motor at base speed. range 0-180
        //--------------------------------------------------
        // Dual brush motors
        //--------------------------------------------------
            // set_pwm_duty_cycle_1(0.5);
            // set_pwm_duty_cycle_2(0.5);
    }

    // handle orange "running LED"
    // blinking: pre-start
    // solid: boat is being controlled by program
    if (start==1)  {
        set_sensorhub_led(1);
    } else {
        blink_sensorhub_led();
    }

    //--------------------------------------------------------------------------------
    // Main routine that runs after boat has started
    //--------------------------------------------------------------------------------
    if (start==1) {
        running_time = timestamp - start_time;			// calculate elapsed time
        
        // place timed events here
         if (running_time < 10000)
             target = 0;                                // run for 10s straight ahead
         else if (running_time < 20000)
             target = 270;                               // turn to 270 for 10s
         else
             target = 180;                              // turn to 270 for 10s
        
        // PID routine
        // bigger P causes boat to have more reaction to heading errors
        error = calculateDifferenceBetweenAngles(heading, target);


        //--------------------------------------------------
        // Rudder & Brushless motor
        // Servo1: Rudder, Servo2: speed controller
        //--------------------------------------------------
        rudder = P * error;
        // set limits on rudder movement
        if (rudder > 90) rudder  = 90;
        if (rudder < -90) rudder = -90;
        // set servo angles in response to PID
        servo1_move_to_angle(90 + rudder);              // if rudder is reversed, change + to -
        //--------------------------------------------------
        // Dual brushless motors
        // Servo1: left motor, Servo2: right motor
        //--------------------------------------------------
            // diff = P * error;
            // servo1_move_to_angle(MOTOR_BASE_SPEED + diff);
            // servo2_move_to_angle(MOTOR_BASE_SPEED + diff);
        //--------------------------------------------------
        // Dual brush motors
        //--------------------------------------------------
            //set_pwm_duty_cycle_1(diff);
            //set_pwm_duty_cycle_2(diff);
    }

    // Turn on LED if heading +- 5 degrees of the target
    if (abs(calculateDifferenceBetweenAngles(heading, target)) < 5.0) {
		set_led1(1);
	} else {
		set_led1(0);
    }

}

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the difference between two angles in a 0-360 system
 * - returns +-179
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
   int delta;

    delta = (angle1 - angle2 + 360) % 360;
       if (delta > 180) delta = delta - 360;

     return delta;
}



