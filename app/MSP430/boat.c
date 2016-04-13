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
    static unsigned long start_time;
    static int start=0;
    static double heading_zero;
    
    unsigned long running_time;
    
    double P = 1.0;
    int target = 0;
    int error = 0;
    int rudder = 0;
    
    double heading_rate;

    heading_rate = heading - old_heading;
    old_heading = heading;
    
    // Reset heading if button 1 is pressed
    //
    if (launchpad_button_1_pressed() == 1) {
        heading_zero = heading;
    }
    
    heading = heading - heading_zero;
    if (heading > 180) 
        heading = heading - 360;
    else if (heading < 0.0)
        heading = heading + 360;
    
    if (fabs(heading_rate) < .005) 
        set_led2(1);
    else
        set_led2(0);
        
    if ((abs(calculateDifferenceBetweenAngles(heading, 90)) < 5.0) && (start==0)) {
        start = 1;
        start_time = timestamp;
    }

    if (start==1) 
        set_sensorhub_led(1);
    else
        blink_sensorhub_led();
        
    if (start==1) {
        running_time = timestamp - start_time;
        if (running_time < 20000) 
            target = 0;
        else if ((running_time > 20000) && (running_time < 40000))
            target = 270;
        else if (running_time > 40000)
            target = 180;
        
        
        error = calculateDifferenceBetweenAngles(heading, target);
        rudder = P * error;
    
        if (rudder > 90) rudder  = 90;
        if (rudder < -90) rudder = -90;
        servo1_move_to_angle(90 - rudder);
        

    
        //MPL_LOGE("Heading: %lf, %lf\n", heading, heading_rate);
      
    }
    
    // Turn on LED if heading +- 5 degrees of 0
      if (abs(calculateDifferenceBetweenAngles(heading, target)) < 5.0) {
            set_led1(1);
        } else {
            set_led1(0);
    }

}

/*
 * Return the difference between two angles in a 0-360 system
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
   int delta;

    delta = (angle1 - angle2 + 360) % 360;
       if (delta > 180) delta = delta - 360;

// private double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
//      {
//           double difference = secondAngle - firstAngle;
//                while (difference < -180) difference += 360;
//                while (difference > 180) difference -= 360;
//         return difference;
//  }

//        if(angle1 > angle2) delta = 360 - angle2 - angle1;
//       else delta = angle2 - angle1;
     return delta;
}



