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
 
  unsigned long running_time;
    
     double P = 1.0;
     int target = 0;
     int error = 0;
      int rudder = 0;
      double heading_rate;


    if ((abs(calculateDifferenceBetweenAngles(heading, 270)) < 5.0) && (start==0)) {
        start = 1;
        start_time = timestamp;
    }

    if (start==1) 
        set_led2(1);
    else
        set_led2(0);
        
    if (start==1) {
        running_time = timestamp - start_time;
        if (running_time < 20000) 
            target = 270;
        else if ((running_time > 20000) && (running_time < 40000))
            target = 180;
        else if (running_time > 40000)
            target = 90;
        
        
        
        error = calculateDifferenceBetweenAngles(heading, target);
        rudder = P * error;
    
        if (rudder > 90) rudder  = 90;
        if (rudder < -90) rudder = -90;
        servo1_move_to_angle(90 - rudder);
    
     heading_rate = heading - old_heading;
          old_heading = heading;
    
           //MPL_LOGE("Heading: %lf, %lf, %d, %d\n", heading, heading_rate, rudder, error);
    
      
    }
    
    // Turn on LED if heading +- 5 degrees of 0
      if (abs(calculateDifferenceBetweenAngles(heading, 0)) < 5.0) {
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



