/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      mllite_test.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */
#include <BCUart.h>
#include <imu.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#include "F5xx_F6xx_Core_Lib/HAL_FLASH.h"

#include "msp430.h"
#include "msp430_clock.h"
#include "msp430_i2c.h"
#include "msp430_interrupt.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "driverlib.h"
#include "servo.h"
#include "imu.h"
#include "led.h"
#include "button.h"
#include "boat.h"
#include "BMP180.h"

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

#define BMP180_ADDRESS   0x77
#define BMP180_WHO_AM_I  0xD0  // WHO_AM_I id of BMP180, should return 0x55
#define BMP180_RESET     0xE0
#define BMP180_CONTROL   0xF4
#define BMP180_OUT_MSB   0xF6
#define BMP180_OUT_LSB   0xF7
#define BMP180_OUT_XLSB  0xF8

enum OSS {  // BMP-085 sampling rate
  OSS_0 = 0,  // 4.5 ms conversion time
  OSS_1,      // 7.5
  OSS_2,      // 13.5
  OSS_3       // 25.5
};

// Specify sensor parameters
uint8_t OSS = OSS_3;           // maximum pressure resolution

// These are constants used to calulate the temperature and pressure from the BMP-085 sensor
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md, b5;
uint16_t ac4, ac5, ac6;
float temperature, pressure, temppress, altitude;
uint32_t delt_t, count, tempcount;

extern struct hal_s hal;

void init_imu();
extern void read_from_mpl();

void BMP180Calibration();
int16_t BMP180GetTemperature();
long BMP180GetPressure();

long up = 0;

main() {
    unsigned long timestamp;
    unsigned char new_temp = 0;
    
#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
#endif

    bcUartInit();          // Init the back-channel UART
    init_imu();
    //BMP180Calibration();
    if (!BMP085_begin(BMP085_MODE_STANDARD)) {
    	printf("BMP085 Begin Error\n");
    }

    msp430_get_clock_ms(&timestamp);

    //array for data
    unsigned char arr[1];
    int val;
    val=msp430_i2c_read(0x77,0xD0,1,arr);
    printf("Read from i2c 0x77, register 0xD0. return = %d register = %x\n",val,arr[0]);

    MPL_LOGE("Starting Program.\n");
    
    while (1) {
        unsigned long sensor_timestamp;

        int new_data = 0;

        msp430_get_clock_ms(&timestamp);
        //printf("The time is now %d\n", timestamp);

#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif
        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

        /*
         * Go to sleep if we are watching sensors and the fifo is empty
         */
        if (!hal.sensors || !hal.new_gyro) {
            /* Put the MSP430 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */
            __bis_SR_register(LPM0_bits + GIE);
            continue;
        }

        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
            	//gyro[0] = (long)accel_short[0];
            	//gyro[1] = (long)accel_short[1];
            	//gyro[2] = (long)accel_short[2];
            	/*float gx,gy,gz;
            	gx = (float)gyro[0] / 16.375;
            	gy = (float)gyro[1] / 16.375;
            	gz = (float)gyro[2] / 16.375;
            	printf("%1.3f, %1.3f, %1.3f\n", gx, gy, gz);*/
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            //FILE* ftp = fopen("acceldata2.txt", "a");
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                /*float x,y,z;
                x = (float)accel[0] / 16384.0;
                y = (float)accel[1] / 16384.0;
                z = (float)accel[2] / 16384.0;
                printf("%1.3f, %1.3f, %1.3f\n", x, y, z);*/


                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            //fclose(ftp);
            if (sensors & INV_WXYZ_QUAT) {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } 
#ifdef COMPASS_ENABLED
        if (new_compass) {
            short compass_short[3];
            long compass[3];
            new_compass = 0;
            /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
             * magnetometer registers are copied to special gyro registers.
             */
            if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                compass[0] = (long)compass_short[0];
                compass[1] = (long)compass_short[1];
                compass[2] = (long)compass_short[2];
                /* NOTE: If using a third-party compass calibration library,
                 * pass in the compass data in uT * 2^16 and set the second
                 * parameter to INV_CALIBRATED | acc, where acc is the
                 * accuracy from 0 to 3.
                 */
                inv_build_compass(compass, 0, sensor_timestamp);
            }
            new_data = 1;
        }
#endif
        if (new_data) {
            if(inv_execute_on_data()) {
                MPL_LOGE("ERROR execute on data\n");
            }

            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
             * between new and stale data.
             */
            long data[9];
            int8_t accuracy;
            if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
                if (inv_get_sensor_type_heading(data, &accuracy,(inv_time_t*)&timestamp)) {
                    double heading = data[0] * 1.0 / (double)((long)1<<16);
                    //printf("%1.2f\t",heading);
                    boat_loop(timestamp, heading);

                }
                if(inv_get_sensor_type_euler(data, &accuracy,(inv_time_t*)&timestamp)){
                	double pitch = data[0]/(double)((long)1<<16);
                	double roll = data[1]/(double)((long)1<<16);
                	double yaw = data[2]/(double)((long)1<<16);
                	//printf("%1.2f %1.2f %1.2f\n",pitch, roll, yaw);
                }
            }
        }

#define PRESSURE_READ_MS 1000
        unsigned long next_pressure_ms = 0;

        if (timestamp > next_pressure_ms) {
            next_pressure_ms = timestamp + PRESSURE_READ_MS;

            float temperature;
            float pressure;
            float altitude;
            getTemperature(&temperature);
            getPressure(&pressure);
            altitude = pressureToAltitude(102317, pressure);

            printf("Temp = %f, Pressure = %f, Altitude = %f m, Altitude = %f ft\n", temperature, pressure, altitude,(altitude*3.281));
//            // Average over the display duty cycle to get the best pressure and altitude resolution
//            temperature = (float)BMP180GetTemperature()/10.;  // Get temperature from BMP-180 in degrees C
//            temppress += (float)BMP180GetPressure();          // Get pressure from BMP-180 in Pa
//            //temppress *= 1000;
//            tempcount++;
//            printf("%f\n", temperature);
//            printf("%f\n", temppress);
//
//            // Serial print and/or display at 0.5 s rate independent of data rates
//            //delt_t = millis() - count;
//            // update LCD once per half-second independent of read rate
//
//
//            pressure = temppress/tempcount;  // use average pressure for reading to get ultra-high resolution
//            temperature = temperature*9./5. + 32.;                          // convert to Fahrenheit
//            altitude = 44330.*( 1. - pow((pressure/102800.0), (1./5.255))); // Calculate altitude in meters
//            printf("The altitude is %f \n temperature = %f\n", altitude, temperature);
        }

    }	// while
}


void BMP180Calibration(){

  unsigned char t1[1];
  unsigned char t2[1];
  msp430_i2c_read(BMP180_ADDRESS, 0xAA,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xAB,1,t2); ac1 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xAC,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xAD,1,t2); ac2 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xAE,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xAF,1,t2); ac3 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xB0,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xB1,1,t2); ac4 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xB2,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xB3,1,t2); ac5 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xB4,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xB5,1,t2); ac6 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xB6,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xB7,1,t2); b1 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xB8,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xB9,1,t2); b2 = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xBA,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xBB,1,t2); mb = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xBC,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xBD,1,t2); mc = t1[0] << 8 | t2[0];
  msp430_i2c_read(BMP180_ADDRESS, 0xBE,1,t1); msp430_i2c_read(BMP180_ADDRESS, 0xBF,1,t2); md = t1[0] << 8 | t2[0];

  printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md);
}

  // Temperature returned will be in units of 0.1 deg C
  int16_t BMP180GetTemperature()
  {
	  const unsigned char temp[1] = {0x2E};

	  int16_t ut = 0;
	  msp430_i2c_pressurewrite(BMP180_ADDRESS, BMP180_CONTROL,1, temp); // start temperature measurement
	  msp430_delay_ms(50);
	  uint8_t rawData[2] = {0, 0};
	  msp430_i2c_read(BMP180_ADDRESS, 0xF6, 2, rawData); // read raw temperature measurement
	  ut = (int16_t)(((int16_t) rawData[0] << 8) | rawData[1]);

	  long x1, x2;

	  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
	  x2 = ((long)mc << 11)/(x1 + md);
	  b5 = x1 + x2;

	  return  ((b5 + 8)>>4);
}

// Calculate pressure read calibration values
// b5 is also required so BMP180GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
long BMP180GetPressure()
{
	long up = 0;
	const unsigned char tempp[1]={0x34 | OSS << 6};
	uint8_t rawData[3] = {0, 0, 0};
	long x1, x2, x3, b3, b6, p;
	unsigned long b4, b7;

	msp430_i2c_pressurewrite(BMP180_ADDRESS, BMP180_CONTROL,1, tempp); // Configure pressure measurement for highest resolution
	//delay(5 + 7*OSS); // delay 5 ms at lowest resolution, 26 ms at highest
	msp430_delay_ms(50);

	msp430_i2c_read(BMP180_ADDRESS, BMP180_OUT_MSB, 3, rawData); // read raw pressure measurement of 19 bits
	up = (((long) rawData[0] << 16) | ((long)rawData[1] << 8) | rawData[2]);
	up = up >> (8 - OSS);


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * ((b6 * b6)>>12))>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += ((x1 + x2 + 3791)>>4);

  return p;
}
