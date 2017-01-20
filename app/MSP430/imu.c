/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      mllite_test.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */
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
#include "BCUart.h"
#include "driverlib.h"
#include "servo.h"
#include "button.h"
#include "led.h"

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct hal_s hal = {0};

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0,-1}
};


#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif



static inline void msp430_reset(void)
{
    PMMCTL0 |= PMMSWPOR;
}


/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}



/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
     va_list args;
    int length;
    char buf[1000];

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
    case MPL_LOG_INFO:
    case MPL_LOG_WARN:
    case MPL_LOG_ERROR:
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }

    bcUartSend((uint8_t*)buf, length+1);
    va_end(args);

    return 0;
}


/* This snippet of code shows how to load and store calibration data from
 * the MPL. For the MSP430, the flash segment must be unlocked before
 * reading/writing and locked when no longer in use. When porting to a
 * different microcontroller, flash memory might be accessible at anytime,
 * or may not be available at all.
 */
void load_calibration_data_from_flash()
{
	size_t store_size;

	inv_get_mpl_state_size(&store_size);
	if (store_size > FLASH_SIZE) {
		MPL_LOGE("Calibration data exceeds available memory.\n");
	} else {
		FCTL3 = FWKEY;
		if(inv_load_mpl_states(FLASH_MEM_START, store_size))
			MPL_LOGE("Error Loading Cal Data.\n");
		FCTL3 = FWKEY + LOCK;
	}

	/* Let MPL know that contiguity was broken. */
	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();
	inv_compass_was_turned_off();
}

void save_calibration_data_to_flash()
{
	size_t store_size;

	inv_get_mpl_state_size(&store_size);
	if (store_size > FLASH_SIZE) {
		MPL_LOGE("Calibration data exceeds available memory.\n");
		return;
	} else {
		unsigned char mpl_states[100], tries = 5, erase_result;
		inv_save_mpl_states(mpl_states, store_size);
		while (tries--) {
			/* Multiple attempts to erase current data. */
			Flash_SegmentErase((uint16_t*)FLASH_MEM_START);
			erase_result = Flash_EraseCheck((uint16_t*)FLASH_MEM_START,
				store_size>>1);
			if (erase_result == FLASH_STATUS_OK)
				break;
		}
		if (erase_result == FLASH_STATUS_ERROR) {
			MPL_LOGE("Could not erase user page for calibration "
				"storage.\n");
		} else {
			FlashWrite_8(mpl_states, FLASH_MEM_START, store_size);
		}
	}

	/* Let MPL know that contiguity was broken. */
	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();
	inv_compass_was_turned_off();
}


void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);

    if (result == 0x7) {
		MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *=  2048.f; //convert to +-16G (bug fix from +-8G)
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                MPL_LOGE("Gyro failed.\n");
            if (!(result & 0x2))
                MPL_LOGE("Accel failed.\n");
            if (!(result & 0x4))
                MPL_LOGE("Compass failed.\n");
     }

}



/* Set up MSP430 peripherals. */
//
// P1.0 - LED1 LaunchPad (GND reference)
// P1.1 - Button2 LaunchPad (GND - no pullup)
// P1.2 - Timer Capture (TA0.1)
// P1.3 - Timer Capture (TA0.2)
// P1.4 - PWM Out (Timer TA0.3)
// P1.5 - PWM Out (Timer TA0.4)
// P2.0 - 9150 Int_Motion
// P2.1 - Button1 LaunchPad (GND - no pullup)
// P2.4 - PWM Out (Timer TA2.1)
// P2.5 - PWM Out (Timer TA2.2)
// P3.0 - i2c SDA
// P3.1 - i2c SCL
// P3.3 - UART UCA0_TXD
// P3.4 - UART UCA0_RXD
// P4.4 - UART UCA1_TXD (connected to Application USB COM port through emulator)
// P4.5 - UART UCA1_RXD (connected to Application USB COM port through emulator)
// P4.7 - LED2 LaunchPad (GND reference)
// P6.2 - LED SensorHub (GND reference)
// P6.3 - Push Button SensorHub 1 (GND - 10k pullup)
// P6.4 - Push Button SensorHub 2 (GND - 10k pullup)
//
static inline void platform_init(void)
{
	WDTCTL = WDTPW | WDTHOLD;
    SetVCore(2);
    msp430_clock_init(12000000L, 2);
    //msp430_reset();
    msp430_i2c_enable();
    msp430_int_init();

    led_init();
    button_init();
    servo_init();



}

void init_imu(void)
{
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned short compass_fsr;
#endif

    /* Set up MSP430 hardware. */
    platform_init();

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    int_param.cb = gyro_data_ready_cb;
    int_param.pin = INT_PIN_P20;
    int_param.lp_exit = INT_EXIT_LPM0;
    int_param.active_low = 1;
    result = mpu_init(&int_param);
    if (result) {
        MPL_LOGE("Could not initialize gyro.\n");
        msp430_reset();
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    result = inv_init_mpl();
    if (result) {
        MPL_LOGE("Could not initialize MPL.\n");
        msp430_reset();
    }

    // Check for selftest
	//
	int i;
	if (sensorhub_button_1_pressed()) {
		for (i=0;i<5;i++) {
			msp430_delay_ms(800);
			set_sensorhub_led(1);
			msp430_delay_ms(200);
			set_sensorhub_led(0);
		}

		set_sensorhub_led(1);
		run_self_test();
		save_calibration_data_to_flash();
		set_sensorhub_led(0);
	}

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

//    I think inv_got_compass_bias() in results_holder.c will tell you when the compass bias has been established. My guess is that if you have set 9-axis fusion enabled, then as soon as the compass bias has been established, the MPL will switch to 9-axis fusion.
//
//    I am not sure if “heading from gyro” gives 9-axis fusion before the compass is calibrated. 9-axis fusion is adding compass readings to the fusion to add a drift free reading into the mix. No compass, no drift free element.

//    Thanks for your response. I tried using inv_got_compass_bias(), but it always stays equal to zero, no matter how many figure-eights I do. I have called the following to enable 9-axis fusion and compass calibration:
//    inv_enable_quaternion();
//    inv_enable_9x_sensor_fusion();
//    inv_enable_vector_compass_cal();
//    inv_enable_magnetic_disturbance();

//    I’m back to looking into this issue of how to determine if the magnetometer is calibrated. I tried inv_got_compass_bias() and inv_get_compass_state() both functions always report zero, before or after some figure-eights.

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    /* inv_enable_fast_nomot(); */
     inv_enable_motion_no_motion();
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */
    inv_enable_heading_from_gyro(); // DON

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            MPL_LOGE("Not authorized.\n");
            msp430_delay_ms(5000);
        }
    }
    if (result) {
        MPL_LOGE("Could not start the MPL.\n");
        msp430_reset();
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* Compass reads are handled by scheduler. */
    msp430_get_clock_ms(&timestamp);

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    if (dmp_load_motion_driver_firmware()) {
            MPL_LOGE("Could not download DMP.\n");
            msp430_reset();
    }
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    inv_set_quat_sample_rate(1000000L / DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;



    __enable_interrupt();
}





