#ifndef IMU_H_
#define IMU_H_


struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

int calculateDifferenceBetweenAngles(int angle1, int angle2);
void run_self_test();
void save_calibration_data_to_flash();

//#define COMPASS_ENABLED

#endif /* IMU_H_ */
