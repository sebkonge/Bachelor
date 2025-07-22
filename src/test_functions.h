#ifndef TEST_FUNCTIONS_H
#define TEST_FUNCTIONS_H

#include "motor_controller.h"
#include "IMU_1.1.h"
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

bool kbhit();
void test_PID_controller(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, float &current_angle, float control_signal_x, float Kp, float Ki, float Kd);
void test_50_100_speed(float control_signal_x, int throttle_percent1, int throttle_percent2);
void test_speed(float control_signal_x, int throttle_percent);
void test_IMU_sensor(Accel &accel, Gyro &gyro, DataVal &dataVal, int datasets);
void programming_ESC();


#endif