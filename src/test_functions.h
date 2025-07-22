#ifndef TEST_FUNCTIONS_H
#define TEST_FUNCTIONS_H

#include "motor_controller.h"
#include "IMU_1.1.h"
#include <iostream>
#include <fstream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

bool kbhit();
void test_PID_controller(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid, sdInfo &sdInfo, std::string pid_axis, char motor_axis, float &current_angle, float control_signal, float Kp, float Ki, float Kd, int datasets);
void test_50_100_speed(float control_signal, char motor_axis, int throttle_percent1, int throttle_percent2);
void test_speed(float control_signal, char motor_axis, int throttle_percent);
void turn_on_motors();
void test_IMU_sensor(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo, int datasets);
void programming_ESC(char motor_axis);
void test_3_range(char motor_axis, float control_signal, int speedtest1, int speedtest2, int speedtest3);
void test_functions(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo, std::string pid_axis, char motor_axis, float &current_angle, float control_signal, float Kp, float Ki, float Kd, int datasets, int speedtest, int speedtest1, int speedtest2, int speedtest3);


#endif