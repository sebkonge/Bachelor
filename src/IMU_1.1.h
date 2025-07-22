#pragma once

#ifndef IMU_1_1_H
#define IMU_1_1_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
/*********************************** - I2C & IMU cofig - ***********************************/
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// I2C & IMU Configurations
#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9
#define IMU_ADDR 0x6A // Default I2C address for LSM6DS3
#define OUTX_L_XL 0x28 // Register for accelerometer X-axis low byte
#define OUTX_L_G 0x22 // Register for gyroscope X-axis low byte
#define OUT_TEMP_L 0x20 // Register for temperature data
#define WHO_AM_I_REG 0x0F // Register for WHO_AM_I value
#define WHO_AM_I_VALUE 0x69 // Value of WHO_AM_I register for LSM6DS3
#define I2C_BAUDRATE 400000// Initialize I2C at 400 kHz

//#define CTRL1_XL 0b10001100 // 1,66 kHz, ±8g, 400 Hz filter
// #define CTRL1_XL 0b10000100 // 1,66 kHz, ±16g, 400 Hz filter
#define CTRL1_XL 0b10000111 // 1,66 kHz, ±16g, 50 Hz filter

#define CTRL2_G 0b10001000 // 1,66 kHz, 1000 dps, disabled (FS mode selection)

//#define CTRL8_XL 0b11000000 // Add low pass filter to accelerometer (9 Hz cutoff) 1.66 kHz / 9 Hz = 184 Hz 

#define RAD_TO_DEG 57.2957795131 // Conversion factor for rad to deg
#define ROLL 0
#define PITCH 1
#define YAW 2
#define X 0
#define Y 1
#define Z 2

#define MIN_PULSE 1000          // 1.0 ms pulse (0% throttle)
#define MAX_PULSE 2000          // 2.0 ms pulse (100% throttle)

struct Accel {
    float acc_xyz[3];
    float acc_ang[2];
    float vel_xyz[3];
    float pos_xyz[3];
    float prev_vel_xyz[3] = {0.f, 0.f, 0.f};
    float prev_pos_xyz[3] = {0.f, 0.f, 0.f};
};

struct Gyro {
    float axis[3];
    float ang_vel_xyz[3];
    float ang_pos_xyz[3] = {0.f, 0.f, 0.f};
    float prev_ang_xyz[3] = {0.f, 0.f, 0.f};
};

struct DataVal {
    int16_t raw_gyro_xyz[3];
    int16_t raw_accel_xyz[3];
    uint32_t timestamp;
    float prev_timestamp_seconds = 0.f;
};

struct PID{
    float target_angle = 0.f;
    float error = 0.f;
    float prev_error = 0.f;
    float integral = 0.f;
    float derivative = 0.f;
    float prev_timestamp_seconds_pid = 0.f;
    bool regulator_on = false;
    float output = 0.f;
    std::string pid_axis;
};

struct sdInfo{
    std::string InfoString;
    uint32_t timestamp;   
    float ZAcceleration;
};

void resetIMU();
void lsm6ds3_init();
void configure_LSM6DS3();
void readTimestamp();
void resetTimestamp();
float PID_regulator(DataVal &dataVal, PID &pid, std::string pid_axis, float &current_angle, float Kp, float Ki, float Kd);
void read_Accel_and_Gyro(Accel &accel, Gyro &gyro, DataVal &dataVal);
void integrate_to_vel_pos(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo);

#endif
