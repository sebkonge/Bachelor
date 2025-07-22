#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "IMU_1.1.h"

#define PWM_PIN_X 20               // GPIO pin for ESC signal
#define PWM_PIN_Y 21               // GPIO pin for ESC signal
#define PWM_FREQ 50             // 50 Hz PWM (20ms period for ESC)
#define MIN_PULSE 1000          // 1.0 ms pulse (0% throttle)
#define MAX_PULSE 2000          // 2.0 ms pulse (100% throttle)
#define MOTOR_ARMING 10         // Motor zero throttle (10%)
#define MOTOR_START 15          // Motor start throttle (15%)
#define MOTOR_MAX_CONTROL 100   // Max control signal (100%)

void init_pwm_x();
void init_pwm_y();
void init_pwm();
void set_motor_throttle(float pulse_width, char motor_axis);
void control_motor(float control_output, char motor_axis, float max_output);
void PID_motor_control_regulator(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid, std::string pid_axis, char motor_axis, float &current_angle, float Kp, float Ki, float Kd);

#endif
