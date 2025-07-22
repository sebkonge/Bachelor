#include "motor_controller.h"

void init_pwm_x() {
    gpio_set_function(PWM_PIN_X, GPIO_FUNC_PWM);
    uint pwm_pin_x = pwm_gpio_to_slice_num(PWM_PIN_X);
    pwm_set_wrap(pwm_pin_x, 20000); // set the wrap value to 20ms period
    pwm_set_clkdiv(pwm_pin_x, 125.0); // set the clock divider to 125Mhz (125000000/125/20000 = 50Hz)
    pwm_set_enabled(pwm_pin_x, true);
}

void init_pwm_y() {
    gpio_set_function(PWM_PIN_Y, GPIO_FUNC_PWM);
    uint pwm_pin_y = pwm_gpio_to_slice_num(PWM_PIN_Y);
    pwm_set_wrap(pwm_pin_y, 20000); // set the wrap value to 20ms period
    pwm_set_clkdiv(pwm_pin_y, 125.0); // set the clock divider to 125Mhz (125000000/125/20000 = 50Hz)
    pwm_set_enabled(pwm_pin_y, true);
}

void init_pwm() {
    init_pwm_x();
    init_pwm_y();
}

void set_motor_throttle(float pulse_width, char motor_axis) {
    if (motor_axis == 'x') {
        uint pwm_pin_x = pwm_gpio_to_slice_num(PWM_PIN_X);
        pwm_set_chan_level(pwm_pin_x, PWM_CHAN_A, pulse_width); // Pin 20 (PWM_PIN_X) is connected to PWM_CHAN_A
    } else if (motor_axis == 'y') {
        uint pwm_pin_y = pwm_gpio_to_slice_num(PWM_PIN_Y);
        pwm_set_chan_level(pwm_pin_y, PWM_CHAN_B, pulse_width); // Pin 21 (PWM_PIN_Y) is connected to PWM_CHAN_B
    }
}

void control_motor(float control_output, char motor_axis, float max_output) {
    float throttle_percent = (control_output / max_output) * 100.0; // If MAX_OUTPUT is changed, the throttle percent will be changed accordingly
    if (throttle_percent < 0) throttle_percent = 0;
    if (throttle_percent > 100) throttle_percent = 100;
    int pulse_width = MIN_PULSE + (throttle_percent / 100) * (MAX_PULSE - MIN_PULSE); // Limit the output to 1000ms - 2000ms

    if (motor_axis == 'x') {
        uint pwm_pin_x = pwm_gpio_to_slice_num(PWM_PIN_X);
        pwm_set_chan_level(pwm_pin_x, PWM_CHAN_A, pulse_width); // Pin 20 (PWM_PIN_X) is connected to PWM_CHAN_A
    } else if (motor_axis == 'y') {
        uint pwm_pin_y = pwm_gpio_to_slice_num(PWM_PIN_Y);
        pwm_set_chan_level(pwm_pin_y, PWM_CHAN_B, pulse_width); // Pin 21 (PWM_PIN_Y) is connected to PWM_CHAN_B
    }
}

void PID_motor_control_regulator(   Accel &accel,
                                    Gyro &gyro, 
                                    DataVal &dataVal, 
                                    PID &pid,
                                    std::string pid_axis, 
                                    char motor_axis,
                                    float &current_angle, 
                                    float Kp, float Ki, float Kd) {
    pid.pid_axis = pid_axis;
    float output = PID_regulator(dataVal, pid, pid_axis, current_angle, Kp, Ki, Kd);
    set_motor_throttle(output, motor_axis);
}
