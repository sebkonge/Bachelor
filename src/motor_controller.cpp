#include "motor_controller.h"

void init_pwm() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_1 = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_1, 20000); // set the wrap value to 20ms period
    pwm_set_clkdiv(slice_1, 125.0); // set the clock divider to 125Mhz (125000000/125/20000 = 50Hz)
    pwm_set_enabled(slice_1, true);
}

void set_motor_throttle(float pulse_width) {
    uint slice_1 = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_1, PWM_CHAN_A, pulse_width);
}

void control_motor(float control_output, float max_output) {
    float throttle_percent = (control_output / max_output) * 100.0; // If MAX_OUTPUT is changed, the throttle percent will be changed accordingly
    if (throttle_percent < 0) throttle_percent = 0;
    if (throttle_percent > 100) throttle_percent = 100;
    int pulse_width = MIN_PULSE + (throttle_percent / 100) * (MAX_PULSE - MIN_PULSE);
    uint slice_1 = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_1, PWM_CHAN_A, pulse_width);
}

void PID_motor_control_regulator(   Accel &accel,
                                    Gyro &gyro, 
                                    DataVal &dataVal, 
                                    PID &pid, 
                                    float &current_angle, 
                                    float Kp, float Ki, float Kd) {
    float output = PID_regulator(dataVal, pid, current_angle, Kp, Ki, Kd);
    set_motor_throttle(output);
}
