#include "motor_controller.h"

void init_pwm() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_1 = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_1, 20000);
    pwm_set_clkdiv(slice_1, 125.0);
    pwm_set_enabled(slice_1, true);
}

void set_motor_throttle(float throttle_percent) {
    if (throttle_percent < 0) throttle_percent = 0;
    if (throttle_percent > 100) throttle_percent = 100;
    int pulse_width = MIN_PULSE + (throttle_percent / 100) * (MAX_PULSE - MIN_PULSE);
    uint slice_1 = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_chan_level(slice_1, PWM_CHAN_A, pulse_width);
}

void control_motor(float control_output, float max_output) {
    float throttle = (control_output / max_output) * 100.0;
    set_motor_throttle(throttle);
}
