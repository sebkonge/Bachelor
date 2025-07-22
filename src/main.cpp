#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "IMU_1.1.h"
#include "motor_controller.h"
#include "test_functions.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "picoSDCard.h"

#define LED_PIN 16
#define TRIGGER_PIN 17
#define LED_FAST_BLINK 150
#define LED_SLOW_BLINK 500
#define LED_VERY_SLOW_BLINK 2000
#define LOGGING_TIME 30
#define LED_ON 1
#define LED_OFF 0
// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)

enum STATES {
    MOUNT_STATE = 0,    // LED turned off
    READY_STATE,        // LED blink slow
    RUN_STATE,          // LED Always on
    FINISH_STATE,       // LED blink very slow
    ERROR_STATE         // LED blink fast
};

int main() {
    float control_signal_x = MOTOR_ARMING;
    float control_signal_y = MOTOR_ARMING;
    float Kp = 12.f, Ki = 0.f, Kd = 3.f; // Most optimal PID values at the moment
    Accel accel;
    Gyro gyro;
    DataVal dataVal;
    PID pid_x;
    PID pid_y;
    sdInfo sdInfo;
    STATES current_state = MOUNT_STATE;
    int led_on_off_switch = 1;

    stdio_init_all();
    sleep_ms(2000);
    init_pwm();
    sleep_ms(5000);
    if (cyw43_arch_init()) {
        std::cout << "Failed to initialize CYW43 chip\n";
        current_state = ERROR_STATE;
        return 1;
    }

    // IMU init
    lsm6ds3_init();
    configure_LSM6DS3();
    resetTimestamp();
    
    const char* filName = "ImuRaket.txt";
    const char* columSetup = "Timestamp Acc_roll Acc_pitch Acc_yaw ang_pos_roll ang_pos_pitch ang_pos_yaw x_regulator_on pid_x_output y_regulator_on pid_y_output\n";
    
    // SD Card init
    SDClass sd(filName, columSetup);
    
    // Trigger pin init
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_IN);
    gpio_pull_up(TRIGGER_PIN);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    //test_functions(accel, gyro, dataVal, pid_x, pid_y, sdInfo, "pid_x", 'x', gyro.ang_pos_xyz[ROLL], control_signal_y, Kp, Ki, Kd, 10, 50, 50, 100, 50); // Test functions for IMU and motor controller

    while(true){
        switch(current_state){
            case MOUNT_STATE:
                if(sd.mountSDCard()){ // if SD card is mounted, go to ready state
                    current_state = READY_STATE;
                }
                else {
                    current_state = ERROR_STATE;
                }
                break;

            case READY_STATE:
                turn_on_motors();
                while(true){
                    bool started = gpio_get(TRIGGER_PIN);
                    if(started){ // if trigger pin is pulled, reset IMU and go to run state
                        resetIMU();
                        resetTimestamp();
                        current_state = RUN_STATE;
                        break;
                    }
                    led_on_off_switch ^= 1;
                    gpio_put(LED_PIN, led_on_off_switch);
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on_off_switch);
                    sleep_ms(LED_SLOW_BLINK);
                }
                break;
                
            case RUN_STATE:
                control_motor(50, 'x', MOTOR_MAX_CONTROL);
                control_motor(50, 'y', MOTOR_MAX_CONTROL);
                gpio_put(LED_PIN, LED_OFF);
                sleep_ms(2000);
                gpio_put(LED_PIN, LED_ON);
                sleep_ms(500);
                while(true){
                    gpio_put(LED_PIN, LED_ON);
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_ON);
                    read_Accel_and_Gyro(accel, gyro, dataVal);
                    integrate_to_vel_pos(accel, gyro, dataVal, pid_x, pid_y, sdInfo);
                    if( ! sd.writeStringSDCard(sdInfo.InfoString)){ // if writing to SD card fails, unmount SD card and go to error state
                        sd.unmountSDCard();
                        current_state = ERROR_STATE;
                        break;
                    }
                    if(abs(gyro.ang_pos_xyz[ROLL]) > 80 || abs(gyro.ang_pos_xyz[PITCH]) > 80){ // if gyro angles are too high, turn off motors and go to finish state
                        control_signal_x = 0;
                        control_signal_y = 0;
                        control_motor(control_signal_x, 'x', MOTOR_MAX_CONTROL);
                        control_motor(control_signal_y, 'y', MOTOR_MAX_CONTROL);
                        // current_state = FINISH_STATE;
                        // break;
                    }
                    else{
                        PID_motor_control_regulator(accel, gyro, dataVal, pid_x, "pid_x", 'x', gyro.ang_pos_xyz[ROLL], Kp, Ki, Kd);
                        PID_motor_control_regulator(accel, gyro, dataVal, pid_y, "pid_y", 'y', gyro.ang_pos_xyz[PITCH], Kp, Ki, Kd);
                    }
                    if(sdInfo.timestamp >= LOGGING_TIME){ // if logging time is reached, unmount SD card and go to finish state
                        sd.unmountSDCard();
                        current_state = FINISH_STATE;
                        break;
                    }
                }
                break;
            case FINISH_STATE:
                while(true){ // if finish state, blink very slow
                    led_on_off_switch ^=1;
                    control_motor(0, 'x', MOTOR_MAX_CONTROL);
                    control_motor(0, 'y', MOTOR_MAX_CONTROL);
                    gpio_put(LED_PIN, led_on_off_switch);
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on_off_switch);
                    sleep_ms(LED_VERY_SLOW_BLINK);
                }
            break;

            case ERROR_STATE:
                while(true){ // if error state, blink fast
                    led_on_off_switch ^=1;
                    control_motor(0, 'x', MOTOR_MAX_CONTROL);
                    control_motor(0, 'y', MOTOR_MAX_CONTROL);
                    gpio_put(LED_PIN, led_on_off_switch);
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on_off_switch);
                    sleep_ms(LED_FAST_BLINK);
                }
            break;
            
            default:
            current_state = ERROR_STATE;
        }   
    }

    return 0;
}
/*******************************************************************************************/
