#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "IMU_1.1.h"
#include "motor_controller.h"
#include "pico/cyw43_arch.h"


// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)

int main() {
    float control_signal_x = MOTOR_ARMING;
    float control_signal_y = MOTOR_ARMING;
    float Kp = 0.3, Ki = 0.1, Kd = 0.1;
    Accel accel;
    Gyro gyro;
    DataVal dataVal;
    PID pid_x;
    PID pid_y;
    int counter = 0;
    
    stdio_init_all();
    init_pwm();
    sleep_ms(5000);

    if (cyw43_arch_init()) {
        std::cout << "Failed to initialize CYW43 chip\n";
        return 1;
    }

    char userInput;
    while (true) {
        sleep_ms(1000);
        printf("Press 'i' to initialize the IMU\n");
        std::cin >> userInput;
        if (userInput == 'i') {
            break;
        }
    }

    lsm6ds3_init();
    configure_LSM6DS3();
    resetTimestamp();

    //Test program for the IMU sensor
    // while(true){
    //     std::cout << "Dataset " << counter << ":" << std::endl;
    //     // read data from IMU for 300 ms
    //     while(dataVal.timestamp < 12000){
    //         read_Accel_and_Gyro(accel, gyro, dataVal);
    //         integrate_to_vel_pos(accel, gyro, dataVal);
    //         PID_regulator(dataVal, pid_x, gyro.ang_pos_xyz[ROLL], Kp, Ki, Kd);
    //         PID_regulator(dataVal, pid_y, gyro.ang_pos_xyz[PITCH], Kp, Ki, Kd);

    //     }
    //     sleep_ms(1000);
    //     resetTimestamp();
    //     counter++;
    //     dataVal.timestamp = 0;
    // }

    //Test program for the motor controller arming mode and throttle control
    while (true) {
        control_motor(control_signal_x, MOTOR_MAX_CONTROL); // Arm the motor at 10% throttle
        sleep_ms(2000); // 2 seconds calibration time
        control_signal_x = MOTOR_START; // Start the motor at required throttle level (15%)
        if(control_signal_x == MOTOR_START){ // If the next signal is 15% throttle
            while(true){
                control_motor(control_signal_x, MOTOR_MAX_CONTROL); // Turn on the motor with required 15% throttle
                control_signal_x += 100;
                if(control_signal_x >= 100){
                    control_motor(control_signal_x, MOTOR_MAX_CONTROL);
                    sleep_ms(10000);
                    // Brake the motor
                    control_signal_x = 0;
                    control_motor(control_signal_x, MOTOR_MAX_CONTROL);
                    if(control_signal_x == 0){
                        while(true){
                            std::cout << "Motor is off" << std::endl;
                            sleep_ms(1000);
                        }
                    }

                }
            }
        }
    }

    return 0;
}
/*******************************************************************************************/
