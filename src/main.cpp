#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "IMU_1.1.h"
#include "motor_controller.h"
#include "test_functions.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)


int main() {
    float control_signal_x = MOTOR_ARMING;
    float control_signal_y = MOTOR_ARMING;
    float Kp = 0.5, Ki = 0.005, Kd = 0.2; // Most optimal PID values at the moment
    Accel accel;
    Gyro gyro;
    DataVal dataVal;
    PID pid_x;
    PID pid_y;

    stdio_init_all();
    sleep_ms(2000);
    init_pwm();
    sleep_ms(5000);
    if (cyw43_arch_init()) {
        std::cout << "Failed to initialize CYW43 chip\n";
        return 1;
    }

    char userInput;
    while (true) {
        std::cout << "Press 'i' to initialize setup\n";
        std::cin >> userInput;
        if (userInput == 'i') {
            break;
        }
    }

    std::cout << "Initializing IMU...\n";
    lsm6ds3_init();
    configure_LSM6DS3();
    resetTimestamp();

    std::cout << "What do you want to test?\n";
    std::cout << "1. IMU sensor\n";
    std::cout << "2. Speed\n";
    std::cout << "3. Desired range\n";
    std::cout << "4. PID controller\n";
    std::cout << "5. Program ESC\n";
    while(true){
        char testChoice;
        std::cin >> testChoice;
        
        if(testChoice == '1'){
            std::cout << "IMU sensor test\n";
            test_IMU_sensor(accel, gyro, dataVal, 5);
        }
        else if(testChoice == '2'){
            std::cout << "Speed test\n";
            test_speed(control_signal_x, 30);
        }
        else if(testChoice == '3'){
            std::cout << "Desired range test\n";
            test_50_100_speed(control_signal_x, 50, 100);
        }
        else if(testChoice == '4'){
            std::cout << "PID controller test\n";
            test_PID_controller(accel, gyro, dataVal, pid_x, gyro.ang_pos_xyz[ROLL], control_signal_x, Kp, Ki, Kd);
        }
        else if(testChoice == '5'){
            std::cout << "Program ESC\n";
            programming_ESC();
        }
        else {
            std::cout << "Invalid choice. \n";
            continue;
        }
    }

    return 0;
}
/*******************************************************************************************/

