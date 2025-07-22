#include "test_functions.h"

bool kbhit() {
    return getchar_timeout_us(0) != PICO_ERROR_TIMEOUT;
}

void programming_ESC(char motor_axis){
    control_motor(100, motor_axis, MOTOR_MAX_CONTROL);
    std::cout << "Programming ESC" << std::endl;
    std::cout << "Turn on powersupply when pressing 'y', your speed is " << 100 << "%" << std::endl;
    char start;
    std::cin >> start;
    if(start == 'y') {
        std::cout << "after 2 seconds, the ESC will be programable with special beep-beep" << std::endl;
        sleep_ms(2000);
        while(true) {
            control_motor(100, motor_axis, MOTOR_MAX_CONTROL);
            std::cout << "Wait another 5 seconds, and a special tone should be emitted which means program mode is entered" << std::endl;
            sleep_ms(5000);
            char input;
            std::cout << "Press 'p' to program ESC within 3 seconds of the desired programmable option on datasheet" << std::endl;
            std::cout << "1 long beep corrosponds to 4 small beeps, 2 long beeps is exit" << std::endl;
            std::cin >> input;
            if(input == 'p') {
                control_motor(0, motor_axis, MOTOR_MAX_CONTROL);
                std::cout << "Now you will hear several tones in loop, press 'd' when desired option is heard" << std::endl;
                char input2;
                std::cin >> input2;
                if(input2 == 'd') {
                    control_motor(100, motor_axis, MOTOR_MAX_CONTROL);
                    input2 = 'n';
                    std::cout << "Desired program saved" << std::endl;
                }
            }
        }
    }
}

void test_PID_controller(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo, std::string pid_axis, char motor_axis, float &current_angle, float control_signal, float Kp, float Ki, float Kd, int datasets) {
    int counter = 0;
    for (int i = 1; i <= datasets; i++){
        counter++;
        char redo;
        std::cout << "Press r for new eksperiment" << std::endl;
        std::cin >> redo;
        std::cout << "Experiment number: " << counter << std::endl;
        if (redo == 'r') {
            control_motor(0, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 0% throttle to counter Over-load protection mode
            sleep_ms(2000);
            if(counter == 1) {
                control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 10% throttle but only for the first time
                sleep_ms(2000); // 2 seconds startup protection time
            }
            control_signal = MOTOR_START; // Start the motor at required throttle level (15%)
            if(control_signal == MOTOR_START) { // If the next signal is 15% throttle
                control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Turn on the motor with required 15% throttle
                control_signal += 35;
                if(control_signal >= 35) { // If the throttle level is at 50%
                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                    std::cout << control_signal << "% throttle" << std::endl;
                    sleep_ms(2000);
                    while (true) { 
                        read_Accel_and_Gyro(accel, gyro, dataVal);
                        integrate_to_vel_pos(accel, gyro, dataVal, pid_x, pid_y, sdInfo);
                        PID_motor_control_regulator(accel, gyro, dataVal, pid_x, pid_axis, motor_axis, current_angle, Kp, Ki, Kd);
                        if (kbhit()) {
                            control_signal = 0;
                            control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                            std::cout << "Experiment done" << std::endl;
                            break;
                        }
                    }
                }
            }
        }
        sleep_ms(1000);
        resetTimestamp();
        dataVal.timestamp = 0;
    }
}

void test_50_100_speed(float control_signal, char motor_axis, int throttle_percent1, int throttle_percent2) {
    int counter = 0;
    while (true) {
        counter++;
        char redo;
        std::cout << "Press r for new eksperiment" << std::endl;
        std::cin >> redo;
        std::cout << "Experiment number: " << counter << std::endl;
        if (redo == 'r') {
            while (true) {
                control_motor(0, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 0% throttle to counter Over-load protection mode
                sleep_ms(2000);
                if(counter == 1) {
                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 10% throttle but only for the first time
                    sleep_ms(2000); // 2 seconds startup protection time
                }
                control_signal = MOTOR_START; // Start the motor at required throttle level (15%)
                if(control_signal == MOTOR_START){ // If the next signal is 15% throttle
                    while(true) {
                        control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                        control_signal = throttle_percent1;
                        if(control_signal >= throttle_percent1) {
                            control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                            std::cout << control_signal << "% throttle" << std::endl;
                            sleep_ms(1000);
                            char speedUp;
                            std::cout << "Press 's' to speed up" << std::endl;
                            std::cin >> speedUp;
                            if(speedUp == 's') {
                                control_signal = throttle_percent2;                    
                                if(control_signal >= throttle_percent2) {
                                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                    std::cout << control_signal << "% throttle" << std::endl;
                                    std::cout << "Ready to brake" << std::endl;
                                    char input;
                                    std::cin >> input;
                                    if(input == 'b') {
                                        control_signal = 0;
                                        control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                        if(control_signal == 0) {
                                            std::cout << "Motor is off" << std::endl;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if(control_signal == 0) {
                        break;
                    }
                }
            }
        }
    }
}

void test_speed(float control_signal, char motor_axis, int throttle_percent) {
    int counter = 0;
    while (true) {
        counter++;
        char redo;
        std::cout << "Press r for new eksperiment" << std::endl;
        std::cin >> redo;
        std::cout << "Experiment number: " << counter << std::endl;
        if (redo == 'r') {
            while (true) {
                control_motor(0, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 0% throttle to counter Over-load protection mode
                sleep_ms(2000);
                if(counter == 1) {
                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 10% throttle but only for the first time
                    sleep_ms(2000); // 2 seconds startup protection time
                }
                control_signal = MOTOR_START; // Start the motor at required throttle level (15%)
                if(control_signal == MOTOR_START){ // If the next signal is 15% throttle
                    while(true) {
                        control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Turn on the motor with required 15% throttle
                        sleep_ms(2000);
                        control_signal = throttle_percent;
                        if(control_signal >= throttle_percent) { // If the throttle level is at 100%
                            control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                            std::cout << control_signal << "% throttle" << std::endl;
                            std::cout << "Ready to brake" << std::endl;
                            char input;
                            std::cin >> input;
                            if(input == 'b') {
                                control_signal = 0;
                                control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                if(control_signal == 0) {
                                    std::cout << "Motor is off" << std::endl;
                                    break;
                                }
                            }
                        }
                    }
                    if(control_signal == 0) {
                        break;
                    }
                }
            }
        }
    }
}

void turn_on_motors() {
    control_motor(0, 'x', MOTOR_MAX_CONTROL); // Arm the motor at 0% throttle to counter Over-load protection mode
    control_motor(0, 'y', MOTOR_MAX_CONTROL);
    sleep_ms(2000);
    control_motor(10, 'x', MOTOR_MAX_CONTROL);
    control_motor(10, 'y', MOTOR_MAX_CONTROL);
    sleep_ms(2000);
    control_motor(15, 'x', MOTOR_MAX_CONTROL); // Start the motor at required throttle level (15%)
    control_motor(15, 'y', MOTOR_MAX_CONTROL);
    sleep_ms(2000);
    control_motor(0, 'x', MOTOR_MAX_CONTROL); // Start the motor at required throttle level (50%)
    control_motor(0, 'y', MOTOR_MAX_CONTROL);
}

void test_IMU_sensor(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo, int datasets) {
    while (true){
        int counter = 0;
        std::cout << "Press 'r' to start the experiment" << std::endl;
        char redo;
        std::cin >> redo;
        if (redo == 'r'){
            for (int i = 1; i <= datasets; i++){
                std::cout << "Dataset " << i << ":" << std::endl;
                // read data from IMU for 300 ms
                while(dataVal.timestamp < 12000){
                    read_Accel_and_Gyro(accel, gyro, dataVal);
                    integrate_to_vel_pos(accel, gyro, dataVal, pid_x, pid_y, sdInfo);
                }
                sleep_ms(1000);
                resetTimestamp();
                counter++;
                dataVal.timestamp = 0;
                if (i == 10){
                    std::cout << "Experiment done" << std::endl;
                    redo = 'n';
                    break;
                }
            }
        }
    }
}

void test_3_range(char motor_axis, float control_signal, int speedtest1, int speedtest2, int speedtest3) {
    int counter = 0;
    while (true) {
        counter++;
        char redo;
        std::cout << "Press r for new eksperiment" << std::endl;
        std::cin >> redo;
        std::cout << "Experiment number: " << counter << std::endl;
        if (redo == 'r') {
            while (true) {
                control_motor(0, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 0% throttle to counter Over-load protection mode
                sleep_ms(2000);
                if(counter == 1) {
                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL); // Arm the motor at 10% throttle but only for the first time
                    sleep_ms(2000); // 2 seconds startup protection time
                }
                control_signal = MOTOR_START; // Start the motor at required throttle level (15%)
                if(control_signal == MOTOR_START){ // If the next signal is 15% throttle
                    while(true) {
                        control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                        control_signal = speedtest1;
                        if(control_signal >= speedtest1) {
                            control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                            std::cout << control_signal << "% throttle" << std::endl;
                            sleep_ms(1000);
                            char speedUp;
                            std::cout << "Press 'r' to speed up" << std::endl;
                            std::cin >> speedUp;
                            if(speedUp == 'r') {
                                control_signal = speedtest2;                    
                                if(control_signal >= speedtest2) {
                                    control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                    std::cout << control_signal << "% throttle" << std::endl;
                                    std::cout << "Deaccelerate" << std::endl;
                                    char input;
                                    std::cin >> input;
                                    if(input == 'r') {
                                        control_signal = speedtest3;
                                        if (control_signal >= speedtest3) {
                                            control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                            std::cout << control_signal << "% throttle" << std::endl;
                                            std::cout << "Ready to brake" << std::endl;
                                            char input;
                                            std::cin >> input;
                                            if(input == 'b') {
                                                control_signal = 0;
                                                control_motor(control_signal, motor_axis, MOTOR_MAX_CONTROL);
                                                if(control_signal == 0) {
                                                    std::cout << "Motor is off" << std::endl;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if(control_signal == 0) {
                        break;
                    }
                }
            }
        }
    }
}


void test_functions(Accel &accel, Gyro &gyro, DataVal &dataVal, PID &pid_x, PID &pid_y, sdInfo &sdInfo, std::string pid_axis, char motor_axis, float &current_angle, float control_signal, float Kp, float Ki, float Kd, int datasets, int speedtest, int speedtest1, int speedtest2, int speedtest3) {
    std::cout << "What do you want to test?\n";
    std::cout << "1. IMU sensor\n";
    std::cout << "2. Speed\n";
    std::cout << "3. Desired range\n";
    std::cout << "4. PID controller\n";
    std::cout << "5. Test motor 3-range\n";
    std::cout << "6. Program ESC\n";

    while(true){
        char testChoice;
        std::cin >> testChoice;
        
        if(testChoice == '1'){
            std::cout << "IMU sensor test\n";
            test_IMU_sensor(accel, gyro, dataVal, pid_x, pid_y, sdInfo, datasets);
        }
        else if(testChoice == '2'){
            std::cout << "Speed test\n";
            test_speed(control_signal, motor_axis, speedtest);
        }
        else if(testChoice == '3'){
            std::cout << "Desired range test\n";
            test_50_100_speed(control_signal, motor_axis, speedtest1, speedtest2);
        }
        else if(testChoice == '4'){
            std::cout << "PID controller test\n";
            test_PID_controller(accel, gyro, dataVal, pid_x, pid_y, sdInfo, pid_axis, motor_axis, current_angle, control_signal, Kp, Ki, Kd, datasets);
            break;
        }
        else if(testChoice == '5'){
            std::cout << "3 range test\n";
            test_3_range(motor_axis, control_signal, speedtest1, speedtest2, speedtest3);
        }
        else if(testChoice == '6'){
            std::cout << "Program ESC\n";
            programming_ESC(motor_axis);
        }
        else {
            std::cout << "Invalid choice. \n";
            continue;
        }
    }
}