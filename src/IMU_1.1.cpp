#include "IMU_1.1.h"

// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)

void resetIMU(){
    uint8_t resetCwd = 0x01;
    uint8_t CTRL3_C = 0x12;
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &CTRL3_C, 1, true);
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &resetCwd, 1, false);
    sleep_ms(10);
}

void lsm6ds3_init(void) {
    //Init of pins with pullup
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C); //Selects specified GPIO Ports for I2C functionallity
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); //Sets specified GPIO Ports to be pulled up
    gpio_pull_up(SCL_PIN);

    sleep_ms(100);  // Short delay for sensor startup

    uint8_t who_am_i_reg = WHO_AM_I_REG;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &who_am_i_reg, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, chipID, 1, false);

    if (chipID[0] != WHO_AM_I_VALUE) {
        std::cout << "LSM6DS3 detected!\n" << std::endl;
        return;
    }
    std::cout << "LSM6DS3 not detected! Check wiring.\n" << std::endl;
}

void configure_LSM6DS3(void){
    uint8_t config[2];

    // Enable Accelerometer (104Hz, ±2g, LPF enabled)
    config[0] = 0x10;  // CTRL1_XL register
    config[1] = CTRL1_XL;
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    // Enable Gyroscope (104Hz, 2000 dps)
    config[0] = 0x11;  // CTRL2_G register
    config[1] = CTRL2_G;
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    // Enable Block Data Update & Auto-increment (CTRL3_C)
    config[0] = 0x12;  // CTRL3_C register
    config[1] = 0b01000100;  // BDU=1, Auto-increment enabled
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    // config[0] = 0x17;  // CTRL8_XL register
    // config[1] = CTRL8_XL; // Add low pass filter to accelerometer (400 Hz)
    // i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    //Enable timestamp
    config[0] = 0x19;  // CTRL10_C register
    config[1] = 0b00100000;  // Set TMD bit (bit 5) to enable timestamp
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    config[0] = 0x5C; // WAKE_UP_DUR register
    config[1] = 0b00010000; //Resolution... LSB = 25μs 
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);
}

void readTimestamp(void){
    uint8_t reg = 0x40;  // Start at TIMESTAMP0_REG
    uint8_t timestamp_data[3];

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, timestamp_data, 3, false);

    // Combine bytes into 24-bit integer
    uint32_t timestamp = (timestamp_data[2] << 16) | (timestamp_data[1] << 8) | timestamp_data[0];

    // Convert to seconds (if TIMER_HR = 1, LSB = 25μs)
    float timestamp_seconds = timestamp * 0.000025;

    std::cout << "Timestamp: " << timestamp_seconds << " s" << std::endl;
}

void resetTimestamp(void){
    uint8_t reset[2] = {0x42, 0xAA};  
    i2c_write_blocking(I2C_PORT, IMU_ADDR, reset, 2, false);
    std::cout << "Timestamp reset\n";
}

/*******************************************************************************************/
/************************************ - PID regulator - ************************************/
float PID_regulator(DataVal &dataVal,
                    PID &pid,
                    std::string pid_axis,
                    float &current_angle,
                    float Kp, float Ki, float Kd
                    ){
    
    // Initialize output
    float output = 1500.f;
    float timestamp_seconds_pid = dataVal.timestamp * 0.000025f; //LSB = 25μs    
    float dt = timestamp_seconds_pid - pid.prev_timestamp_seconds_pid; // Set internal function dt
    
    // Calculate the error
    pid.error = pid.target_angle - current_angle;

    // Calculate the integral
    if(dt > 0){
        pid.integral = pid.integral + pid.error * dt;
    }
    
    // Calculate the derivative
    if(dt > 0){
        pid.derivative = (pid.error - pid.prev_error) / dt;
    } else {
        pid.derivative = 0.f;
    }

    if(pid_axis == "pid_x"){
        // std::cout << "pid_x" << std::endl;
        output -= (Kp * pid.error) + (Ki * pid.integral) + (Kd * pid.derivative);
    } else if(pid_axis == "pid_y"){
        // std::cout << "pid_y" << std::endl;
        output -= (Kp * pid.error) + (Ki * pid.integral) + (Kd * pid.derivative);
    }
    
    // Update the previous values
    pid.prev_error = pid.error;
    pid.prev_timestamp_seconds_pid = timestamp_seconds_pid;

    // Round the output to the nearest integer
    output = std::round(output);

    // Limit the output to 1000ms - 2000ms
    if (output > MAX_PULSE){
        output = MAX_PULSE;
    } else if(output < MIN_PULSE){
        output = MIN_PULSE;
    }

    if (output < 1400.f || output > 1600.f){
        pid.regulator_on = true; // Regulator is on when output is outside the range
    } else {
        pid.regulator_on = false; // Regulator is off when output is within the range
    }

    // Apply threshold to keep 50% throttle
    if (abs(output - 1500.f) < 80.f){ // Change in respect to oscillations or vibrations
        output = 1500.f;
    }

    // std::cout << "Output: " << output << std::endl;

    pid.output = output; // Store the output in the PID struct for data logging

    return output;
}
/*******************************************************************************************/
/************************************ - Read IMU data - ************************************/
void read_Accel_and_Gyro(   Accel &accel, 
                            Gyro &gyro, 
                            DataVal &dataVal){
    uint8_t reg_gyro_l = OUTX_L_G;
    uint8_t reg_timer = 0x40;  // Start at TIMESTAMP0_REG
    uint8_t timestamp_data[3];
    uint8_t gyro_accel_data[12];
    
    // Both accelerometer and gyroscope
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_gyro_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, gyro_accel_data, 12, false);
    
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_timer, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, timestamp_data, 3, false);

    // Gather raw data from accelerometer and gyroscope
    dataVal.timestamp = (timestamp_data[2] << 16) | (timestamp_data[1] << 8) | timestamp_data[0];
    dataVal.raw_gyro_xyz[ROLL] = (int16_t)((gyro_accel_data[1] << 8) | gyro_accel_data[0]);
    dataVal.raw_gyro_xyz[PITCH] = (int16_t)((gyro_accel_data[3] << 8) | gyro_accel_data[2]);
    dataVal.raw_gyro_xyz[YAW] = (int16_t)((gyro_accel_data[5] << 8) | gyro_accel_data[4]);
    dataVal.raw_accel_xyz[X] = (int16_t)((gyro_accel_data[7] << 8) | gyro_accel_data[6]);
    dataVal.raw_accel_xyz[Y] = (int16_t)((gyro_accel_data[9] << 8) | gyro_accel_data[8]);
    dataVal.raw_accel_xyz[Z] = (int16_t)((gyro_accel_data[11] << 8) | gyro_accel_data[10]);
    
}

void integrate_to_vel_pos(  Accel &accel, 
                            Gyro &gyro, 
                            DataVal &dataVal,
                            PID &pid_x,
                            PID &pid_y,
                            sdInfo &sdInfo
                            ){
    float lin_acc_sens = 0.488 / 1000; // 0.244 Sensitivity for ±8g (mg/LSB) and use 0.488 Sensitivity for ±16g (mg/LSB)
    float gyro_sens = 35.0 / 1000; // Sensitivity for ±1000 dps (mdps/LSB)
    float gravity = 9.81; // Gravity constant (m/s^2)
    float alpha = 0.98; // How much weight to give, causing the filter to help the gyro with reference
    
    //Calculate time steps
    float timestamp_seconds = dataVal.timestamp * 0.000025; //LSB = 25μs    
    float dt = timestamp_seconds - dataVal.prev_timestamp_seconds;

    //Calculate the linear acceleration and angular velocity using sensitivity for acc and gyro
    accel.acc_xyz[X] = (dataVal.raw_accel_xyz[X] * lin_acc_sens) * gravity; // Convert to m/s^2 by multiplying with gravity
    accel.acc_xyz[Y] = (dataVal.raw_accel_xyz[Y] * lin_acc_sens) * gravity; // Convert to m/s^2 by multiplying with gravity
    accel.acc_xyz[Z] = (dataVal.raw_accel_xyz[Z] * lin_acc_sens) * gravity; // Convert to m/s^2 by multiplying with gravity

    gyro.ang_vel_xyz[ROLL] = dataVal.raw_gyro_xyz[ROLL] * gyro_sens;
    gyro.ang_vel_xyz[PITCH] = dataVal.raw_gyro_xyz[PITCH] * gyro_sens;
    gyro.ang_vel_xyz[YAW] = dataVal.raw_gyro_xyz[YAW] * gyro_sens;

    //Calculate pitch and roll from accelerometer and convert to degrees
    accel.acc_ang[ROLL] = atan2(accel.acc_xyz[Y], sqrt(pow(accel.acc_xyz[X], 2) + pow(accel.acc_xyz[Z], 2))) * RAD_TO_DEG;
    accel.acc_ang[PITCH]  = atan2(-accel.acc_xyz[X], sqrt(pow(accel.acc_xyz[Y],2) + pow(accel.acc_xyz[Z], 2))) * RAD_TO_DEG;

    //Gyro integration
    gyro.axis[ROLL] = gyro.ang_pos_xyz[ROLL] + gyro.ang_vel_xyz[ROLL] * dt;
    gyro.axis[PITCH] = gyro.ang_pos_xyz[PITCH] + gyro.ang_vel_xyz[PITCH] * dt;
    gyro.axis[YAW] = gyro.ang_pos_xyz[YAW] + gyro.ang_vel_xyz[YAW] * dt;

    //Complementary filter
    gyro.ang_pos_xyz[ROLL] = alpha * gyro.axis[ROLL] + (1.f - alpha) * accel.acc_ang[ROLL];
    gyro.ang_pos_xyz[PITCH] = alpha * gyro.axis[PITCH] + (1.f - alpha) * accel.acc_ang[PITCH];
    gyro.ang_pos_xyz[YAW] = gyro.ang_pos_xyz[YAW] + gyro.axis[YAW];
    
    // Debug
    // std::cout << "\n----------------Timestamp----------------" << std::endl;
    // std::cout << "Timestamp: " << timestamp_seconds << " s" << std::endl;
    // std::cout << "\n----------------Accelerometer values----------------" << std::endl;
    // std::cout << "Acceleration X: " << accel.acc_xyz[ROLL] << "m/s^2, ";
    // std::cout << "Acceleration Y: " << accel.acc_xyz[PITCH] << "m/s^2, ";
    // std::cout << "Acceleration Z: " << accel.acc_xyz[YAW] << "m/s^2" << std::endl;
    // std::cout << "\n----------------Gyro values----------------" << std::endl;
    // std::cout << "Angular posi X: " << gyro.ang_pos_xyz[ROLL] << "°, ";
    // std::cout << "Angular posi Y: " << gyro.ang_pos_xyz[PITCH] << "°, ";
    // std::cout << "Angular posi Z: " << gyro.ang_pos_xyz[YAW] << "°\n" << std::endl;

    // Data logging to SD card
    sdInfo.InfoString = std::to_string(timestamp_seconds) + " " + std::to_string(accel.acc_xyz[X])  + " " 
    + std::to_string(accel.acc_xyz[Y]) + " " + std::to_string(accel.acc_xyz[Z]/*< 0.15 ? 0 : accel.acc_xyz[YAW]*/) + " " 
    + std::to_string(gyro.ang_pos_xyz[ROLL]) + " " + std::to_string(gyro.ang_pos_xyz[PITCH]) + " " + std::to_string(gyro.ang_pos_xyz[YAW]) + " "
    + std::to_string(pid_x.regulator_on) + " " + std::to_string(pid_x.output) + " " + std::to_string(pid_y.regulator_on) + " " + std::to_string(pid_y.output) + "\n";

    std::cout << sdInfo.InfoString << std::endl;

    sdInfo.timestamp = timestamp_seconds;
    sdInfo.ZAcceleration  = accel.acc_xyz[YAW] - gravity;

    //Update the previous values
    accel.prev_vel_xyz[X] = accel.vel_xyz[X];
    accel.prev_vel_xyz[Y] = accel.vel_xyz[Y];
    accel.prev_vel_xyz[Z] = accel.vel_xyz[Z];
    accel.prev_pos_xyz[X] = accel.pos_xyz[X];
    accel.prev_pos_xyz[Y] = accel.pos_xyz[Y];
    accel.prev_pos_xyz[Z] = accel.pos_xyz[Z];
    gyro.prev_ang_xyz[ROLL] = gyro.ang_pos_xyz[ROLL];
    gyro.prev_ang_xyz[PITCH] = gyro.ang_pos_xyz[PITCH];
    gyro.prev_ang_xyz[YAW] = gyro.ang_pos_xyz[YAW];
    dataVal.prev_timestamp_seconds = timestamp_seconds;
}
/*******************************************************************************************/