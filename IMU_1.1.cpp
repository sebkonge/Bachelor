#include "IMU_1.1.h"

// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)

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
        std::cout << "LSM6DS3 not detected! Check wiring.\n" << std::endl;
        return;
    }
    std::cout << "LSM6DS3 detected!\n" << std::endl;
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
                    float &current_angle,
                    float Kp, float Ki, float Kd
                    ){
    
    float output = 0;
    float timestamp_seconds_pid = dataVal.timestamp * 0.000025; //LSB = 25μs    
    float dt = timestamp_seconds_pid - pid.prev_timestamp_seconds_pid;
    
    //Calculate the error
    pid.error = pid.target_angle - current_angle;

    //Calculate the integral and Anti-windup
    if(pid.integral <= pid.target_angle){
        pid.integral = pid.integral + pid.error * dt;
    }
    
    //Calculate the derivative
    if(dt > 0){
        pid.derivative = (pid.error - pid.prev_error) / dt;
    } else {
        pid.derivative = 0;
    }

    //Calculate the output
    output = (Kp * pid.error) + (Ki * pid.integral) + (Kd * pid.derivative);

    //Update the previous values
    pid.prev_error = pid.error;
    pid.prev_timestamp_seconds_pid = timestamp_seconds_pid;


    // Limit output from MIN and MAX ticks
    if(output > MAX_PULSE){
        output = MAX_PULSE;
    } else if(output < MIN_PULSE){
        output = MIN_PULSE;
    }

    // Normalize output to 0-100%
    output = (output - MIN_PULSE) / (MAX_PULSE - MIN_PULSE) * 100;

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
    
    //Both accelerometer and gyroscope
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_gyro_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, gyro_accel_data, 12, false);
    
    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_timer, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, timestamp_data, 3, false);
    
    dataVal.timestamp = (timestamp_data[2] << 16) | (timestamp_data[1] << 8) | timestamp_data[0];
    dataVal.raw_gyro_xyz[ROLL] = (int16_t)((gyro_accel_data[1] << 8) | gyro_accel_data[0]);
    dataVal.raw_gyro_xyz[PITCH] = (int16_t)((gyro_accel_data[3] << 8) | gyro_accel_data[2]);
    dataVal.raw_gyro_xyz[YAW] = (int16_t)((gyro_accel_data[5] << 8) | gyro_accel_data[4]);
    dataVal.raw_accel_xyz[ROLL] = (int16_t)((gyro_accel_data[7] << 8) | gyro_accel_data[6]);
    dataVal.raw_accel_xyz[PITCH] = (int16_t)((gyro_accel_data[9] << 8) | gyro_accel_data[8]);
    dataVal.raw_accel_xyz[YAW] = (int16_t)((gyro_accel_data[11] << 8) | gyro_accel_data[10]);
    
}

void integrate_to_vel_pos(  Accel &accel, 
                            Gyro &gyro, 
                            DataVal &dataVal
                            ){
    float lin_acc_sens = 0.244 / 1000; // Sensitivity for ±8g (mg/LSB) and use 0.488 Sensitivity for ±16g (mg/LSB)
    float gyro_sens = 35.0 / 1000; // Sensitivity for ±1000 dps (mdps/LSB)
    float gravity = 9.81; // Gravity constant (m/s^2)
    float alpha = 0.98; // Between 0.95 and 0.98
    
    //Calculate time steps
    float timestamp_seconds = dataVal.timestamp * 0.000025; //LSB = 25μs    
    float dt = timestamp_seconds - dataVal.prev_timestamp_seconds;

    //Calculate the linear acceleration and angular velocity using sensitivity for acc and gyro
    accel.acc_xyz[ROLL] = (dataVal.raw_accel_xyz[ROLL] * lin_acc_sens) * 9.81; // Convert to m/s^2 by multiplying with gravity
    accel.acc_xyz[PITCH] = (dataVal.raw_accel_xyz[PITCH] * lin_acc_sens) * 9.81; // Convert to m/s^2 by multiplying with gravity
    accel.acc_xyz[YAW] = (dataVal.raw_accel_xyz[YAW] * lin_acc_sens) * 9.81; // Convert to m/s^2 by multiplying with gravity
    gyro.ang_vel_xyz[ROLL] = dataVal.raw_gyro_xyz[ROLL] * gyro_sens;
    gyro.ang_vel_xyz[PITCH] = dataVal.raw_gyro_xyz[PITCH] * gyro_sens;
    gyro.ang_vel_xyz[YAW] = dataVal.raw_gyro_xyz[YAW] * gyro_sens;

    //Calculate pitch and roll from accelerometer and convert to degrees
    accel.acc_ang[ROLL] = atan2(accel.acc_xyz[PITCH], sqrt(pow(accel.acc_xyz[ROLL], 2) + pow(accel.acc_xyz[YAW], 2))) * RAD_TO_DEG;
    accel.acc_ang[PITCH]  = atan2(-accel.acc_xyz[ROLL], sqrt(pow(accel.acc_xyz[PITCH],2) + pow(accel.acc_xyz[YAW], 2))) * RAD_TO_DEG;

    //Gyro integration
    gyro.axis[ROLL] = gyro.ang_pos_xyz[ROLL] + gyro.ang_vel_xyz[ROLL] * dt;
    gyro.axis[PITCH] = gyro.ang_pos_xyz[PITCH] + gyro.ang_vel_xyz[PITCH] * dt;
    gyro.axis[YAW] = gyro.ang_pos_xyz[YAW] + gyro.ang_vel_xyz[YAW] * dt;

    //Complementary filter
    gyro.ang_pos_xyz[ROLL] = alpha * gyro.axis[ROLL] + (1 - alpha) * accel.acc_ang[ROLL];
    gyro.ang_pos_xyz[PITCH] = alpha * gyro.axis[PITCH] + (1 - alpha) * accel.acc_ang[PITCH];
    //gyro.ang_pos_xyz[YAW] = gyro.ang_pos_xyz[YAW] + gyro.axis[YAW];
    
    //Print to terminal
    std::cout << "\n----------------Timestamp----------------" << std::endl;
    std::cout << "Timestamp: " << timestamp_seconds << " s" << std::endl;
    std::cout << "\n----------------Accelerometer values----------------" << std::endl;
    std::cout << "Acceleration X: " << accel.acc_xyz[ROLL] << "m/s^2, ";
    std::cout << "Acceleration Y: " << accel.acc_xyz[PITCH] << "m/s^2, ";
    std::cout << "Acceleration Z: " << accel.acc_xyz[YAW] - gravity << "m/s^2" << std::endl;
    std::cout << "\n----------------Gyro values----------------" << std::endl;
    std::cout << "Angular posi X: " << gyro.ang_pos_xyz[ROLL] << "°, ";
    std::cout << "Angular posi Y: " << gyro.ang_pos_xyz[PITCH] << "°, ";
    std::cout << "Angular posi Z: " << gyro.ang_pos_xyz[YAW] << "°\n" << std::endl;
    
    //Update the previous values
    accel.prev_vel_xyz[ROLL] = accel.vel_xyz[ROLL];
    accel.prev_vel_xyz[PITCH] = accel.vel_xyz[PITCH];
    accel.prev_vel_xyz[YAW] = accel.vel_xyz[YAW];
    accel.prev_pos_xyz[ROLL] = accel.pos_xyz[ROLL];
    accel.prev_pos_xyz[PITCH] = accel.pos_xyz[PITCH];
    accel.prev_pos_xyz[YAW] = accel.pos_xyz[YAW];
    gyro.prev_ang_xyz[ROLL] = gyro.ang_pos_xyz[ROLL];
    gyro.prev_ang_xyz[PITCH] = gyro.ang_pos_xyz[PITCH];
    gyro.prev_ang_xyz[YAW] = gyro.ang_pos_xyz[YAW];
    dataVal.prev_timestamp_seconds = timestamp_seconds;
}
/*******************************************************************************************/
