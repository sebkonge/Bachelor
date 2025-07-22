void read_Gyro(Gyro &gyro){
    uint8_t reg_gyro_l = OUTX_L_G;
    uint8_t gyro_data[6];
    float gyro_sens = 35.0 / 1000; // Sensitivity for ±1000 dps (mdps/LSB)
    

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_gyro_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, gyro_data, 6, false);

    int16_t raw_gyroX = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t raw_gyroY = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t raw_gyroZ = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    gyro.vel_pitch = raw_gyroX * gyro_sens;
    gyro.vel_yaw = raw_gyroY * gyro_sens;
    gyro.vel_roll = raw_gyroZ * gyro_sens;

    std::cout << "Gyro X: " << gyro.vel_pitch << "°/s, ";
    std::cout << "Gyro Y: " << gyro.vel_yaw << "°/s, ";
    std::cout << "Gyro Z: " << gyro.vel_roll << "°/s" << std::endl;
}




void read_Accelerometer(Accel &accel){
    uint8_t reg_accel_l = OUTX_L_XL;
    uint8_t accel_data[6];
    float lin_acc_sens = 0.122 / 1000; // Sensitivity for ±4g (mg/LSB)
    float gravity = 9.816; // Gravity constant (m/s^2)

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_accel_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, accel_data, 6, false);

    int16_t raw_accelX = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t raw_accelY = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t raw_accelZ = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    accel.acc_xyz[0] = (raw_accelX * lin_acc_sens) * 9.81;
    accel.acc_xyz[1] = (raw_accelY * lin_acc_sens) * 9.81;
    accel.acc_xyz[2] = (raw_accelZ * lin_acc_sens) * 9.81;

    std::cout << "Accel X: " << accel.acc_xyz[0] << "m/s^2, ";
    std::cout << "Accel Y: " << accel.acc_xyz[1] << "m/s^2, ";
    std::cout << "Accel Z: " << accel.acc_xyz[2] - gravity << "m/s^2" << std::endl;
}



void read_Temperature(void){
    uint8_t reg_temp_l = OUT_TEMP_L; 
    uint8_t temp_data[2];

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_temp_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, temp_data, 2, false);

    int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);

    float temperature = (raw_temp / 256.0) + 25;
    std::cout << "Temperature: " << temperature << "°C" << std::endl;
}
