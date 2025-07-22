#include <iostream>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" //Wifi chip librar
#include "hardware/i2c.h" //I2C
// #include "IMUsensor.h"

// This is code for bachelor project Rocket stabilization system. The code is written in C++ and
// developed by SDU students: Aksel Møller-Hansen, Harald Bay Nielsen og Sebastian Piessenberger.
// This following code is for the Raspberry Pi Pico WH board using accelerometer and Gyroscope
// of the ST-LSM6DS3 IMU sensor. (datasheet: https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)

#define I2C_PORT i2c0 //Defines whether using i2c 1 or -2.

//Which GPIO ports on PICO
#define SDA_PIN 0
#define SCL_PIN 1

#define IMU_ADDR 0x6A // Default I2C address for LSM6DS3

#define WHO_AM_I_REG 0x0F // Register for WHO_AM_I value/ID
#define WHO_AM_I_VALUE 0x69 // Expected device ID for LSM6DS3

#define I2C_BAUDRATE 400*1000 // Initialize I2C at 400 kHz

#define OUT_TEMP_L 0x20 //Output register for temperature //Only the low (OUT_TEMP_L) bc reads two bytes

#define OUTX_L_G 0x22 //Address for gyro //Anmod 6 byte frem

#define OUTX_L_XL 0x28 //Address for accelerometer



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
    config[1] = 0b10000100;  //1.66kHz, +-16g (high acceleration, but low resolution), 400Hz anti-aliasing
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    // Enable Gyroscope (104Hz, 2000 dps)
    config[0] = 0x11;  // CTRL2_G register
    config[1] = 0b10001000;  // 1.66kHz, 1000dps
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    // Enable Block Data Update & Auto-increment (CTRL3_C)
    config[0] = 0x12;  // CTRL3_C register
    config[1] = 0b01000100;  // Block Date Update (until MSB & LSB hav been read) = 1, Auto-increment enabled (Register address automaticcaly updated during multiple byte access) = 1
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    //Enable timestamp
    config[0] = 0x19;  // CTRL10_C register
    config[1] = 0b00100000;  // Set TMD bit (bit 5) to enable timestamp
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);

    config[0] = 0x5C; // WAKE_UP_DUR register
    config[1] = 0b00010000; //Resolution...
    i2c_write_blocking(I2C_PORT, IMU_ADDR, config, 2, false);
    
}


void readTemperature(void){
    uint8_t reg_temp_l = OUT_TEMP_L; 
    uint8_t temp_data[2];

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_temp_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, temp_data, 2, false);

    int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);

    float temperature = (raw_temp / 256.0) + 25;
    std::cout << "Temperature: " << temperature << "°C" << std::endl;
}



void readGyroscope(void){
    uint8_t reg_temp_l = OUTX_L_G;
    uint8_t temp_data[6];

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_temp_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, temp_data, 6, false);

    int16_t GX = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    int16_t GY = (int16_t)((temp_data[3] << 8) | temp_data[2]);
    int16_t GZ = (int16_t)((temp_data[5] << 8) | temp_data[4]);

    std::cout << "X: " << GX << "  |  Y: " << GY << "  |  Z: " << GZ << std::endl;
}

void readAccelerometer(void){
    uint8_t reg_temp_l = OUTX_L_XL;
    uint8_t temp_data[6];

    i2c_write_blocking(I2C_PORT, IMU_ADDR, &reg_temp_l, 1, true);
    i2c_read_blocking(I2C_PORT, IMU_ADDR, temp_data, 6, false);

    int16_t XLX = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    int16_t XLY = (int16_t)((temp_data[3] << 8) | temp_data[2]);
    int16_t XLZ = (int16_t)((temp_data[5] << 8) | temp_data[4]);

    std::cout << "X: " << XLX << "  |  Y: " << XLY << "  |  Z: " << XLZ << std::endl;
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


int main()
{
    //Initialise Input/Output
    stdio_init_all();


    //Delay for startup
    sleep_ms(7500);

    // Initialize the CYW43439 Wi-Fi chip (required for controlling the onboard LED)
    if (cyw43_arch_init()) {
        std::cout << "Failed to initialize CYW43 chip\n" << std::endl;
        return 1;  // Exit if initialization fails
    }

    //Variable for input
    char userInput;

    //Init I2C
    while(true){
        printf("Press 'i' to initialize the IMU\n");
        std::cin >> userInput;
        if(userInput == 'i'){
            break;
        }
    }
    lsm6ds3_init();

    //Configure LSM6DS3
    configure_LSM6DS3();

    //Read temp
    resetTimestamp();
    while(true){
        // readGyroscope();
        // readAccelerometer();
        readTimestamp();
        sleep_ms(100);
    }


    // //Main loop
    // while (true) {
    //     std::cout << "\nInput for LED: 1 -> On  |  0 -> Off\n" << std::endl;
    //     std::cin >> userInput;

    //     if(userInput == '1'){
    //         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);  // Turn LED on
    //         std::cout << "LED ON!\n" << std::endl;
    //     }
    //     else if(userInput == '0'){
    //         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);  // Turn LED off
    //         std::cout << "LED OFF!\n" << std::endl;

    //     }
    //     else{
    //         std::cout << "Not valid input\n" << std:: endl;
    //     }
    // }

    cyw43_arch_deinit();
    return 0;
}
