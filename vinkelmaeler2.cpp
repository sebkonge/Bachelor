#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <serial/serial.h>

// Constants
const std::string SERIAL_PORT = "COM6";  // Change this to match your port
const unsigned long BAUD_RATE = 3000000;
const std::string CSV_FILENAME = "data_log.csv";
const double ANGLE_RESOLUTION = 8196.0 * 256.0;  // 360 degrees
const int SAMPLE_RATE = 15000;  // 15 kHz
const double TIME_STEP = 1.0 / SAMPLE_RATE;  // Time per sample

// Function to convert a 32-bit unsigned value to signed
int32_t convertToSigned32Bit(uint32_t value) {
    if (value > 0x7FFFFFFF) {  // If larger than max 32-bit signed int
        value -= 0x100000000;  // Apply two's complement conversion
    }
    return static_cast<int32_t>(value);
}

void readSerialData() {
    serial::Serial ser;
    
    try {
        // Open serial port
        ser.setPort(SERIAL_PORT);
        ser.setBaudrate(BAUD_RATE);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();

        if (!ser.isOpen()) {
            std::cerr << "Failed to open serial port!" << std::endl;
            return;
        }

        std::cout << "Logging data to " << CSV_FILENAME << "..." << std::endl;

        // Open CSV file for writing
        std::ofstream file(CSV_FILENAME);
        if (!file.is_open()) {
            std::cerr << "Failed to open CSV file!" << std::endl;
            return;
        }

        file << "Time (s),Angle (°)\n";  // Write header

        double simulatedTime = 0.0;  // Time counter

        while (true) {
            try {
                std::string line = ser.readline();  // Read a line from serial
                line.erase(line.find_last_not_of(" \n\r\t") + 1);  // Trim whitespace

                if (!line.empty()) {
                    simulatedTime += TIME_STEP;  // Increment time

                    std::istringstream iss(line);
                    std::string token1, token2;

                    if (!(iss >> token1 >> token2)) {
                        continue;  // Ignore invalid lines
                    }

                    uint32_t rawAngle;
                    std::stringstream ss;
                    ss << std::hex << token2;
                    ss >> rawAngle;

                    // Convert to signed and to degrees
                    int32_t signedAngle = convertToSigned32Bit(rawAngle);
                    double angleDegrees = (signedAngle / ANGLE_RESOLUTION) * 360.0;

                    // Format and print
                    std::cout << std::fixed << std::setprecision(6)
                              << "Time: " << simulatedTime << " s, Angle: " << angleDegrees << "°" << std::endl;

                    // Write to CSV file
                    file << std::fixed << std::setprecision(6)
                         << simulatedTime << "," << angleDegrees << "\n";
                    file.flush();
                }
            } catch (const std::exception &e) {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }
    } catch (const serial::IOException &e) {
        std::cerr << "Serial connection error: " << e.what() << std::endl;
    }
}

int main() {
    readSerialData();
    return 0;
}
