import serial
import csv

# Set up the serial connection (Change 'COM3' to match your port)
ser = serial.Serial(port='COM6', baudrate=3000000, timeout=1)

# Constants
ANGLE_RESOLUTION = 8196 * 256  # 360 degrees
SAMPLE_RATE = 15000  # 15 kHz
TIME_STEP = 1 / SAMPLE_RATE  # Time per sample

# Start time counter
simulated_time = 0.0  # We manually increment this

# Open CSV file for writing
csv_filename = "data_log.csv"
with open(csv_filename, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Angle (°)"])  # Write clean header

def convert_to_signed_32bit(value):
    """ Converts a 32-bit unsigned value to signed if needed. """
    if value > 0x7FFFFFFF:  # If larger than max 32-bit signed int
        value -= 0x100000000  # Apply two's complement conversion
    return value

def read_serial_data():
    global simulated_time  # Use the manual time counter

    # Open CSV file for appending data
    with open(csv_filename, mode='a', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)

        while True:
            try:
                # Read a line from the serial monitor
                line = ser.readline().decode('utf-8').strip()
                
                if line:
                    # Increment the simulated time by the sample interval
                    simulated_time += TIME_STEP

                    # Split the received data into two hex values
                    data = line.split()
                    if len(data) == 2:
                        raw_angle = int(data[1], 16)  # Convert angle from hex to int

                        # Convert to signed 32-bit
                        signed_angle = convert_to_signed_32bit(raw_angle)

                        # Convert to degrees
                        angle_degrees = (signed_angle / ANGLE_RESOLUTION) * 360

                        # Format time to 6 decimal places
                        formatted_time = f"{simulated_time:.6f}"
                        formatted_angle = f"{angle_degrees:.6f}"

                        # Print to console
                        print(f"Time: {formatted_time} s, Angle: {formatted_angle}°")

                        # Save to CSV file
                        writer.writerow([formatted_time, formatted_angle])
                        file.flush()  # Ensure data is written immediately

            except KeyboardInterrupt:
                print("Exiting...")
                break
            except Exception as e:
                print("Error:", e)

if __name__ == "__main__":
    print(f"Logging data to {csv_filename}...")
    read_serial_data()
    ser.close()  # Close the serial connection