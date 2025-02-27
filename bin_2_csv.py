import struct
import csv

# Define the binary structure format
# "24s" is for the 24-character timestamp (char[24])
# "fffffffff" for 9 floats (IMU data)
# "6i" for 6 signed 32-bit integers (load cell data) (was previously "6I")
IMU_DATA_FORMAT = "<24sfffffffff6i"  # Use little-endian ("<") and signed integers ("i")
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FORMAT)

def binary_to_csv(binary_file, csv_file):
    """
    Converts a binary file containing IMU data with a string timestamp and 6 signed load cell values into a CSV file.
    
    Args:
        binary_file (str): Path to the input binary file.
        csv_file (str): Path to the output CSV file.
    """
    with open(binary_file, "rb") as bin_file, open(csv_file, "w", newline="") as csv_out:
        csv_writer = csv.writer(csv_out, delimiter=';')
        
        # Write the CSV header
        csv_writer.writerow([
            "Timestamp",
            "Euler_X (deg)", "Euler_Y (deg)", "Euler_Z (deg)",
            "Gyro_X (rad/s)", "Gyro_Y (rad/s)", "Gyro_Z (rad/s)",
            "Accel_X (m/s^2)", "Accel_Y (m/s^2)", "Accel_Z (m/s^2)",
            "LoadCell_1", "LoadCell_2", "LoadCell_3", "LoadCell_4", "LoadCell_5", "LoadCell_6"
        ])
        
        # Read and parse binary data
        while True:
            record = bin_file.read(IMU_DATA_SIZE)
            if not record:
                break  # End of file

            # Ensure the record has the expected size
            if len(record) < IMU_DATA_SIZE:
                print("Incomplete record. Skipping.")
                continue

            # Unpack the binary data
            unpacked_data = struct.unpack(IMU_DATA_FORMAT, record)
            
            # Convert the timestamp from bytes to a string and strip any null characters
            timestamp = unpacked_data[0].decode("utf-8").strip("\x00")
            
            # Prepare the row with the timestamp, IMU data, and load cell values
            imu_data = list(unpacked_data[1:10])  # IMU data (9 floats)
            load_cells = list(unpacked_data[10:])  # Load cell values (6 signed int32)
            row = [timestamp] + imu_data + load_cells
            
            # Write the unpacked data to the CSV file
            csv_writer.writerow(row)

    print(f"Conversion complete. Data saved to {csv_file}")

# Example usage
binary_file_path = "_imu_log_0005.bin"  # Replace with the actual binary file path
csv_file_path = "imu_log_0005.csv"     # Replace with the desired CSV file path
binary_to_csv(binary_file_path, csv_file_path)
