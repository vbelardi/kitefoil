import struct
import csv

# Nouveau format binaire basé sur la structure C fournie
IMU_DATA_FORMAT = "<I24sff9f6i"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FORMAT)

def binary_to_csv(binary_file, csv_file):
    """
    Convertit un fichier binaire contenant des données IMU, capteurs de force et GPS avec deux timestamps en CSV.
    
    Args:
        binary_file (str): Chemin du fichier binaire en entrée.
        csv_file (str): Chemin du fichier CSV de sortie.
    """
    with open(binary_file, "rb") as bin_file, open(csv_file, "w", newline="") as csv_out:
        csv_writer = csv.writer(csv_out, delimiter=';')

        # Écriture de l'en-tête du fichier CSV
        csv_writer.writerow([
            "CPU Timestamp (ms)", "Timestamp GPS",
            "Latitude", "Longitude",
            "Euler_X (deg)", "Euler_Y (deg)", "Euler_Z (deg)",
            "Gyro_X (rad/s)", "Gyro_Y (rad/s)", "Gyro_Z (rad/s)",
            "Accel_X (m/s^2)", "Accel_Y (m/s^2)", "Accel_Z (m/s^2)",
            "LoadCell_1", "LoadCell_2", "LoadCell_3", "LoadCell_4", "LoadCell_5", "LoadCell_6"
        ])

        # Lecture et conversion des données binaires
        while True:
            record = bin_file.read(IMU_DATA_SIZE)
            if not record:
                break  # Fin du fichier

            if len(record) < IMU_DATA_SIZE:
                print("Enregistrement incomplet. Ignoré.")
                continue

            # Décodage des données binaires
            unpacked_data = struct.unpack(IMU_DATA_FORMAT, record)

            # Extraction des valeurs
            cpu_timestamp = unpacked_data[0]  # uint32_t (millis)
            timestamp_gps = unpacked_data[1].decode("utf-8").strip("\x00")  # char[24]
            latitude = unpacked_data[2]  # float
            longitude = unpacked_data[3]  # float
            imu_data = list(unpacked_data[4:13])  # 9 floats (IMU)
            load_cells = list(unpacked_data[13:])  # 6 int32 (capteurs de force)

            # Écriture des données dans le fichier CSV
            row = [cpu_timestamp, timestamp_gps, latitude, longitude] + imu_data + load_cells
            csv_writer.writerow(row)

    print(f"Conversion terminée. Données enregistrées dans {csv_file}")

# Exemple d'utilisation
binary_file_path = "_imu_log_0008.bin"  # Remplace par le chemin réel du fichier binaire
csv_file_path = "imu_log_0008.csv"     # Remplace par le chemin souhaité du fichier CSV
binary_to_csv(binary_file_path, csv_file_path)
