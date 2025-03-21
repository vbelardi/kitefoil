import csv
import numpy as np

def process_loadcells(input_csv, output_csv):
    """
    Lit le fichier CSV issu de binary_to_csv, calcule les forces et moments
    à partir des valeurs des loadcells, et écrit les résultats dans un nouveau CSV.
    
    Les colonnes attendues dans le CSV d'entrée sont :
    "CPU_Timestamp_ms", "GPS_Timestamp",
    "Latitude", "Longitude",
    "Euler_X_deg", "Euler_Y_deg", "Euler_Z_deg",
    "Gyro_X_rad_s", "Gyro_Y_rad_s", "Gyro_Z_rad_s",
    "Accel_X_m_s2", "Accel_Y_m_s2", "Accel_Z_m_s2",
    "LoadCell_1", "LoadCell_2", "LoadCell_3",
    "LoadCell_4", "LoadCell_5", "LoadCell_6"
    
    Le fichier de sortie contiendra, pour chaque enregistrement, le temps (en secondes),
    les forces totales (avant et arrière), les moments totaux (M_tot_X, M_tot_Y),
    les moments spécifiques (avant et arrière) et les positions de réaction (calculées lorsque
    la force dépasse un seuil).
    
    Args:
        input_csv (str): Chemin du fichier CSV d'entrée.
        output_csv (str): Chemin du fichier CSV de sortie.
    """
    # Lecture des données CSV
    data = []
    with open(input_csv, "r", newline="") as f:
        reader = csv.reader(f, delimiter=';')
        header = next(reader)  # lecture de l'en-tête
        for row in reader:
            data.append(row)
    data = np.array(data)
    
    # Extraction du temps et conversion de ms à s

    
    time = data[:, 0].astype(float) / 1000.0
    
    # Extraction des valeurs des loadcells (colonnes 13 à 18)
    # Indices : 0=CPU_Timestamp_ms, 1=GPS_Timestamp, 2=Latitude, 3=Longitude, 
    # 4-6=Euler, 7-9=Gyro, 10-12=Accel, 13-18=LoadCells.

    
    forces = data[:, 13:19].astype(float)
    

    # Calcul des forces totales sur l'avant et l'arrière
    F_front = np.sum(forces[:, 3:6], axis=1)
    F_back  = np.sum(forces[:, 0:3], axis=1)
    # g1 = distance entre l'axe de symetrie de la planche en longueur et les deux loadcell avant non centrées (en mm)
    g1 = 139.5
    # g2 = distance entre l'axe de symetrie de la planche en longueur et les deux loadcell arrières non centrées(en mm)
    g2 = 106.5  

    #mauvaises valeurs
    e1 = 276.5  
    e2 = 539    
    e3 = 294    
    e4 = 464
    
    # Calcul du moment total autour de l'axe X et Y
    M_tot_X = (g1 * forces[:, 5] + g2 * forces[:, 1] - g1 * forces[:, 4] - g2 * forces[:, 0]) / 1000.0
    
    #mauvaises valeurs
    M_tot_Y = (e1 * (forces[:, 1] + forces[:, 2]) + e2 * forces[:, 0] - e3 * (forces[:, 4] + forces[:, 5]) - e4 * forces[:, 3]) / 1000.0
    

    # Calcul des moments spécifiques pour la partie avant et arrière

    # moitié de la distance entre les loadcells avant en longueur
    h1 = 90.8
    #moitié de la distance entre les loadcells arrière en longueur
    h2 = 48.5
    
    # Moments pour la partie avant
    M_front_X = (g1 * (forces[:, 5] - forces[:, 4])) / 1000.0
    M_front_Y = h1 * (forces[:, 5] + forces[:, 4] - forces[:, 3]) / 1000.0
    
    # Moments pour la partie arrière
    M_back_X = (g2 * (forces[:, 1] - forces[:, 0])) / 1000.0
    M_back_Y = h2 * (forces[:, 1] + forces[:, 0] - forces[:, 2]) / 1000.0
    
    # Calcul des positions de réaction (si la force est supérieure à un seuil)
    threshold = 100  # seuil en N

    # Initialisation avec NaN pour les enregistrements où la force est insuffisante
    P_front_X = np.full_like(F_front, np.nan)
    P_front_Y = np.full_like(F_front, np.nan)
    P_back_X  = np.full_like(F_back, np.nan)
    P_back_Y  = np.full_like(F_back, np.nan)
    
    # Partie avant
    mask_front = F_front > threshold
    P_front_X[mask_front] = (M_front_Y[mask_front] * 1000.0) / F_front[mask_front]
    P_front_Y[mask_front] = (-M_front_X[mask_front] * 1000.0) / F_front[mask_front]
    
    # Partie arrière
    mask_back = F_back > threshold
    P_back_X[mask_back] = (M_back_Y[mask_back] * 1000.0) / F_back[mask_back]
    P_back_Y[mask_back] = (-M_back_X[mask_back] * 1000.0) / F_back[mask_back]
    
    # Écriture des résultats dans un nouveau CSV
    with open(output_csv, "w", newline="") as fout:
        writer = csv.writer(fout, delimiter=';')
        writer.writerow([
            "Time_s", "F_front", "F_back",
            "M_tot_X", "M_tot_Y",
            "M_front_X", "M_front_Y",
            "M_back_X", "M_back_Y",
            "P_front_X", "P_front_Y",
            "P_back_X", "P_back_Y"
        ])
        
        # Boucle sur chaque enregistrement pour écrire les valeurs calculées
        for i in range(len(data)):
            writer.writerow([
                time[i],
                F_front[i], F_back[i],
                M_tot_X[i], M_tot_Y[i],
                M_front_X[i], M_front_Y[i],
                M_back_X[i], M_back_Y[i],
                P_front_X[i], P_front_Y[i],
                P_back_X[i], P_back_Y[i]
            ])
    
    print(f"Traitement terminé. Données enregistrées dans {output_csv}")

# Exemple d'utilisation
if __name__ == "__main__":
    input_csv_file = "imu_log_0042.csv"      # CSV issu de binary_to_csv
    output_csv_file = "donnees_moments_forces.csv"   # CSV de sortie avec moments et forces
    process_loadcells(input_csv_file, output_csv_file)
