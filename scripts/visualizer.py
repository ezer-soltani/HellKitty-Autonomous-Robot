import serial
import matplotlib.pyplot as plt
import numpy as np

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'  # À CHANGER: ex: 'COM3' sur Windows, '/dev/tty.usbserial-XXXX' sur macOS
BAUD_RATE = 115200
# ---------------------

# Initialisation du plot
plt.ion() # Mode interactif ON
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)
ax.set_rlabel_position(-90)
ax.set_theta_zero_location("N") # 0° en haut
ax.set_theta_direction(-1) # Sens horaire
line, = ax.plot([], [], 'b.', markersize=2) # 'b.' = points bleus

# Limites du graph, rmax est la portée maximale en mm
ax.set_rmax(8000) 

def update_plot(scan_points):
    """Met à jour le graphique avec les nouveaux points."""
    if not scan_points:
        return
    
    angles_rad = [p[0] * np.pi / 180.0 for p in scan_points]
    distances = [p[1] for p in scan_points]
    
    line.set_data(angles_rad, distances)
    fig.canvas.draw()
    fig.canvas.flush_events()

def run_visualizer():
    """Fonction principale pour lire le port série et mettre à jour le plot."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connexion au port série {SERIAL_PORT}...")
    except serial.SerialException as e:
        print(f"Erreur: Impossible d'ouvrir le port série {SERIAL_PORT}.")
        print(f"Message: {e}")
        print("Vérifiez le nom du port et assurez-vous que l'appareil est connecté.")
        return

    current_scan = []
    
    while True:
        try:
            line_read = ser.readline().decode('utf-8').strip()

            if not line_read:
                continue

            if line_read == "SYNC":
                # Un nouveau scan commence, on affiche le précédent
                # et on réinitialise la liste
                print(f"Scan complet reçu, {len(current_scan)} points.")
                update_plot(current_scan)
                current_scan = []
                continue
            
            # Parsing "angle,distance"
            parts = line_read.split(',')
            if len(parts) == 2:
                angle = float(parts[0])
                distance = float(parts[1])
                # On ne garde que les points valides
                if distance > 0:
                    current_scan.append((angle, distance))

        except KeyboardInterrupt:
            print("Arrêt du programme.")
            break
        except ValueError:
            print(f"Donnée invalide reçue: {line_read}")
            continue
        except Exception as e:
            print(f"Une erreur est survenue: {e}")
            break

    ser.close()

if __name__ == '__main__':
    run_visualizer()
