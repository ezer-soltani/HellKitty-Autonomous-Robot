# Contexte du Projet : Firmware de Robot Mobile (HellKitty)

Tu es un expert Senior en Systèmes Embarqués. Tu assistes Ezer sur le projet **HellKitty**, un robot de combat autonome pour un jeu de **Chat et Souris** sur table (Projet ENSEA 3A ESE).

---

## 1. Spécifications Matérielles (Hardware)

*   **Microcontrôleur :** STM32G431CBU6 (Cortex-M4 170MHz, FPU activée).
*   **PCB :** Carte custom 4 couches.
*   **Capteurs de Distance (Vide) :** 4x VL53L0X (Time-of-Flight) sur bus I2C.
    *   *Configuration :* **Mode Polling (20ms)**. Initialisation séquentielle via broches `XSHUT`.
*   **Vision (Ennemi) :** LIDAR YDLIDAR X2 (UART/DMA).
*   **IMU :** ADXL343 (Accéléromètre) pour détection de chocs/collisions (> 3.5G).
*   **Connectivité :** Module Bluetooth HC-05 (UART3, 9600 bauds) pour Télécommande et Logs.
*   **Actionneurs :** 2x Moteurs DC avec encodeurs quadratiques (2048 ticks/tour).

---

## 2. Architecture Logicielle (FreeRTOS)

Le système est critique et temps réel. L'ordonnancement est strict :

| Tâche | Priorité | Fréquence | Rôle |
| :--- | :---: | :---: | :--- |
| **vSafetyTask** | ⭐⭐⭐ (MAX) | **100Hz (10ms)** | **Polling** des 4 TOF. Si Vide (>500mm), Override avec séquence de Pivot. |
| **vControlTask** | ⭐⭐ (Med) | 100Hz | Gestion Stratégie, PID Vitesse et Odométrie. |
| **vLidarTask** | ⭐⭐ (Med) | 10Hz | Parsing DMA, Segmentation Objets, Tracking Cible. |
| **vImuTask** | ⭐ (Low) | 20Hz | Surveillance des chocs pour changement de rôle. |

---

## 3. Algorithmes Critiques

### A. Pilotage & Asservissement
*   **PID Vitesse :** `Kp=5.0`, `Ki=20.0`, `Kd=0.0`.
*   **Cinématique :** Conversion Vitesse Linéaire/Angulaire $\leftrightarrow$ Vitesse Roues.
*   **FeedForward (Prévu) :** Ajout d'un offset statique et d'un gain proportionnel pour compenser les frottements et l'inertie.

### B. Stratégie (Machine à États FSM)
*   **Rôles :** 🐱 **CHAT** / 🐭 **SOURIS** (Basculement sur choc IMU).
*   **États :** `SEARCH`, `ATTACK`, `FLEE`.
*   **Tracking :** Filtre Alpha ($\alpha=0.3$) et mémoire de 1s.

### C. Perception LIDAR
*   **Segmentation :** $Largeur = 2 \cdot Distance \cdot \tan(\frac{\Delta Angle}{2})$
*   **Clustering (En cours) :** Conversion cartésienne (X,Y) et fusion des clusters proches (`MERGE_THRESHOLD = 150mm`) pour une meilleure robustesse au bruit.

### D. Sécurité Anti-Chute
*   **Séquence Pivot :** 
    1. **STOP** (500ms) 
    2. **Rotation** (-50/50, 100ms) 
    3. **Avance** (50/50, 1000ms) pour s'éloigner du bord.

---

## 4. État d'Avancement (au 14 Jan 2026)

### ✅ Terminés & Validés
1.  **Drivers Bas Niveau :** Tous capteurs + Moteurs.
2.  **Architecture OS :** FreeRTOS stable, FPU activée.
3.  **Asservissement :** PID réglé.
4.  **Sécurité (Nouvelle version) :** 
    *   Passage des TOF en mode **Polling (50Hz)** pour une meilleure fiabilité (suppression des EXTI).
    *   Timing Budget réduit à **20ms** pour une réactivité immédiate.
    *   Implémentation de la **séquence de Pivot** validée.

### ⚠️ Problèmes Identifiés (A corriger)
1.  **Instabilité Capteurs :** Lors du mouvement, des **faux positifs** apparaissent aléatoirement :
    *   **TOF :** Détection de "vide" furtive (bruit de mesure ?).
    *   **IMU :** Détection de "choc" intempestive (vibrations moteurs ?).

### 📅 Perspectives (Prochaine Session)
1.  **Filtrage :** Ajouter un filtre (moyenne glissante ou debounce) sur les capteurs TOF et IMU pour éliminer les faux positifs.
2.  **Amélioration Lidar :** Implémenter le clustering cartésien.
3.  **Optimisation Moteur :** Ajouter le FeedForward.

---
**Note pour l'agent :** Ce fichier est la référence absolue. Le code est propre et prêt pour la suite des améliorations.
