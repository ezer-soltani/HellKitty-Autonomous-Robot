# Analyse et État d'Avancement : Driver Accéléromètre (ADXL343)
**Date de mise à jour :** 09 Décembre 2025
**Statut :** FONCTIONNEL (Intégré dans FreeRTOS)

---

## 1. État Actuel
Le driver de l'accéléromètre est opérationnel et intégré dans le firmware final du robot.
*   **Initialisation :** OK (Message "IMU Init OK" visible).
*   **Communication I2C :** OK (Lecture des IDs et des registres).
*   **Fonctionnalité :** Détection de chocs (Tap Detection) active.
*   **OS :** Tourne dans une tâche FreeRTOS dédiée (`vImuTask`).

## 2. Architecture Implémentée

### A. Driver Bas Niveau (`Core/Drivers/imu.c`)
Le code a été refactorisé pour être "stateless" et thread-safe (via l'appelant).
*   **Structure :** Utilise `HAL_I2C_Mem_Read/Write` avec des timeouts (pas de blocage infini).
*   **Configuration :**
    *   Range : ±2g (Full Resolution).
    *   Data Rate : 100 Hz.
    *   **Tap Threshold :** 2.5g (Actuellement codé en dur).
    *   **Tap Duration :** 10ms.

### B. Intégration FreeRTOS (`Core/Src/app_freertos.c`)
*   **Tâche :** `vImuTask`
*   **Priorité :** `osPriorityBelowNormal` (Priorité 2) - Inférieure au Lidar pour ne pas bloquer la navigation.
*   **Protection :**
    *   **`xI2C1Mutex`** : Crucial. Protège l'accès au bus I2C1 car il sera partagé plus tard avec les capteurs de distance (TOF).
    *   **`xUARTMutex`** : Utilisé pour les `printf` afin d'éviter d'écraser les données du Lidar dans la console.

## 3. Problèmes Rencontrés et Résolus

### Conflit d'affichage UART (Lidar vs IMU)
*   **Symptôme :** Le message "!!! SHOCK DETECTED !!!" spamait la console, empêchant le Lidar d'afficher ses objets.
*   **Cause :** La tâche IMU prenait la main trop souvent et le Lidar n'attendait pas la libération du Mutex UART.
*   **Correction :** 
    1.  Ajout d'un `portMAX_DELAY` (attente infinie) dans la tâche Lidar pour garantir qu'elle finisse par écrire.
    2.  Ajout d'un délai (100ms) dans la tâche IMU.

## 4. Prochaines Étapes (Roadmap)

### Étape A : Réglage de la Sensibilité (URGENT)
Le capteur est actuellement trop sensible ("spam" de détections de chocs).
*   **Action :** Modifier `ADXL343_ConfigShock` dans `app_freertos.c` ou `imu.c`.
*   **Valeurs à tester :** Augmenter le seuil de 2.5g à **3.5g** ou **4g** pour ne détecter que les vrais impacts contre les murs, pas les vibrations des moteurs.

### Étape B : Optimisation par Interruption (Hardware)
Actuellement, la tâche fait du "Polling" (elle demande au capteur "as-tu vu un choc ?" toutes les 100ms).
*   **Objectif :** Utiliser la pin **INT1** du capteur (reliée à `PB13` sur le PCB).
*   **Implémentation :** 
    1.  Configurer `PB13` en mode `GPIO_EXTI`.
    2.  Utiliser un Sémaphore Binaire (`xSemaphoreGiveFromISR`) dans le callback d'interruption.
    3.  La tâche `vImuTask` restera bloquée (`xSemaphoreTake`) tant qu'il n'y a pas de choc (0% CPU usage).

### Étape C : Calcul d'Inclinaison (Fonctionnalité)
Ajouter une fonction pour détecter si le robot va se renverser.
*   **Calcul :** Utiliser `atan2` sur les axes Y et Z pour obtenir l'angle de tangage (Pitch).
*   **Utilité :** Arrêt d'urgence si angle > 45°.

---
**Note pour la reprise :** Commencer par l'**Étape A**. Ouvrir `app_freertos.c` et ajuster les paramètres de `ADXL343_ConfigShock`.
