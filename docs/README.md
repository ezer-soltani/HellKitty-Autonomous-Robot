# 🐱 **HellKitty - Projet Robot Chat & Souris**

![STM32](https://img.shields.io/badge/STM32-G431-blue?style=for-the-badge&logo=stmicroelectronics)
![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green?style=for-the-badge)
![Bluetooth](https://img.shields.io/badge/Connectivity-HC--05-blueviolet?style=for-the-badge&logo=bluetooth)
[![ENSEA](https://img.shields.io/badge/ENSEA-3A--ESE-red?style=for-the-badge&logo=https://upload.wikimedia.org/wikipedia/fr/8/82/Logo_ENSEA.svg)](https://www.ensea.fr)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

---

## 📜 **Description**
**HellKitty** est un robot mobile autonome conçu pour la compétition de "Chat et Souris".
🎯 **Objectif :** Survivre sur une table sans bordure, détecter l'adversaire via un LIDAR, et alterner dynamiquement entre les rôles de **Prédateur** (Chat) et de **Proie** (Souris) en fonction des chocs reçus.

Ce projet est réalisé dans le cadre de la dernière année de la filière **Électronique et Systèmes Embarqués (ESE)** de l'ENSEA.

### **Les contributeurs :**
- 👨‍💻 **Serine**
- 👨‍💻 **Aymen**
- 👨‍💻 **Ezer**

---

## 📚 **Table des matières**
1. [📖 Contexte & Règles](#-contexte--règles)
2. [📐 Architecture Système](#-architecture-système)
3. [🧠 Stratégie & IA](#-stratégie--ia)
4. [📡 Connectivité & Debug](#-connectivité--debug)
5. [📚 Explication Technique](#-explication-technique)
6. [🔧 Matériel utilisé](#-matériel-utilisé)
7. [🚀 État d'avancement](#-état-davancement)

---

## 📖 **Contexte & Règles**
Le défi consiste à concevoir un système embarqué complet (Hardware + Firmware) capable de :
1.  **Ne jamais tomber** de la table (Sécurité critique).
2.  **Localiser** un adversaire dynamique via LIDAR.
3.  **Changer de rôle** : Le robot commence en "Chat". S'il subit une collision (détectée par IMU), il devient "Souris" et doit fuir.

---

## 📐 **Architecture Système**

### **Architecture Logicielle (FreeRTOS)**
Le firmware repose sur un OS temps réel pour garantir la réactivité et la ségrégation des tâches critiques.

| Tâche | Priorité | Fréquence | Rôle |
| :--- | :---: | :---: | :--- |
| **vSafetyTask** | ⭐⭐⭐ (High) | *Event (EXTI)* | Activée par interruption TOF. Arrêt d'urgence et manœuvre de recul prioritaire. |
| **vControlTask** | ⭐⭐ (Med) | 100Hz | Gestion de la stratégie, Tracking cible, PID Moteur et Odométrie. |
| **vLidarTask** | ⭐⭐ (Med) | 10Hz | Parsing DMA, Filtrage géométrique et Tracking temporel de l'ennemi. |
| **vImuTask** | ⭐ (Low) | 20Hz | Détection de chocs (Tap detection) pour le changement de rôle. |

---

## 🧠 **Stratégie & IA**

Le robot utilise une **Machine à États Finis (FSM)** complexe couplée à un **Tracking Temporel** de la cible.

### **1. Les Rôles (Game Logic)**
*   **🐱 CHAT (Led Verte) :** Le robot cherche activement l'ennemi pour le percuter.
*   **🐭 SOURIS (Led Rouge) :** Le robot cherche l'ennemi pour le fuir (s'orienter à l'opposé).
*   **🔄 Basculement :** Un choc > 3.5G détecté par l'accéléromètre (ADXL343) inverse instantanément le rôle (avec anti-rebond de 2s).

### **2. Les États Comportementaux**
*   **SEARCH (Recherche) :** Rotation sur place (~3 rad/s) pour scanner l'environnement à 360°.
*   **ATTACK (Poursuite) :**
    *   Verrouillage de la cible.
    *   Correction angulaire (PID) pour rester face à l'ennemi.
    *   Vitesse linéaire : **200 mm/s**.
*   **FLEE (Fuite) :**
    *   Calcul du vecteur opposé à la menace (Cible à 180°).
    *   Demi-tour rapide (6 rad/s).
    *   Fuite en ligne droite à **250 mm/s**.

### **3. Target Tracking (Mémoire)**
Pour éviter les mouvements erratiques, le robot utilise un filtre Alpha ($\alpha=0.3$) sur la position de l'ennemi et garde en mémoire la dernière position connue pendant **1 seconde** si le LIDAR perd le visuel.

---

## 📡 **Connectivité & Debug**

Le robot intègre un module Bluetooth **HC-05** redirigeant la sortie standard (`printf`).

### **Commandes Distantes**
Il est possible de piloter le robot depuis un smartphone/PC (115200 bauds) :
*   `START` : Active les moteurs et la stratégie.
*   `STOP` : Arrêt d'urgence logiciel (Moteurs à 0).
*   `CHAT` : Force le mode Chasseur.
*   `SOURIS` : Force le mode Proie.

### **Logs en Temps Réel**
Le robot renvoie son état interne :
> `Strategy: Target Found! Switching to ATTACK`
> `IMU: SHOCK DETECTED! Toggling Role.`
> `Safety: VOID DETECTED! Evacuating...`

---

## 📚 **Explication Technique**

### 1. 👁️ Perception LIDAR (YDLIDAR X2)
Nous n'utilisons pas simplement les données brutes. Le driver implémente un pipeline de traitement avancé :
*   **Correction Géométrique :** Compensation de la distorsion angulaire liée à la rotation.
*   **Segmentation :** Calcul de la **largeur physique** des objets ($L = 2 \cdot d \cdot \tan(\theta/2)$).
    *   Seuls les objets de **5cm à 30cm** sont considérés comme des robots.
    *   Les murs et pieds de table sont ignorés.

### 2. 🛡️ Barrière Immatérielle (4x VL53L0X)
Quatre capteurs TOF sont disposés aux coins du robot.
*   **Override Hardware :** Si $d > 500mm$ (vide), une interruption déclenche `vSafetyTask`.
*   Cette tâche **bloque** toutes les commandes de la stratégie pour exécuter une séquence de sauvetage inviolable.

### 3. ⚙️ Asservissement & Odométrie
*   **PID Vitesse :** $Kp=5.0, Ki=20.0$. Réponse rapide (< 100ms) sans dépassement.
*   **Odométrie :** Suivi de position ($x, y, \theta$) par intégration de Runge-Kutta.

---

## 🔧 **Matériel utilisé**

*   **MCU :** STM32G431CBU6 (Cortex-M4 170MHz, FPU).
*   **LIDAR :** YDLIDAR X2 (UART/DMA).
*   **TOF :** 4x VL53L0X (I2C) adressés dynamiquement.
*   **IMU :** ADXL343 (I2C) pour les chocs.
*   **Coms :** Module Bluetooth HC-05.
*   **Moteurs :** DC avec encodeurs quadratiques (2048 ticks/tour).

---

## 🚀 **État d'avancement**

### ✅ Terminés & Validés
- [x] **Architecture OS** (Tasks, Mutex, Semaphores).
- [x] **Drivers Bas Niveau** (Tous capteurs + Moteurs).
- [x] **Asservissement PID** (Réglé et stable).
- [x] **Sécurité Anti-Chute** (Infaillible).
- [x] **Stratégie Complète** (Search, Attack, Flee).
- [x] **Logique de Jeu** (Changement de rôle par choc).
- [x] **Connectivité Bluetooth** (Télécommande & Logs).

### 📅 Perspectives
- Optimisation de la vitesse de rotation en combat pour éviter le dérapage.
- Ajout d'une stratégie d'évitement prédictif des murs (SLAM simplifié).

---

## 📄 **Licence**
Projet développé à l'ENSEA. Code source sous licence MIT.