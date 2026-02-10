# Architecture Firmware Robot (STM32G4 + FreeRTOS)

Ce document décrit l'architecture logicielle proposée pour la migration du firmware vers un système temps réel (FreeRTOS). L'architecture est organisée en couches (Layers) pour découpler le matériel de la logique décisionnelle.

## 1. Diagramme Architecturel Global

Le diagramme ci-dessous illustre les interactions entre les périphériques matériels, les drivers HAL, les drivers de composants, les algorithmes de contrôle et les tâches FreeRTOS.

```mermaid
graph BT
    %% On utilise BT (Bottom-Top) pour empiler les couches du bas vers le haut

    %% --- 1. COUCHE HARDWARE (Tout en bas) ---
    subgraph Hardware [Hardware Layer]
        direction BT
        HW_LIDAR[YDLIDAR X2]
        HW_TOF[VL53L0X x4]
        HW_IMU[ADXL343]
        HW_MOT[Moteurs DC]
        HW_ENC[Encodeurs]
        HW_BT[Bluetooth HC-05]
        HW_PC[PC Debug]
    end

    %% --- 2. COUCHE HAL ---
    subgraph HAL [HAL Layer - Low Level]
        direction BT
        UART2[usart.c - UART2]
        I2C1[i2c.c - I2C1]
        TIM3[tim.c - TIM3 PWM]
        TIM2_4[tim.c - TIM2/TIM4 ENC]
        UART3[usart.c - UART3]
        UART1[usart.c - UART1]
        GPIO[gpio.c]
        
        %% Interrupts
        IT_UART2(USART2_IRQHandler)
        IT_TIM(TIM_Update_IT)
    end

    %% LIENS: HARDWARE <--> HAL
    HW_LIDAR <--> UART2
    HW_TOF <--> I2C1
    HW_IMU <--> I2C1
    HW_MOT <--> TIM3
    HW_ENC <--> TIM2_4
    HW_BT <--> UART3
    HW_PC <--> UART1

    %% --- 3. COUCHE DRIVERS ---
    subgraph Drivers [Component Drivers]
        direction BT
        DRV_LIDAR[lidar.c]
        DRV_TOF[tof.c]
        DRV_IMU[imu.c]
        DRV_MOT[motor.c]
        DRV_ENC[encoder.c]
        DRV_COM[bluetooth.c]
    end

    %% LIENS: HAL --> DRIVERS (Flux de données remonte vers le haut)
    UART2 --> DRV_LIDAR
    IT_UART2 -.->|Notify| DRV_LIDAR
    I2C1 <--> DRV_TOF
    I2C1 <--> DRV_IMU
    TIM2_4 -->|Count| DRV_ENC
    UART3 <--> DRV_COM
    UART1 <--> DRV_COM
    GPIO --> DRV_TOF
    
    %% Pour les commandes (vers le bas), on inverse la syntaxe (<--) 
    %% pour garder le Driver au-dessus du HAL visuellement
    TIM3 <-- DRV_MOT
    GPIO <-- DRV_MOT

    %% --- 4. COUCHE HAUTE: ALGOS & RTOS ---
    subgraph HighLevel [Application Layer]
        direction BT
        
        subgraph Algos [Control & Strategy Libs]
            LIB_ODO[odometrie.c]
            LIB_ASSERV[asserv.c]
            LIB_TRAJ[trajectoire.c]
        end

        subgraph RTOS [FreeRTOS Tasks]
            TASK_SAFETY([Task_Safety_Edge\nCRITICAL Prio])
            TASK_IMU([Task_IMU_Acq\nMedium Prio])
            TASK_CONTROL([Task_ControlLoop\nHigh Prio])
            TASK_LIDAR([Task_Lidar\nMedium Prio])
            TASK_STRAT([Task_Strategy\nLow Prio])
            
            %% Objets RTOS
            MUTEX_I2C{Mutex_I2C}
            QUEUE_LIDAR[[Queue_LidarPts]]
            QUEUE_CMDS[[Queue_Commands]]
        end
    end

    %% LIENS: DRIVERS <--> ALGOS (Flux montant et descendant)
    DRV_ENC --> LIB_ODO
    DRV_IMU --> LIB_ODO
    LIB_ODO -->|Pos X,Y| LIB_ASSERV
    LIB_TRAJ -->|Consigne| LIB_ASSERV
    
    %% Commande moteur (Descente): Syntaxe <-- force LIB au dessus de DRV
    DRV_MOT <-- LIB_ASSERV

    %% LIENS: RTOS LOGIC
    %% Interrupt (Bas) réveille Queue (Haut)
    IT_UART2 -.->|Give ISR| QUEUE_LIDAR
    DRV_COM -->|Cmd ISR| QUEUE_CMDS

    %% Taches lisent les Queues/Mutex
    QUEUE_LIDAR -.-> TASK_LIDAR
    QUEUE_CMDS -.-> TASK_STRAT
    
    %% SÉPARATION TOF / IMU sur le Mutex I2C
    TASK_SAFETY -->|Take| MUTEX_I2C
    TASK_IMU -->|Take| MUTEX_I2C
    
    %% Appels de fonctions (Control Flow: Haut vers Bas)
    MUTEX_I2C -.-> DRV_TOF
    MUTEX_I2C -.-> DRV_IMU
    
    %% Safety Action Directe
    TASK_SAFETY ==>|EMERGENCY STOP| DRV_MOT
    
    DRV_LIDAR <-- TASK_LIDAR
    
    %% Data Flow Logic (Flux montant)
    DRV_LIDAR -->|Obstacles| TASK_STRAT

    %% Control Loop Layout
    LIB_ODO <-- TASK_CONTROL
    LIB_ASSERV <-- TASK_CONTROL
    LIB_TRAJ <-- TASK_STRAT
    TASK_CONTROL <.-|Update Setpoint| TASK_STRAT
```

## 2. Description des Modules

### A. Couche Bas Niveau (HAL)
Ce sont les fichiers générés par STM32CubeMX (`Core/Src/`).
*   **`usart.c`** : Configure les UARTs.
    *   *UART2* : Réception DMA circulaire ou IT (Interruption) pour le LIDAR.
    *   *UART3* : Communication Bluetooth.
*   **`i2c.c`** : Gestion du bus I2C1 partagé. **Critique :** Doit être protégé par un Mutex FreeRTOS car accédé par plusieurs drivers (TOF et IMU).
*   **`tim.c`** :
    *   *TIM2 & TIM4* : Mode Encodeur (Lecture matérielle des pas).
    *   *TIM3* : Génération PWM pour les moteurs.

### B. Couche Drivers
Abstraction du matériel. Ces fichiers ne contiennent pas de logique "métier" (pas d'asservissement ici).
*   **`lidar.c`** : Décode les paquets de données (Header `0xAA 55`, Checksum). Stocke les points (Angle, Distance) dans une structure accessible.
*   **`tof.c`** : Initialise les 4 capteurs VL53L0X (gestion des broches XSHUT) et lance les mesures.
*   **`motor.c`** : Conversion pourcentage vitesse (-100 à 100) vers PWM + Direction GPIO.
*   **`encoder.c`** : Lit les registres CNT des timers et gère le dépassement (overflow) pour avoir une valeur en ticks continue.

### C. Couche Algorithmique
Logique pure, indépendante du hardware (pourrait être testée sur PC).
*   **`odometrie.c`** :
    *   Entrée : Différence de ticks encodeurs (gauche/droite).
    *   Sortie : Mise à jour de la position globale robot (x, y, theta).
*   **`asserv.c`** :
    *   Contient les PID (Vitesse et Position/Angle).
    *   Calcule la commande moteur pour atteindre la consigne donnée par la stratégie.
*   **`trajectoire.c`** :
    *   Génère des profils de vitesse (trapèze) ou des points de passage pour aller d'un point A à B.

### D. Tâches FreeRTOS (Ordonnancement)

| Tâche | Priorité | Fréquence | Rôle |
| :--- | :--- | :--- | :--- |
| **Task_TOF** | **Très Haute / Critique** | **30-50 Hz** | **Sécurité.** Lit les 4 TOF (via I2C Mutex). Si distance > seuil (vide détecté), coupe immédiatement les moteurs (`Emergency Stop`) en contournant l'asservissement. |
| **Task_ControlLoop** | **Haute** | **100 Hz (10ms)** | **Temps Réel.** Lit les encodeurs -> Calcule l'odométrie -> Exécute les PIDs -> Applique le PWM moteurs. Doit être régulière. |
| **Task_IMU_Acq** | Moyenne | 20 Hz | Lit l'IMU (via I2C Mutex) pour détecter les chocs ou l'inclinaison. Met à jour l'état global. |
| **Task_Lidar** | Moyenne | Événementiel | Attend des données dans la `Queue_LidarPts` (remplie par l'IRQ UART), décode les trames et met à jour la carte des obstacles. |
| **Task_Strategy** | Basse | 10 Hz | Machine à états (FSM). Analyse les obstacles (LIDAR/TOF), décide de la prochaine action et envoie la consigne (X,Y) à l'asservissement. |

## 3. Gestion des Ressources Partagées

1.  **Bus I2C1 (Mutex_I2C)** :
    *   L'IMU et les 4 TOF sont sur le même bus.
    *   Si `Task_Safety_Edge` est préemptée, elle doit pouvoir prendre le Mutex dès qu'il est libre (mécanisme d'héritage de priorité FreeRTOS).
    *   L'IMU doit être lue dans une tâche moins prioritaire (`Task_IMU_Acq`) pour ne pas retarder la détection de bord.

2.  **Données UART Lidar (Queue + StreamBuffer)** :
    *   L'UART reçoit les octets très vite.
    *   L'ISR (Interrupt Service Routine) doit être très courte : elle pousse les octets dans un `StreamBuffer` ou une `Queue`.
    *   La `Task_Lidar` se réveille quand des données sont disponibles pour les traiter.

```