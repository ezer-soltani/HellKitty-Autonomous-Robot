# Architecture FreeRTOS pour l'intégration LIDAR + IMU

Ce document décrit la structure à implémenter dans `app_freertos.c` (et les fichiers associés) pour faire tourner les drivers LIDAR et IMU en parallèle.

## 1. Vue d'ensemble des Tâches

| Tâche | Priorité | Stack Size | Périodicité | Rôle |
| :--- | :--- | :--- | :--- | :--- |
| **Task_Lidar** | `osPriorityNormal` | 512 Words | Événementiel (UART RX) | Récupère les données brutes du DMA, parse le protocole X2, détecte les obstacles. |
| **Task_Imu** | `osPriorityBelowNormal` | 256 Words | Périodique (100ms) | Lit l'accéléromètre via I2C, détecte les chocs/tap, calcule l'inclinaison. |
| **defaultTask** | `osPriorityLow` | 128 Words | - | Tâche par défaut (LED Heartbeat ou Idle). |

## 2. Ressources Partagées (Mutex/Sémaphores)

*   **`myI2C1Mutex`** : Essentiel pour protéger l'accès au bus I2C1 (utilisé par l'IMU maintenant, et les capteurs TOF plus tard).
*   **`myUARTMutex`** : Recommandé pour éviter que les `printf` de différentes tâches ne se mélangent sur la console série.

---

## 3. Implémentation dans `app_freertos.c`

### A. Includes et Déclarations

Ajoutez les headers de vos drivers et déclarez les handles.

```c
/* Private includes ----------------------------------------------------------*/
#include "lidar.h"
#include "imu.h"
#include "usart.h" // Pour huart2 (Lidar) et printf
#include "i2c.h"   // Pour hi2c1

/* Private variables ---------------------------------------------------------*/
// Tâches
osThreadId lidiTaskHandle;
osThreadId imuTaskHandle;

// Mutex
osMutexId i2c1MutexHandle;
osMutexId uartMutexHandle;

// Buffer DMA pour le Lidar (Circular Buffer)
#define LIDAR_DMA_BUFFER_SIZE 1024
uint8_t lidar_dma_buffer[LIDAR_DMA_BUFFER_SIZE];
```

### B. Initialisation (`MX_FREERTOS_Init`)

Créez les mutex et les tâches.

```c
void MX_FREERTOS_Init(void) {
  /* Create Mutexes */
  osMutexDef(i2c1Mutex);
  i2c1MutexHandle = osMutexCreate(osMutex(i2c1Mutex));

  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* Create Tasks */
  // Task Lidar
  osThreadDef(lidarTask, StartLidarTask, osPriorityNormal, 0, 512);
  lidiTaskHandle = osThreadCreate(osThread(lidarTask), NULL);

  // Task IMU
  osThreadDef(imuTask, StartImuTask, osPriorityBelowNormal, 0, 256);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
  
  // defaultTask (déjà existante) ...
}
```

### C. Code de la `Task_Lidar`

Cette tâche doit initialiser le Lidar, lancer la réception DMA, et traiter les données reçues.

**Note sur le DMA :** Pour éviter de bloquer, on lance le DMA en mode circulaire. La tâche surveille la position du pointeur DMA pour traiter les nouveaux octets.

```c
void StartLidarTask(void const * argument)
{
  /* Init Driver Lidar */
  ydlidar_init();
  
  /* Démarrage réception DMA Circulaire sur UART2 */
  HAL_UART_Receive_DMA(&huart2, lidar_dma_buffer, LIDAR_DMA_BUFFER_SIZE);
  
  static uint16_t old_pos = 0;
  
  for(;;)
  {
    // Calcul de la position courante du DMA (NDTR compte à rebours)
    uint16_t pos = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    
    if (pos != old_pos) {
      if (pos > old_pos) {
        // Pas de bouclage du buffer : on traite de old_pos à pos
        ydlidar_process_data(&lidar_dma_buffer[old_pos], pos - old_pos);
      } else {
        // Bouclage (wrap around) : on traite de old_pos à la fin, puis du début à pos
        ydlidar_process_data(&lidar_dma_buffer[old_pos], LIDAR_DMA_BUFFER_SIZE - old_pos);
        if (pos > 0) {
            ydlidar_process_data(&lidar_dma_buffer[0], pos);
        }
      }
      old_pos = pos;
    }

    // Détection d'objets périodique ou basée sur le scan complet
    // Ici on peut le faire à chaque tour ou utiliser un flag dans le driver
    static uint32_t last_check = 0;
    if (HAL_GetTick() - last_check > 50) { // 20Hz
        LidarObject_t objects[MAX_LIDAR_OBJECTS];
        uint8_t count = 0;
        
        // Protection printf si nécessaire
        // osMutexWait(uartMutexHandle, osWaitForever);
        ydlidar_detect_objects(objects, &count);
        // osMutexRelease(uartMutexHandle);
        
        last_check = HAL_GetTick();
    }

    osDelay(10); // Laisser la main aux autres tâches (10ms polling du buffer)
  }
}
```

### D. Code de la `Task_Imu`

Cette tâche est plus simple : elle lit les capteurs périodiquement. **Il faut absolument protéger l'accès I2C.**

```c
void StartImuTask(void const * argument)
{
  /* Init Driver IMU */
  // On prend le mutex même pour l'init pour être propre (si d'autres tâches I2C existaient)
  osMutexWait(i2c1MutexHandle, osWaitForever);
  if (ADXL343_Init(&hi2c1) != HAL_OK) {
      printf("IMU Init Error\r\n");
  }
  ADXL343_ConfigShock(&hi2c1, 2.5f, 10.0f); // Exemple config choc
  osMutexRelease(i2c1MutexHandle);

  adxl343_axes_t accel_data;

  for(;;)
  {
    osMutexWait(i2c1MutexHandle, osWaitForever);
    
    // 1. Lecture Accélération
    if (ADXL343_ReadAxes(&hi2c1, &accel_data) == HAL_OK) {
        // Conversion en float si besoin pour logique métier
        // float x_g = ADXL343_RawTo_g(accel_data.x);
    }

    // 2. Vérification Choc (Tap)
    if (ADXL343_CheckShock(&hi2c1)) {
        printf("!!! CHOC DETECTE !!!\r\n");
        // Action : Stop moteurs, LED rouge, etc.
    }
    
    osMutexRelease(i2c1MutexHandle);

    osDelay(100); // 10Hz suffisant pour l'inclinomètre. Pour les chocs, réduire le délai ou utiliser les interruptions GPIO.
  }
}
```

---

## 4. Modifications nécessaires dans `main.c`

Assurez-vous que le **Timer de base de temps** (Timebase Source) dans le fichier `.ioc` (CubeMX) est bien réglé sur un autre Timer que Systick (ex: `TIM1` ou `TIM6`), car FreeRTOS utilise Systick.
*   Si vous avez généré le code avec CubeMX en activant FreeRTOS, cela devrait être déjà fait (`HAL_InitTick` utilise TIM1 dans votre code actuel `stm32g4xx_hal_timebase_tim.c`).

## 5. Prochaines étapes

1.  Copier les prototypes et déclarations dans `app_freertos.c`.
2.  Implémenter `StartLidarTask` et `StartImuTask`.
3.  Compiler et flasher.
4.  Vérifier via le port série (Virtual Com Port) que :
    *   Le Lidar affiche les objets détectés.
    *   L'IMU réagit aux mouvements/chocs.
    *   Les deux messages s'affichent sans bloquer le système.

```