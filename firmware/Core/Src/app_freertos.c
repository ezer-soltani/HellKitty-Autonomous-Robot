/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "i2c.h"
#include "tim.h"
#include <stdio.h>
#include "lidar.h"
#include "imu.h"
#include "tof.h"
#include "motor.h"
#include "pid.h"
#include "odometry.h"
#include "strategy.h"
#include "hc-05_bluetooth.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Task Handles
TaskHandle_t xLidarTaskHandle = NULL;
TaskHandle_t xImuTaskHandle = NULL;
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;

// Mutex Handles
SemaphoreHandle_t xI2C1Mutex = NULL;
SemaphoreHandle_t xUARTMutex = NULL;

// Lidar Buffer
uint8_t lidar_dma_buffer[LIDAR_DMA_BUFFER_SIZE];

// Motor Handles
Motor_Handle_t hMotor1;
Motor_Handle_t hMotor2;

// Control & Navigation
PID_Controller_t pid_vel_left;  // PID Vitesse Moteur Gauche (Motor 2)
PID_Controller_t pid_vel_right; // PID Vitesse Moteur Droit (Motor 1)
Odometry_t robot_odom;          // Position du robot (X, Y, Theta)

volatile float target_speed_lin_x = 0.0f; // Vitesse linéaire cible
volatile float target_speed_ang_z = 0.0f; // Vitesse de rotation cible

volatile uint8_t g_safety_override = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vLidarTask(void *pvParameters);
void vImuTask(void *pvParameters);
void vSafetyTask(void *pvParameters);
void vControlTask(void *pvParameters);
void MX_Motor_Init(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* Create Mutexes */
  xI2C1Mutex = xSemaphoreCreateMutex();
  xUARTMutex = xSemaphoreCreateMutex();

  /* Initialize Drivers */
  MX_Motor_Init();
  Strategy_Init();
  HC05_Init();

  /* Create Tasks */
  if (xTaskCreate(vImuTask, "ImuTask", 256, NULL, 1, &xImuTaskHandle) != pdPASS) {
      printf("IMU Task Creation Failed\r\n");
      Error_Handler();
  }

  if (xTaskCreate(vLidarTask, "LidarTask", 512, NULL, 2, &xLidarTaskHandle) != pdPASS) {
      printf("Lidar Task Creation Failed\r\n");
      Error_Handler();
  }

  if (xTaskCreate(vSafetyTask, "SafetyTask", 256, NULL, 3, &xSafetyTaskHandle) != pdPASS) {
      printf("Safety Task Creation Failed\r\n");
      Error_Handler();
  }

  if (xTaskCreate(vControlTask, "ControlTask", 512, NULL, 4, &xControlTaskHandle) != pdPASS) {
      printf("Control Task Creation Failed\r\n");
      Error_Handler();
  }
  
  /* USER CODE END Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MX_Motor_Init(void)
{
  /* Initialize Motors */
  // Motor 1 (Right): TIM3 CH1/CH2, Encoder TIM2 (32-bit)
  hMotor1.pwm_timer = &htim3;
  hMotor1.channel_fwd = TIM_CHANNEL_1; // PA6
  hMotor1.channel_rev = TIM_CHANNEL_2; // PA7
  hMotor1.enc_timer = &htim2;
  hMotor1.enc_resolution = 2048; //(Mode TI1/TI2)
  Motor_Init(&hMotor1);
  hMotor1.pwm_ramp_step = 100.0f;

  // Motor 2 (Left): TIM3 CH3/CH4, Encoder TIM4 (16-bit)
  hMotor2.pwm_timer = &htim3;
  hMotor2.channel_fwd = TIM_CHANNEL_3; // PB0
  hMotor2.channel_rev = TIM_CHANNEL_4; // PB1
  hMotor2.enc_timer = &htim4;
  hMotor2.enc_resolution = 2048;
  Motor_Init(&hMotor2);
  hMotor2.pwm_ramp_step = 100.0f;

  hMotor1.enc_timer->Instance->SMCR &= ~TIM_SMCR_ETF;
  hMotor1.enc_timer->Instance->SMCR |= (0x08 << TIM_SMCR_ETF_Pos);
  hMotor2.enc_timer->Instance->SMCR &= ~TIM_SMCR_ETF;
  hMotor2.enc_timer->Instance->SMCR |= (0x08 << TIM_SMCR_ETF_Pos);
}

void vControlTask(void *pvParameters)
{
    const float dt = 0.01f;

    PID_Init(&pid_vel_left,  5.0f, 20.0f, 0.0f, dt, -100.0f, 100.0f);
    PID_Init(&pid_vel_right, 5.0f, 20.0f, 0.0f, dt, -100.0f, 100.0f);

    Odom_Init(&robot_odom);
    Odom_SetPosition(&robot_odom, 0.0f, 0.0f, 0.0f);

    vTaskDelay(pdMS_TO_TICKS(500));
    
    Motor_ResetEncoder(&hMotor1);
    Motor_ResetEncoder(&hMotor2);
    
    PID_Reset(&pid_vel_left);
    PID_Reset(&pid_vel_right);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for(;;) 
    {
        Motor_UpdateSpeed(&hMotor1, dt); // Droit
        Motor_UpdateSpeed(&hMotor2, dt); // Gauche

        if (g_safety_override == 0) {
            // Force Forward Motion (Low Speed Test)
            /*target_speed_lin_x = 150.0f; // 150 mm/s
            target_speed_ang_z = 0.0f;   // Straight
            */
            if (Strategy_IsEnabled()) {
                Strategy_Update();
            } else {
                target_speed_lin_x = 0;
                target_speed_ang_z = 0;
            }

        }

        float speed_L = -hMotor2.speed_rad_s; 
        float speed_R = hMotor1.speed_rad_s;
        Odom_Update(&robot_odom, speed_L, speed_R, dt);

        if (g_safety_override == 0)
        {
            float v_lin = target_speed_lin_x;
            float v_ang = target_speed_ang_z;
            float half_track = WHEEL_TRACK / 2.0f;
            float radius = WHEEL_DIAMETER / 2.0f;

            float target_rad_L = (v_lin - (v_ang * half_track)) / radius;
            float target_rad_R = (v_lin + (v_ang * half_track)) / radius;

            float pwm_L = PID_Compute(&pid_vel_left,  target_rad_L, speed_L);
            float pwm_R = PID_Compute(&pid_vel_right, target_rad_R, speed_R);

            Motor_SetSpeed(&hMotor2, -pwm_L); 
            Motor_SetSpeed(&hMotor1, pwm_R);
            
            Motor_UpdatePWM(&hMotor2);
            Motor_UpdatePWM(&hMotor1);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void vLidarTask(void *pvParameters)
{
  ydlidar_init(lidar_dma_buffer, LIDAR_DMA_BUFFER_SIZE);

  uint16_t old_pos = 0;
  static uint32_t last_check = 0;

  for(;;)
  {
    uint16_t pos = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if (pos != old_pos) {
      if (pos > old_pos) {
        ydlidar_process_data(&lidar_dma_buffer[old_pos], pos - old_pos);
      } else {
        ydlidar_process_data(&lidar_dma_buffer[old_pos], LIDAR_DMA_BUFFER_SIZE - old_pos);
        if (pos > 0) {
            ydlidar_process_data(&lidar_dma_buffer[0], pos);
        }
      }
      old_pos = pos;
    }

    if ((HAL_GetTick() - last_check) > 100) {
        LidarObject_t objects[MAX_LIDAR_OBJECTS];
        uint8_t count = 0;

        if (xSemaphoreTake(xUARTMutex, portMAX_DELAY) == pdTRUE) {
            ydlidar_detect_objects(objects, &count);
            ydlidar_update_tracking(objects, count);

            xSemaphoreGive(xUARTMutex);

            // Get the closest tracked object and print its details
            LidarTarget_t closest_object = ydlidar_get_target();
            if (closest_object.is_valid) {
                printf("Closest Object -> Angle: %.2f, Distance: %.2f\r\n", closest_object.angle, closest_object.distance);
            }
        }

        last_check = HAL_GetTick();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vImuTask(void *pvParameters)
{
  if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
      if (ADXL343_Init(&hi2c1) != HAL_OK) {
          printf("IMU Init Failed\r\n");
      } else {
          ADXL343_ConfigShock(&hi2c1, 3.5f, 10.0f);
          printf("IMU Init OK\r\n");
      }
      xSemaphoreGive(xI2C1Mutex);
  }

  adxl343_axes_t accel_data;
  uint8_t error_count = 0;

  for(;;)
  {
    if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {

        uint8_t shock_detected = 0;
        static uint32_t last_shock_time = 0;
        HAL_StatusTypeDef status_read;

        // Read Axes
        status_read = ADXL343_ReadAxes(&hi2c1, &accel_data);

        if (status_read != HAL_OK) {
            error_count++;
        } else {
            error_count = 0;
            if (ADXL343_CheckShock(&hi2c1)) {
                // Ignorer le choc si la manoeuvre de sécurité est en cours
                if (g_safety_override == 0) {
                    shock_detected = 1;
                }
            }
        }

        if (error_count > 5) {
            printf("IMU I2C Error. Resetting...\r\n");
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
            HAL_Delay(10);
            ADXL343_Init(&hi2c1);
            ADXL343_ConfigShock(&hi2c1, 3.5f, 10.0f);
            error_count = 0;
        }

        xSemaphoreGive(xI2C1Mutex);

        if (shock_detected && (HAL_GetTick() - last_shock_time > 2000)) {
            printf("IMU: SHOCK DETECTED! Toggling Role.\r\n");
            Strategy_ToggleRole();
            last_shock_time = HAL_GetTick();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vSafetyTask(void *pvParameters)
{
  uint16_t dist[4] = {0};

  // Delay start to allow sensors to boot
  vTaskDelay(pdMS_TO_TICKS(500));

  for(;;) {
      // Poll sensors (Blocking call - up to 20ms per sensor if not ready)
      if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
          TOF_Read_All(dist);
          xSemaphoreGive(xI2C1Mutex);
      }

      // Check for Void (> 500mm)
      // TOF1: Fwd Right, TOF2: Fwd Left, TOF3: Rear Left, TOF4: Rear Right
      int void_fwd_right  = (dist[0] > 200);
      int void_fwd_left   = (dist[1] > 200);
      int void_rear_left  = (dist[2] > 200);
      int void_rear_right = (dist[3] > 200);

      if (void_fwd_left || void_fwd_right) {
          
          printf("Safety: VOID DETECTED! (D1:%d D2:%d D3:%d D4:%d)\r\n", dist[0], dist[1], dist[2], dist[3]);
          g_safety_override = 1;
          HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_SET);

          Motor_SetSpeed(&hMotor1, 0.0f);
          Motor_SetSpeed(&hMotor2, 0.0f);
          Motor_UpdatePWM(&hMotor1);
          Motor_UpdatePWM(&hMotor2);
          vTaskDelay(pdMS_TO_TICKS(500));

          Motor_SetSpeed(&hMotor1, -50.0f);
          Motor_SetSpeed(&hMotor2, 50.0f);
          Motor_UpdatePWM(&hMotor1);
          Motor_UpdatePWM(&hMotor2);
          vTaskDelay(pdMS_TO_TICKS(600));

          Motor_SetSpeed(&hMotor1, 50.0f);
          Motor_SetSpeed(&hMotor2, 50.0f);
          Motor_UpdatePWM(&hMotor1);
          Motor_UpdatePWM(&hMotor2);
          vTaskDelay(pdMS_TO_TICKS(800));
          
          PID_Reset(&pid_vel_left);
          PID_Reset(&pid_vel_right);
          HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_RESET);
          g_safety_override = 0;
      }
      if (void_rear_left || void_rear_right){
    	  printf("Safety: VOID DETECTED! (D1:%d D2:%d D3:%d D4:%d)\r\n", dist[0], dist[1], dist[2], dist[3]);
      g_safety_override = 1;
      HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_SET);

      Motor_SetSpeed(&hMotor1, 0.0f);
      Motor_SetSpeed(&hMotor2, 0.0f);
      Motor_UpdatePWM(&hMotor1);
      Motor_UpdatePWM(&hMotor2);
      vTaskDelay(pdMS_TO_TICKS(500));

      Motor_SetSpeed(&hMotor1, 50.0f);
      Motor_SetSpeed(&hMotor2, -50.0f);
      Motor_UpdatePWM(&hMotor1);
      Motor_UpdatePWM(&hMotor2);
      vTaskDelay(pdMS_TO_TICKS(600));

      Motor_SetSpeed(&hMotor1, 50.0f);
      Motor_SetSpeed(&hMotor2, 50.0f);
      Motor_UpdatePWM(&hMotor1);
      Motor_UpdatePWM(&hMotor2);
      vTaskDelay(pdMS_TO_TICKS(800));

      PID_Reset(&pid_vel_left);
      PID_Reset(&pid_vel_right);
      HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_RESET);
      g_safety_override = 0;
  }
      
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Callback pour la réception Bluetooth
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        HC05_RxCallback();
    }
}
/* USER CODE END Application */

