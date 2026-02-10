│  1 #ifndef INC_MOTOR_H_ // Changed from INC_MOTOR_DRIVER_H_ to INC_MOTOR_H_ to match new file name                   │
│  2 #define INC_MOTOR_H_ // Changed from INC_MOTOR_DRIVER_H_ to INC_MOTOR_H_ to match new file name                   │
│  3                                                                                                                   │
│  4 #include "stm32g4xx_hal.h"                                                                                        │
│  5                                                                                                                   │
│  6 typedef struct {                                                                                                  │
│  7     // PWM                                                                                                        │
│  8     TIM_HandleTypeDef* pwm_timer;                                                                                 │
│  9     uint32_t           channel_fwd; // Canal PWM Avant (e.g., TIM_CHANNEL_1)                                      │
│ 10     uint32_t           channel_rev; // Canal PWM Arrière (e.g., TIM_CHANNEL_2)                                    │
│ 11                                                                                                                   │
│ 12     // Encodeur                                                                                                   │
│ 13     TIM_HandleTypeDef* enc_timer;                                                                                 │
│ 14     int32_t            enc_prev_counter; // Stockage précédent pour delta                                         │
│ 15     uint32_t           enc_resolution;   // PPR * 4 (Mode TI1+TI2 si utilisé) ou PPR (e.g., 2000 for 500 PPR      │
│    quadrature)                                                                                                       │
│ 16                                                                                                                   │
│ 17     // Données Physiques                                                                                          │
│ 18     float              speed_rpm;                                                                                 │
│ 19     float              speed_rad_s;                                                                               │
│ 20     int32_t            total_ticks;      // Odométrie absolue                                                     │
│ 21 } Motor_Handle_t;                                                                                                 │
│ 22                                                                                                                   │
│ 23 /**                                                                                                               │
│ 24  * @brief Initializes the motor driver.                                                                           │
│ 25  *        Starts the PWM and encoder timers.                                                                      │
│ 26  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│ 27  */                                                                                                               │
│ 28 void Motor_Init(Motor_Handle_t* hmotor);                                                                          │
│ 29                                                                                                                   │
│ 30 /**                                                                                                               │
│ 31  * @brief Sets the motor speed and direction.                                                                     │
│ 32  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│ 33  * @param pwm_percent PWM duty cycle from -100.0 (full reverse) to +100.0 (full forward).                         │
│ 34  */                                                                                                               │
│ 35 void Motor_SetSpeed(Motor_Handle_t* hmotor, float pwm_percent);                                                   │
│ 36                                                                                                                   │
│ 37 /**                                                                                                               │
│ 38  * @brief Updates the motor speed based on encoder readings.                                                      │
│ 39  *        Should be called periodically (e.g., in a control loop task).                                           │
│ 40  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│ 41  * @param delta_time_s Time elapsed since the last update in seconds.                                             │
│ 42  */                                                                                                               │
│ 43 void Motor_UpdateSpeed(Motor_Handle_t* hmotor, float delta_time_s);                                               │
│ 44                                                                                                                   │
│ 45 #endif /* INC_MOTOR_H_ */     

│  1 #include "motor.h"                                                                                                │
│  2 #include <stdlib.h> // For abs()                                                                                  │
│  3                                                                                                                   │
│  4 /**                                                                                                               │
│  5  * @brief Initializes the motor driver.                                                                           │
│  6  *        Starts the PWM and encoder timers.                                                                      │
│  7  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│  8  */                                                                                                               │
│  9 void Motor_Init(Motor_Handle_t* hmotor) {                                                                         │
│ 10     if (hmotor == NULL) return;                                                                                   │
│ 11                                                                                                                   │
│ 12     // Start PWM channels                                                                                         │
│ 13     HAL_TIM_PWM_Start(hmotor->pwm_timer, hmotor->channel_fwd);                                                    │
│ 14     HAL_TIM_PWM_Start(hmotor->pwm_timer, hmotor->channel_rev);                                                    │
│ 15                                                                                                                   │
│ 16     // Start encoder timer                                                                                        │
│ 17     HAL_TIM_Encoder_Start(hmotor->enc_timer, TIM_CHANNEL_ALL);                                                    │
│ 18                                                                                                                   │
│ 19     // Initialize previous counter value and total ticks                                                          │
│ 20     hmotor->enc_prev_counter = __HAL_TIM_GET_COUNTER(hmotor->enc_timer);                                          │
│ 21     hmotor->total_ticks = 0;                                                                                      │
│ 22     hmotor->speed_rpm = 0.0f;                                                                                     │
│ 23     hmotor->speed_rad_s = 0.0f;                                                                                   │
│ 24 }                                                                                                                 │
│ 25                                                                                                                   │
│ 26 /**                                                                                                               │
│ 27  * @brief Sets the motor speed and direction.                                                                     │
│ 28  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│ 29  * @param pwm_percent PWM duty cycle from -100.0 (full reverse) to +100.0 (full forward).                         │
│ 30  */                                                                                                               │
│ 31 void Motor_SetSpeed(Motor_Handle_t* hmotor, float pwm_percent) {                                                  │
│ 32     if (hmotor == NULL) return;                                                                                   │
│ 33                                                                                                                   │
│ 34     // Clamp PWM percentage to [-100, 100]                                                                        │
│ 35     if (pwm_percent > 100.0f) pwm_percent = 100.0f;                                                               │
│ 36     else if (pwm_percent < -100.0f) pwm_percent = -100.0f;                                                        │
│ 37                                                                                                                   │
│ 38     // Calculate absolute duty cycle                                                                              │
│ 39     uint32_t duty_cycle = (uint32_t)(abs((int)pwm_percent) * hmotor->pwm_timer->Init.Period / 100.0f);            │
│ 40                                                                                                                   │
│ 41     if (pwm_percent > 0) { // Forward                                                                             │
│ 42         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, duty_cycle);                                │
│ 43         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, 0);                                         │
│ 44     } else if (pwm_percent < 0) { // Reverse                                                                      │
│ 45         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, 0);                                         │
│ 46         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, duty_cycle);                                │
│ 47     } else { // Stop                                                                                              │
│ 48         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, 0);                                         │
│ 49         __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, 0);                                         │
│ 50     }                                                                                                             │
│ 51 }                                                                                                                 │
│ 52                                                                                                                   │
│ 53 /**                                                                                                               │
│ 54  * @brief Updates the motor speed based on encoder readings.                                                      │
│ 55  *        Should be called periodically (e.g., in a control loop task).                                           │
│ 56  * @param hmotor Pointer to the Motor_Handle_t structure.                                                         │
│ 57  * @param delta_time_s Time elapsed since the last update in seconds.                                             │
│ 58  */                                                                                                               │
│ 59 void Motor_UpdateSpeed(Motor_Handle_t* hmotor, float delta_time_s) {                                              │
│ 60     if (hmotor == NULL || delta_time_s == 0.0f) return;                                                           │
│ 61                                                                                                                   │
│ 62     int32_t current_counter = __HAL_TIM_GET_COUNTER(hmotor->enc_timer);                                           │
│ 63     int32_t delta_ticks;                                                                                          │
│ 64                                                                                                                   │
│ 65     // Handle 16-bit timer overflow/underflow for delta calculation if period is 65535                            │
│ 66     if (hmotor->enc_timer->Init.Period == 65535) { // Assuming 16-bit timer                                       │
│ 67         delta_ticks = (int16_t)(current_counter - hmotor->enc_prev_counter);                                      │
│ 68     } else { // Assuming 32-bit timer or larger period                                                            │
│ 69         delta_ticks = current_counter - hmotor->enc_prev_counter;                                                 │
│ 70     }                                                                                                             │
│ 71                                                                                                                   │
│ 72     hmotor->enc_prev_counter = current_counter;                                                                   │
│ 73     hmotor->total_ticks += delta_ticks;                                                                           │
│ 74                                                                                                                   │
│ 75     // Calculate speed in rad/s and RPM                                                                           │
│ 76     // (delta_ticks / enc_resolution) gives revolutions                                                           │
│ 77     // (revolutions / delta_time_s) gives revolutions per second                                                  │
│ 78     // (rev/s * 60) gives RPM                                                                                     │
│ 79     // (rev/s * 2*PI) gives rad/s                                                                                 │
│ 80                                                                                                                   │
│ 81     float revolutions_per_second = (float)delta_ticks / (float)hmotor->enc_resolution / delta_time_s;             │
│ 82     hmotor->speed_rpm = revolutions_per_second * 60.0f;                                                           │
│ 83     hmotor->speed_rad_s = revolutions_per_second * 2.0f * 3.1415926535f; // Using PI constant                     │
│ 84 }                                                                                                                 │