#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"

typedef struct {
    // PWM
    TIM_HandleTypeDef* pwm_timer;
    uint32_t           channel_fwd; // Canal PWM Avant
    uint32_t           channel_rev; // Canal PWM Arrière
    // Encodeur
    TIM_HandleTypeDef* enc_timer;
    int32_t            enc_prev_counter;
    uint32_t           enc_resolution;
    // Données
    float              speed_rpm;
    float              speed_rad_s;
    int32_t            total_ticks;
    // Ramp Control
    float              current_pwm;
    float              target_pwm;
    float              pwm_ramp_step;
} Motor_Handle_t;

void Motor_Init(Motor_Handle_t* hmotor);

void Motor_SetSpeed(Motor_Handle_t* hmotor, float pwm_percent);

void Motor_UpdatePWM(Motor_Handle_t* hmotor);

void Motor_UpdateSpeed(Motor_Handle_t* hmotor, float delta_time_s);

void Motor_ResetEncoder(Motor_Handle_t* hmotor);

#endif /* INC_MOTOR_H_ */
