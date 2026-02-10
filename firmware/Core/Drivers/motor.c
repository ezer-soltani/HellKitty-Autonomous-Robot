#include "motor.h"
#include <stdlib.h>
#include <math.h>

void Motor_Init(Motor_Handle_t* hmotor) {
    if (hmotor == NULL) return;

    // Start PWM channels
    HAL_TIM_PWM_Start(hmotor->pwm_timer, hmotor->channel_fwd);
    HAL_TIM_PWM_Start(hmotor->pwm_timer, hmotor->channel_rev);

    // Start encoder timer
    HAL_TIM_Encoder_Start(hmotor->enc_timer, TIM_CHANNEL_ALL);

    hmotor->enc_prev_counter = __HAL_TIM_GET_COUNTER(hmotor->enc_timer);
    hmotor->total_ticks = 0;
    hmotor->speed_rpm = 0.0f;
    hmotor->speed_rad_s = 0.0f;

    // Ramp Init
    hmotor->current_pwm = 0.0f;
    hmotor->target_pwm = 0.0f;
    hmotor->pwm_ramp_step = 5.0f;
}

void Motor_SetSpeed(Motor_Handle_t* hmotor, float pwm_percent) {
    if (hmotor == NULL) return;

    // Clamp target
    if (pwm_percent > 100.0f) pwm_percent = 100.0f;
    else if (pwm_percent < -100.0f) pwm_percent = -100.0f;

    hmotor->target_pwm = pwm_percent;
}

void Motor_UpdatePWM(Motor_Handle_t* hmotor) {
    if (hmotor == NULL) return;

    // Ramping Logic
    float diff = hmotor->target_pwm - hmotor->current_pwm;
    
    if (diff > hmotor->pwm_ramp_step) {
        hmotor->current_pwm += hmotor->pwm_ramp_step;
    } else if (diff < -hmotor->pwm_ramp_step) {
        hmotor->current_pwm -= hmotor->pwm_ramp_step;
    } else {
        hmotor->current_pwm = hmotor->target_pwm;
    }

    // Apply to Hardware
    float applied_pwm = hmotor->current_pwm;
    uint32_t duty_cycle = (uint32_t)(fabs(applied_pwm) * hmotor->pwm_timer->Init.Period / 100.0f);

    if (applied_pwm > 0) { // Forward
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, duty_cycle);
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, 0);
    } else if (applied_pwm < 0) { // Reverse
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, 0);
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, duty_cycle);
    } else { // Stop
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_fwd, 0);
        __HAL_TIM_SET_COMPARE(hmotor->pwm_timer, hmotor->channel_rev, 0);
    }
}

void Motor_UpdateSpeed(Motor_Handle_t* hmotor, float delta_time_s) {
    if (hmotor == NULL || delta_time_s == 0.0f) return;

    int32_t current_counter = __HAL_TIM_GET_COUNTER(hmotor->enc_timer);
    int32_t delta_ticks;

    if (hmotor->enc_timer->Init.Period <= 65535) { // 16-bit timer
        delta_ticks = (int16_t)(current_counter - hmotor->enc_prev_counter);
    } else { // 32-bit timer
        delta_ticks = current_counter - hmotor->enc_prev_counter;
    }

    hmotor->enc_prev_counter = current_counter;
    hmotor->total_ticks += delta_ticks;

    float revolutions_per_second = (float)delta_ticks / (float)hmotor->enc_resolution / delta_time_s;
    hmotor->speed_rpm = revolutions_per_second * 60.0f;
    hmotor->speed_rad_s = revolutions_per_second * 2.0f * M_PI;
}

void Motor_ResetEncoder(Motor_Handle_t* hmotor) {
    if (hmotor == NULL) return;

    // Reset Hardware Counter
    __HAL_TIM_SET_COUNTER(hmotor->enc_timer, 0);

    // Reset Internal Variables
    hmotor->enc_prev_counter = 0;
    hmotor->total_ticks = 0;
    hmotor->speed_rpm = 0.0f;
    hmotor->speed_rad_s = 0.0f;
}
