#ifndef __PID_H
#define __PID_H

#include "stm32g4xx_hal.h"

typedef struct {
    // Gains
    float Kp;
    float Ki;
    float Kd;

    // État interne
    float prev_error;
    float integral;

    // Limites (Anti-windup et Saturation)
    float out_min;
    float out_max;
    float integral_max;

    // Période d'échantillonnage (en secondes)
    float dt;
} PID_Controller_t;

/**
 * @brief Initialise le contrôleur PID
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max);

float PID_Compute(PID_Controller_t *pid, float setpoint, float measured);

void PID_Reset(PID_Controller_t *pid);

#endif /* __PID_H */
