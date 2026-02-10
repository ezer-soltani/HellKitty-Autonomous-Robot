#include "pid.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral_max = out_max; 
    
    PID_Reset(pid);
}

void PID_Reset(PID_Controller_t *pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measured) {
    // 1. Calcul de l'erreur
    float error = setpoint - measured;

    // 2. Calcul du terme Proportionnel
    float P = pid->Kp * error;

    // 3. Calcul du terme Intégral avec Anti-Windup
    pid->integral += error * pid->dt;
    
    // Limitation de l'intégrale (Anti-windup)
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    float I = pid->Ki * pid->integral;

    // 4. Calcul du terme Dérivé
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;

    // 5. Calcul de la sortie totale
    float output = P + I + D;

    // 6. Saturation de la sortie
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    // 7. Sauvegarde de l'erreur pour le prochain tour
    pid->prev_error = error;

    return output;
}
