#include "strategy.h"
#include "lidar.h"
#include <stdio.h>
#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Extern variables from app_freertos.c
extern volatile float target_speed_lin_x;
extern volatile float target_speed_ang_z;

static Strategy_State_t current_state = STATE_IDLE;
static Game_Role_t current_role = ROLE_CHAT;
static uint8_t robot_enabled = 0; // Robot arrêté par défaut

#define SEARCH_ROT_SPEED 0.0f     // rad/s
#define ATTACK_LIN_SPEED 150.0f   // mm/s
#define ATTACK_MAX_ANG_SPEED 2.0f // rad/s

#define FLEE_LIN_SPEED 150.0f     // Fuite rapide !
#define FLEE_MAX_ANG_SPEED 2.0f   // Virage serré

void Strategy_SetLEDS(void) {
    if (current_role == ROLE_CHAT) {
        HAL_GPIO_WritePin(STATUS_CHAT_LED_GPIO_Port, STATUS_CHAT_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(STATUS_SOURIS_LED_GPIO_Port, STATUS_SOURIS_LED_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(STATUS_CHAT_LED_GPIO_Port, STATUS_CHAT_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STATUS_SOURIS_LED_GPIO_Port, STATUS_SOURIS_LED_Pin, GPIO_PIN_SET);
    }
}

void Strategy_Init(void) {
    current_state = STATE_SEARCH;
    current_role = ROLE_CHAT;
    Strategy_SetLEDS();
}

void Strategy_ToggleRole(void) {
    if (current_role == ROLE_CHAT) {
        current_role = ROLE_SOURIS;
        printf("Strategy: Switch to MOUSE Mode!\r\n");
    } else {
        current_role = ROLE_CHAT;
        printf("Strategy: Switch to CAT Mode!\r\n");
    }
    Strategy_SetLEDS();
    
    // Reset state to Search to re-evaluate situation
    current_state = STATE_SEARCH;
}

void Strategy_Update(void) {
    LidarTarget_t target = ydlidar_get_target();

    // 1. Détermination de l'état
    switch (current_state) {
        case STATE_SEARCH:
            target_speed_lin_x = 0.0f;
            target_speed_ang_z = SEARCH_ROT_SPEED;

            if (target.is_valid) {
                if (current_role == ROLE_CHAT) {
                    current_state = STATE_ATTACK;
                    printf("CAT: Target Found -> ATTACK\r\n");
                } else {
                    current_state = STATE_FLEE;
                    printf("MOUSE: Threat Found -> FLEE\r\n");
                }
            }
            break;

        case STATE_ATTACK: // Uniquement pour le CHAT
            if (current_role == ROLE_SOURIS) { current_state = STATE_SEARCH; break; } // Sécurité
            
            if (!target.is_valid) {
                current_state = STATE_SEARCH;
                break;
            }

            // P-Control pour faire face à la cible (Angle = 0)
            float angle_error_deg = target.angle;
            if (angle_error_deg > 180.0f) angle_error_deg -= 360.0f;
            
            float angle_error_rad = angle_error_deg * (M_PI / 180.0f);
            
            target_speed_ang_z = angle_error_rad * 5.0f;
            
            // Saturation Angulaire
            if (target_speed_ang_z > ATTACK_MAX_ANG_SPEED) target_speed_ang_z = ATTACK_MAX_ANG_SPEED;
            if (target_speed_ang_z < -ATTACK_MAX_ANG_SPEED) target_speed_ang_z = -ATTACK_MAX_ANG_SPEED;

            // Gestion Distance
            if (target.distance > 100.0f) {
                target_speed_lin_x = ATTACK_LIN_SPEED;
            } else {
                target_speed_lin_x = 50.0f; 
            }
            break;

        case STATE_FLEE: // Uniquement pour la SOURIS
            if (current_role == ROLE_CHAT) { current_state = STATE_SEARCH; break; } // Sécurité

            if (!target.is_valid) {
                current_state = STATE_SEARCH;
                break;
            }

            // Logique de Fuite
            float flee_angle_deg = target.angle - 180.0f;

            if (flee_angle_deg < -180.0f) flee_angle_deg += 360.0f;
            if (flee_angle_deg > 180.0f)  flee_angle_deg -= 360.0f;

            float flee_error_rad = flee_angle_deg * (M_PI / 180.0f);

            target_speed_ang_z = flee_error_rad * 6.0f; 
            
            if (target_speed_ang_z > FLEE_MAX_ANG_SPEED) target_speed_ang_z = FLEE_MAX_ANG_SPEED;
            if (target_speed_ang_z < -FLEE_MAX_ANG_SPEED) target_speed_ang_z = -FLEE_MAX_ANG_SPEED;

            if (fabsf(flee_angle_deg) < 45.0f) {
                target_speed_lin_x = FLEE_LIN_SPEED;
            } else {
                target_speed_lin_x = 50.0f; 
            }
            break;

        case STATE_IDLE:
            target_speed_lin_x = 0.0f;
            target_speed_ang_z = 0.0f;
            break;
    }
}

Strategy_State_t Strategy_GetCurrentState(void) {
    return current_state;
}

Game_Role_t Strategy_GetCurrentRole(void) {
    return current_role;
}

void Strategy_SetRole(Game_Role_t role) {
    current_role = role;
    Strategy_SetLEDS();
    printf("Strategy: Role set to %s\r\n", (role == ROLE_CHAT) ? "CAT" : "MOUSE");
}

void Strategy_SetEnabled(uint8_t enabled) {
    robot_enabled = enabled;
    if (enabled) {
        printf("Robot: STARTED\r\n");
    } else {
        printf("Robot: STOPPED\r\n");
        target_speed_lin_x = 0;
        target_speed_ang_z = 0;
    }
}

uint8_t Strategy_IsEnabled(void) {
    return robot_enabled;
}
