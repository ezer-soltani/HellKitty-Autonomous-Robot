#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32g4xx_hal.h"

#define WHEEL_DIAMETER    65.0f
#define WHEEL_TRACK       161.0f
#define WHEEL_CIRC        (WHEEL_DIAMETER * 3.14159f)

typedef struct {
    float x;      // Position X en mm
    float y;      // Position Y en mm
    float theta;  // Orientation en radians
} Odometry_t;

void Odom_Init(Odometry_t *odom);

void Odom_Update(Odometry_t *odom, float v_left, float v_right, float dt);

void Odom_SetPosition(Odometry_t *odom, float x, float y, float theta);

#endif /* __ODOMETRY_H */
