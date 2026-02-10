#include "odometry.h"
#include <math.h>

void Odom_Init(Odometry_t *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
}

void Odom_SetPosition(Odometry_t *odom, float x, float y, float theta) {
    odom->x = x;
    odom->y = y;
    odom->theta = theta;
}

void Odom_Update(Odometry_t *odom, float v_left, float v_right, float dt) {
    // 1. Conversion rad/s -> mm/s
    float radius = WHEEL_DIAMETER / 2.0f;
    float linear_v_left = v_left * radius;
    float linear_v_right = v_right * radius;

    // 2. Calcul des vitesses du robot
    float v_linear = (linear_v_right + linear_v_left) / 2.0f;
    float v_angular = (linear_v_right - linear_v_left) / WHEEL_TRACK;

    // 3. Intégration de la position
    // On utilise l'angle moyen pendant le déplacement pour plus de précision
    float delta_theta = v_angular * dt;
    float avg_theta = odom->theta + (delta_theta / 2.0f);

    odom->x += v_linear * cosf(avg_theta) * dt;
    odom->y += v_linear * sinf(avg_theta) * dt;
    odom->theta += delta_theta;

    // 4. Normalisation de l'angle entre -PI et PI
    while (odom->theta > M_PI)  odom->theta -= 2.0f * M_PI;
    while (odom->theta < -M_PI) odom->theta += 2.0f * M_PI;
}
