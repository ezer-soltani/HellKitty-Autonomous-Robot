/*
 * ADXL343_Driver.h
 *
 *
 */

#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdint.h>

// Adresse I2C de l'ADXL343
#define ADXL343_ADDR            (0x53 << 1)

// Registres ADXL343
#define ADXL_DEVID              0x00
#define ADXL_THRESH_TAP         0x1D
#define ADXL_DUR                0x21
#define ADXL_LATENT             0x22
#define ADXL_WINDOW             0x23
#define ADXL_TAP_AXES           0x2A
#define ADXL_BW_RATE            0x2C
#define ADXL_POWER_CTL          0x2D
#define ADXL_INT_ENABLE         0x2E
#define ADXL_INT_MAP            0x2F
#define ADXL_INT_SOURCE         0x30
#define ADXL_DATA_FORMAT        0x31
#define ADXL_DATAX0             0x32

// Valeurs de configuration
#define ADXL_POWER_MEASURE      (1 << 3)
#define ADXL_FULL_RES           (1 << 3)
#define ADXL_RANGE_2G           0x00
#define ADXL_RANGE_4G           0x01
#define ADXL_RANGE_8G           0x02
#define ADXL_RANGE_16G          0x03
#define ADXL_DATA_RATE_100HZ    0x0A

// Interruptions
#define ADXL_INT_DATA_READY     (1 << 7)
#define ADXL_INT_SINGLE_TAP     (1 << 6)

// Timeout I2C
#define ADXL_I2C_TIMEOUT        100 // ms

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} adxl343_axes_t;

// Prototypes des fonctions
HAL_StatusTypeDef ADXL343_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ADXL343_ReadAxes(I2C_HandleTypeDef *hi2c, adxl343_axes_t *axes);
HAL_StatusTypeDef ADXL343_ConfigShock(I2C_HandleTypeDef *hi2c, float thresh_g, float dur_ms);
uint8_t ADXL343_CheckShock(I2C_HandleTypeDef *hi2c); // Retourne 1 si choc, 0 sinon
float ADXL343_ComputeTotalG(float xg, float yg, float zg);
static inline float ADXL343_RawTo_g(int16_t raw_value)
{
	return ((float)raw_value) * 0.0039f;
}

#endif
