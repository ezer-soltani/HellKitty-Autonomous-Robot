/*
 * ADXL343_Driver.c
 *
 */

#include "imu.h"
#include <math.h>

// Fonction d'écriture I2C (Interne)
static HAL_StatusTypeDef adxl_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			ADXL_I2C_TIMEOUT);
}

// Fonction de lecture I2C (Interne)
static HAL_StatusTypeDef adxl_read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
	return HAL_I2C_Mem_Read(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			value,
			1,
			ADXL_I2C_TIMEOUT);
}

// Initialisation
HAL_StatusTypeDef ADXL343_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t devid = 0;
	HAL_StatusTypeDef status;

	// Vérifier l'ID du capteur
	status = adxl_read(hi2c, ADXL_DEVID, &devid);
	if (status != HAL_OK) return status;

	if (devid != 0xE5) return HAL_ERROR;

	// 100 Hz
	status = adxl_write(hi2c, ADXL_BW_RATE, ADXL_DATA_RATE_100HZ);
	if (status != HAL_OK) return status;

	// FULL RES + ±2g
	uint8_t data_format = ADXL_FULL_RES | ADXL_RANGE_2G;
	status = adxl_write(hi2c, ADXL_DATA_FORMAT, data_format);
	if (status != HAL_OK) return status;

	// Mode mesure
	status = adxl_write(hi2c, ADXL_POWER_CTL, ADXL_POWER_MEASURE);
	return status;
}

// Lecture des axes
HAL_StatusTypeDef ADXL343_ReadAxes(I2C_HandleTypeDef *hi2c, adxl343_axes_t *axes)
{
	uint8_t buf[6];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(hi2c, ADXL343_ADDR, ADXL_DATAX0, I2C_MEMADD_SIZE_8BIT, buf, 6, ADXL_I2C_TIMEOUT);
	if (status != HAL_OK) return status;

	axes->x = (int16_t)((buf[1] << 8) | buf[0]);
	axes->y = (int16_t)((buf[3] << 8) | buf[2]);
	axes->z = (int16_t)((buf[5] << 8) | buf[4]);

	return HAL_OK;
}

// Configuration détection de choc
HAL_StatusTypeDef ADXL343_ConfigShock(I2C_HandleTypeDef *hi2c, float thresh_g, float dur_ms)
{
	HAL_StatusTypeDef ret;

	// Seuil choc (62.5 mg/LSB)
	const float lsb_g = 0.0625f;
	uint8_t thresh = (uint8_t)(thresh_g / lsb_g);
	if (thresh == 0) thresh = 1;

	ret = adxl_write(hi2c, ADXL_THRESH_TAP, thresh);
	if (ret != HAL_OK) return ret;

	// Durée choc (625 µs/LSB)
	const float lsb_ms = 0.625f;
	uint8_t dur = (uint8_t)(dur_ms / lsb_ms);
	if (dur == 0) dur = 1;

	ret = adxl_write(hi2c,  ADXL_DUR, dur);
	if (ret != HAL_OK) return ret;

	// Activation des 3 axes pour la detection
	ret = adxl_write(hi2c,  ADXL_TAP_AXES, 0x07); // X,Y,Z
	if (ret != HAL_OK) return ret;

	// Enable l'interruption SINGLE_TAP
	uint8_t int_enable = ADXL_INT_DATA_READY | ADXL_INT_SINGLE_TAP;
	ret = adxl_write(hi2c, ADXL_INT_ENABLE, int_enable);
	if (ret != HAL_OK) return ret;

	// Map sur INT2 (0x40)
	uint8_t int_map = 0x40;
	ret = adxl_write(hi2c, ADXL_INT_MAP, int_map);

	return ret;
}

// Vérifier le choc (Retourne 1 si détecté, 0 sinon)
uint8_t ADXL343_CheckShock(I2C_HandleTypeDef *hi2c)
{
	uint8_t src = 0;
	if (adxl_read(hi2c, ADXL_INT_SOURCE, &src) != HAL_OK) {
		return 0; // Erreur de lecture considérée comme "pas de choc"
	}

	return (src & ADXL_INT_SINGLE_TAP) ? 1 : 0;
}

// Calculer l'acceleration totale
float ADXL343_ComputeTotalG(float xg, float yg, float zg)
{
	return sqrtf((xg * xg) + (yg * yg) + (zg * zg));
}
