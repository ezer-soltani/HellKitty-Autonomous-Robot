#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

//------------------------------------------------------------
// Constants & Definitions
//------------------------------------------------------------
#define ADDRESS_DEFAULT 0b01010010
#define bool  uint8_t
#define true  1
#define false 0
//------------------------------------------------------------
// Structures & Enums
//------------------------------------------------------------
typedef enum { 
    VcselPeriodPreRange, 
    VcselPeriodFinalRange 
} vcselPeriodType;

typedef struct {
  uint16_t rawDistance;
  uint16_t signalCnt;
  uint16_t ambientCnt;
  uint16_t spadCnt;
  uint8_t  rangeStatus;
} statInfo_t_VL53L0X;

typedef struct {
    I2C_HandleTypeDef *I2cHandle;
    uint8_t I2cDevAddr;
    uint16_t ioTimeout;
    bool isTimeout;
    uint16_t timeoutStartMs;
    uint8_t stopVariable;
    uint32_t measurementTimingBudgetUs;
} VL53L0X_Dev_t;

typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
} SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

//------------------------------------------------------------
// GLOBAL INSTANCES (Declared in tof.c)
//------------------------------------------------------------
extern VL53L0X_Dev_t tof1, tof2, tof3, tof4;

//------------------------------------------------------------
// HIGH LEVEL API (Project Specific)
//------------------------------------------------------------

/**
 * @brief  Initializes all 4 TOF sensors with unique addresses.
 *         Manages XSHUT sequence.
 * @note   Blocking function (uses HAL_Delay).
 */
void TOF_Init_All(void);

/**
 * @brief  Reads current distance from all sensors.
 * @param  distances: Pointer to array of 4 uint16_t.
 * @return 0 on success.
 */
uint8_t TOF_Read_All(uint16_t* distances);

/**
 * @brief  Configures the interrupt threshold for a sensor.
 * @param  dev: Pointer to the VL53L0X device.
 * @param  threshold_mm: Distance in mm above which an interrupt is fired.
 */
void TOF_Set_Interrupt_Threshold(VL53L0X_Dev_t *dev, uint16_t threshold_mm);

/**
 * @brief  Clears the interrupt flag on the sensor (via I2C).
 * @param  dev: Pointer to the VL53L0X device.
 */
void TOF_Clear_Interrupt(VL53L0X_Dev_t *dev);



//------------------------------------------------------------
// LOW LEVEL API (ST VL53L0X Driver)
//------------------------------------------------------------
void setAddress_VL53L0X(VL53L0X_Dev_t *dev, uint8_t new_addr);
uint8_t getAddress_VL53L0X(VL53L0X_Dev_t *dev);
uint8_t initVL53L0X(VL53L0X_Dev_t *dev, bool io_2v8, I2C_HandleTypeDef *handler);
uint8_t setSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps);
float getSignalRateLimit(VL53L0X_Dev_t *dev);
uint8_t setMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us);
uint32_t getMeasurementTimingBudget(VL53L0X_Dev_t *dev);
uint8_t setVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks);
uint8_t getVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type);
void startContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms);
void stopContinuous(VL53L0X_Dev_t *dev);
uint16_t readRangeContinuousMillimeters(VL53L0X_Dev_t *dev, statInfo_t_VL53L0X *extraStats);
uint16_t readRangeSingleMillimeters(VL53L0X_Dev_t *dev, statInfo_t_VL53L0X *extraStats);
void setTimeout(VL53L0X_Dev_t *dev, uint16_t timeout);
uint16_t getTimeout(VL53L0X_Dev_t *dev);
bool timeoutOccurred(VL53L0X_Dev_t *dev);

// Macros for timeout management
#define startTimeout(dev) ((dev)->timeoutStartMs = HAL_GetTick())
#define checkTimeoutExpired(dev) ((dev)->ioTimeout > 0 && ((uint16_t)HAL_GetTick() - (dev)->timeoutStartMs) > (dev)->ioTimeout)
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
