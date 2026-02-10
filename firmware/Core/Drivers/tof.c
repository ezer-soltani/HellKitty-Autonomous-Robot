#include "tof.h"
#include "main.h"  // Required for GPIO definitions (XSHUT Pins)
#include "i2c.h"   // Required for hi2c1 handle
#include <string.h>

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
VL53L0X_Dev_t tof1, tof2, tof3, tof4;

// Internal buffers
static uint8_t msgBuffer[4];
static HAL_StatusTypeDef i2cStat;

// Constants
#define I2C_TIMEOUT 100
#define I2C_READ 1
#define I2C_WRITE 0

// Register Addresses (Private)
enum regAddr {
  SYSRANGE_START                              = 0x00,
  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,
  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
  MSRC_CONFIG_CONTROL                         = 0x60,
  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,
  OSC_CALIBRATE_VAL                           = 0xF8,
  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,
  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,
  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

//=============================================================================
// PRIVATE FUNCTION PROTOTYPES
//=============================================================================
static bool getSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture);
static void getSequenceStepEnables(VL53L0X_Dev_t *dev, SequenceStepEnables * enables);
static void getSequenceStepTimeouts(VL53L0X_Dev_t *dev, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
static bool performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static void writeReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value);
static void writeReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value);
static void writeReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value);
static uint8_t readReg(VL53L0X_Dev_t *dev, uint8_t reg);
static uint16_t readReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg);
static uint32_t readReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg);
static void writeMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t const *src, uint8_t count);
static void readMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t * dst, uint8_t count);

//=============================================================================
// HIGH LEVEL API
//=============================================================================

void TOF_Init_All(void) {
    // 0. Reconfigure TOF Interrupt Pins as Input (Disable EXTI)
    // All TOF GPIOs are on GPIOB (Checked in gpio.c)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TOF1_GPIO_Pin | TOF2_GPIO_Pin | TOF3_GPIO_Pin | TOF4_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 1. Reset all sensors (XSHUT Low)
    HAL_GPIO_WritePin(TOF1_XSHUT_GPIO_Port, TOF1_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF2_XSHUT_GPIO_Port, TOF2_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF3_XSHUT_GPIO_Port, TOF3_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF4_XSHUT_GPIO_Port, TOF4_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);

    // 2. Initialize TOF1 (Addr 0x54 from Test_TOF)
    HAL_GPIO_WritePin(TOF1_XSHUT_GPIO_Port, TOF1_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    initVL53L0X(&tof1, 1, &hi2c1);
    setAddress_VL53L0X(&tof1, 0x54);
    setSignalRateLimit(&tof1, 0.1);
    setVcselPulsePeriod(&tof1, VcselPeriodPreRange, 18);
    setVcselPulsePeriod(&tof1, VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(&tof1, 20000);
    startContinuous(&tof1, 0);

    // 3. Initialize TOF2 (Addr 0x56 from Test_TOF)
    HAL_GPIO_WritePin(TOF2_XSHUT_GPIO_Port, TOF2_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    initVL53L0X(&tof2, 1, &hi2c1);
    setAddress_VL53L0X(&tof2, 0x56);
    setSignalRateLimit(&tof2, 0.1);
    setVcselPulsePeriod(&tof2, VcselPeriodPreRange, 18);
    setVcselPulsePeriod(&tof2, VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(&tof2, 20000);
    startContinuous(&tof2, 0);

    // 4. Initialize TOF3 (Addr 0x58 from Test_TOF)
    HAL_GPIO_WritePin(TOF3_XSHUT_GPIO_Port, TOF3_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    initVL53L0X(&tof3, 1, &hi2c1);
    setAddress_VL53L0X(&tof3, 0x58);
    setSignalRateLimit(&tof3, 0.1);
    setVcselPulsePeriod(&tof3, VcselPeriodPreRange, 18);
    setVcselPulsePeriod(&tof3, VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(&tof3, 20000);
    startContinuous(&tof3, 0);

    // 5. Initialize TOF4 (Addr 0x5A from Test_TOF)
    HAL_GPIO_WritePin(TOF4_XSHUT_GPIO_Port, TOF4_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    initVL53L0X(&tof4, 1, &hi2c1);
            setAddress_VL53L0X(&tof4, 0x5A);
            setSignalRateLimit(&tof4, 0.1);
            setVcselPulsePeriod(&tof4, VcselPeriodPreRange, 18);
            setVcselPulsePeriod(&tof4, VcselPeriodFinalRange, 14);
            setMeasurementTimingBudget(&tof4, 20000);
            startContinuous(&tof4, 0);
}

void TOF_Set_Interrupt_Threshold(VL53L0X_Dev_t *dev, uint16_t threshold_mm) {
    // 1. Set Threshold High
    writeReg16Bit(dev, SYSTEM_THRESH_HIGH, threshold_mm);
    writeReg16Bit(dev, SYSTEM_THRESH_LOW, 0); 

    // 2. Configure GPIO for "Level High" (Value > Thresh_High)
    // 0x02 = Level High (Active when distance > High Threshold)
    writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x02);
    
    // 3. Ensure Polarity is Active Low (bit 4 = 0)
    // This matches STM32 GPIO_MODE_IT_FALLING
    uint8_t gpio_mux = readReg(dev, GPIO_HV_MUX_ACTIVE_HIGH);
    writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, gpio_mux & ~0x10); 
    
    // 4. Clear any pending interrupt
    writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
}

void TOF_Clear_Interrupt(VL53L0X_Dev_t *dev) {
    writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
}

uint8_t TOF_Read_All(uint16_t* distances) {
    if (!distances) return 1;
    distances[0] = readRangeContinuousMillimeters(&tof1, 0);
    distances[1] = readRangeContinuousMillimeters(&tof2, 0);
    distances[2] = readRangeContinuousMillimeters(&tof3, 0);
    distances[3] = readRangeContinuousMillimeters(&tof4, 0);
    return 0;
}


//=============================================================================
// LOW LEVEL DRIVER IMPLEMENTATION (ST)
//=============================================================================

// I2C Helpers
static void writeReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value) {
  msgBuffer[0] = value;
  i2cStat = HAL_I2C_Mem_Write(dev->I2cHandle, dev->I2cDevAddr | I2C_WRITE, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
}

static void writeReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value){
  uint8_t temp[2];
  temp[0] = (value >> 8) & 0xFF;
  temp[1] = value & 0xFF;
  i2cStat = HAL_I2C_Mem_Write(dev->I2cHandle, dev->I2cDevAddr | I2C_WRITE, reg, 1, temp, 2, I2C_TIMEOUT);
}

static void writeReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value){
  uint8_t temp[4];
  temp[0] = (value >> 24) & 0xFF;
  temp[1] = (value >> 16) & 0xFF;
  temp[2] = (value >> 8) & 0xFF;
  temp[3] = value & 0xFF;
  i2cStat = HAL_I2C_Mem_Write(dev->I2cHandle, dev->I2cDevAddr | I2C_WRITE, reg, 1, temp, 4, I2C_TIMEOUT);
}

static uint8_t readReg(VL53L0X_Dev_t *dev, uint8_t reg) {
  uint8_t value;
  i2cStat = HAL_I2C_Mem_Read(dev->I2cHandle, dev->I2cDevAddr | I2C_READ, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
  value = msgBuffer[0];
  return value;
}

static uint16_t readReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg) {
  uint16_t value;
  i2cStat = HAL_I2C_Mem_Read(dev->I2cHandle, dev->I2cDevAddr | I2C_READ, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
  value = (uint16_t)((msgBuffer[0] << 8) | msgBuffer[1]);
  return value;
}

static uint32_t readReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg) {
  uint32_t value;
  i2cStat = HAL_I2C_Mem_Read(dev->I2cHandle, dev->I2cDevAddr | I2C_READ, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
  value = (uint32_t)((msgBuffer[0] << 24) | (msgBuffer[1] << 16) | (msgBuffer[2] << 8) | msgBuffer[3]);
  return value;
}

static void writeMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t const *src, uint8_t count){
  i2cStat = HAL_I2C_Mem_Write(dev->I2cHandle, dev->I2cDevAddr | I2C_WRITE, reg, 1, (uint8_t*)src, count, I2C_TIMEOUT);
}

static void readMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t * dst, uint8_t count) {
	i2cStat = HAL_I2C_Mem_Read(dev->I2cHandle, dev->I2cDevAddr | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT);
}


// Public Methods

void setAddress_VL53L0X(VL53L0X_Dev_t *dev, uint8_t new_addr) {
  writeReg(dev, I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F );
  dev->I2cDevAddr = new_addr;
}

uint8_t getAddress_VL53L0X(VL53L0X_Dev_t *dev) {
  return dev->I2cDevAddr;
}

uint8_t initVL53L0X(VL53L0X_Dev_t *dev, bool io_2v8, I2C_HandleTypeDef *handler){
  // Handler
  dev->I2cHandle = handler;
  dev->I2cDevAddr = ADDRESS_DEFAULT;
  dev->ioTimeout = 0;
  dev->isTimeout = false;

  if (io_2v8)
  {
    writeReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  writeReg(dev, 0x88, 0x00);
  writeReg(dev, 0x80, 0x01);
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);
  dev->stopVariable = readReg(dev, 0x91);
  writeReg(dev, 0x00, 0x01);
  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(dev, MSRC_CONFIG_CONTROL, readReg(dev, MSRC_CONFIG_CONTROL) | 0x12);
  setSignalRateLimit(dev, 0.25);
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(dev, &spad_count, &spad_type_is_aperture)) { return false; }

  uint8_t ref_spad_map[6];
  readMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_load_tuning_settings() begin
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x09, 0x00);
  writeReg(dev, 0x10, 0x00);
  writeReg(dev, 0x11, 0x00);

  writeReg(dev, 0x24, 0x01);
  writeReg(dev, 0x25, 0xFF);
  writeReg(dev, 0x75, 0x00);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x4E, 0x2C);
  writeReg(dev, 0x48, 0x00);
  writeReg(dev, 0x30, 0x20);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x30, 0x09);
  writeReg(dev, 0x54, 0x00);
  writeReg(dev, 0x31, 0x04);
  writeReg(dev, 0x32, 0x03);
  writeReg(dev, 0x40, 0x83);
  writeReg(dev, 0x46, 0x25);
  writeReg(dev, 0x60, 0x00);
  writeReg(dev, 0x27, 0x00);
  writeReg(dev, 0x50, 0x06);
  writeReg(dev, 0x51, 0x00);
  writeReg(dev, 0x52, 0x96);
  writeReg(dev, 0x56, 0x08);
  writeReg(dev, 0x57, 0x30);
  writeReg(dev, 0x61, 0x00);
  writeReg(dev, 0x62, 0x00);
  writeReg(dev, 0x64, 0x00);
  writeReg(dev, 0x65, 0x00);
  writeReg(dev, 0x66, 0xA0);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x22, 0x32);
  writeReg(dev, 0x47, 0x14);
  writeReg(dev, 0x49, 0xFF);
  writeReg(dev, 0x4A, 0x00);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x7A, 0x0A);
  writeReg(dev, 0x7B, 0x00);
  writeReg(dev, 0x78, 0x21);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x23, 0x34);
  writeReg(dev, 0x42, 0x00);
  writeReg(dev, 0x44, 0xFF);
  writeReg(dev, 0x45, 0x26);
  writeReg(dev, 0x46, 0x05);
  writeReg(dev, 0x40, 0x40);
  writeReg(dev, 0x0E, 0x06);
  writeReg(dev, 0x20, 0x1A);
  writeReg(dev, 0x43, 0x40);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x34, 0x03);
  writeReg(dev, 0x35, 0x44);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x31, 0x04);
  writeReg(dev, 0x4B, 0x09);
  writeReg(dev, 0x4C, 0x05);
  writeReg(dev, 0x4D, 0x04);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x44, 0x00);
  writeReg(dev, 0x45, 0x20);
  writeReg(dev, 0x47, 0x08);
  writeReg(dev, 0x48, 0x28);
  writeReg(dev, 0x67, 0x00);
  writeReg(dev, 0x70, 0x04);
  writeReg(dev, 0x71, 0x01);
  writeReg(dev, 0x72, 0xFE);
  writeReg(dev, 0x76, 0x00);
  writeReg(dev, 0x77, 0x00);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x0D, 0x01);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x01);
  writeReg(dev, 0x01, 0xF8);

  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x8E, 0x01);
  writeReg(dev, 0x00, 0x01);
  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x00);
  // -- VL53L0X_load_tuning_settings() end

  writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, readReg(dev, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

  dev->measurementTimingBudgetUs = getMeasurementTimingBudget(dev);
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);
  setMeasurementTimingBudget(dev, dev->measurementTimingBudgetUs);

  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(dev, 0x40)) { return false; }
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(dev, 0x00)) { return false; }
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  return true;
}

uint8_t setSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
  writeReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

float getSignalRateLimit(VL53L0X_Dev_t *dev)
{
  return (float)readReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

uint8_t setMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t const MinTimingBudget = 2000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(dev, &enables);
  getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (enables.tcc)
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

  if (enables.dss)
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  else if (enables.msrc)
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

  if (enables.pre_range)
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    if (used_budget_us > budget_us) return false;

    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
      final_range_timeout_mclks += timeouts.pre_range_mclks;

    writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    dev->measurementTimingBudgetUs = budget_us;
  }
  return true;
}

uint32_t getMeasurementTimingBudget(VL53L0X_Dev_t *dev)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(dev, &enables);
  getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (enables.tcc)
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

  if (enables.dss)
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  else if (enables.msrc)
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

  if (enables.pre_range)
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);

  if (enables.final_range)
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);

  dev->measurementTimingBudgetUs = budget_us;
  return budget_us;
}

uint8_t setVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(dev, &enables);
  getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (type == VcselPeriodPreRange)
  {
    switch (period_pclks)
    {
      case 12: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return false;
    }
    writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(dev, 0xFF, 0x01);
        writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
        writeReg(dev, 0xFF, 0x00);
        break;

      case 10:
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(dev, 0xFF, 0x01);
        writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        writeReg(dev, 0xFF, 0x00);
        break;

      case 12:
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(dev, 0xFF, 0x01);
        writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        writeReg(dev, 0xFF, 0x00);
        break;

      case 14:
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(dev, 0xFF, 0x01);
        writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        writeReg(dev, 0xFF, 0x00);
        break;

      default: return false;
    }

    writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;

    writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));
  }
  else { return false; }

  setMeasurementTimingBudget(dev, dev->measurementTimingBudgetUs);

  uint8_t sequence_config = readReg(dev, SYSTEM_SEQUENCE_CONFIG);
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(dev, 0x0);
  writeReg(dev, SYSTEM_SEQUENCE_CONFIG, sequence_config);

  return true;
}

uint8_t getVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
    return decodeVcselPeriod(readReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  else if (type == VcselPeriodFinalRange)
    return decodeVcselPeriod(readReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  else return 255;
}

void startContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms)
{
  writeReg(dev, 0x80, 0x01);
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);
  writeReg(dev, 0x91, dev->stopVariable);
  writeReg(dev, 0x00, 0x01);
  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x00);

  if (period_ms != 0)
  {
    uint16_t osc_calibrate_val = readReg16Bit(dev, OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
      period_ms *= osc_calibrate_val;

    writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    writeReg(dev, SYSRANGE_START, 0x04); // TIMED
  }
  else
  {
    writeReg(dev, SYSRANGE_START, 0x02); // BACKTOBACK
  }
}

void stopContinuous(VL53L0X_Dev_t *dev)
{
  writeReg(dev, SYSRANGE_START, 0x01); // SINGLESHOT
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);
  writeReg(dev, 0x91, 0x00);
  writeReg(dev, 0x00, 0x01);
  writeReg(dev, 0xFF, 0x00);
}

uint16_t readRangeContinuousMillimeters(VL53L0X_Dev_t *dev, statInfo_t_VL53L0X *extraStats ) {
  uint8_t tempBuffer[12];
  uint16_t temp;
  startTimeout(dev);
  while ((readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpired(dev))
    {
      dev->isTimeout = true;
      return 65535;
    }
  }
  if( extraStats == 0 ){
    temp = readReg16Bit(dev, RESULT_RANGE_STATUS + 10);
  } else {
    readMulti(dev, 0x14, tempBuffer, 12);
    extraStats->rangeStatus =  tempBuffer[0x00]>>3;
    extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
    extraStats->signalCnt   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
    extraStats->ambientCnt  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];    
    temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
    extraStats->rawDistance = temp;
  }
  writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
  return temp;
}

uint16_t readRangeSingleMillimeters(VL53L0X_Dev_t *dev, statInfo_t_VL53L0X *extraStats ) {
  writeReg(dev, 0x80, 0x01);
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);
  writeReg(dev, 0x91, dev->stopVariable);
  writeReg(dev, 0x00, 0x01);
  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x00);
  writeReg(dev, SYSRANGE_START, 0x01);
  startTimeout(dev);
  while (readReg(dev, SYSRANGE_START) & 0x01){
    if (checkTimeoutExpired(dev)){
      dev->isTimeout = true;
      return 65535;
    }
  }
  return readRangeContinuousMillimeters(dev, extraStats );
}

bool timeoutOccurred(VL53L0X_Dev_t *dev)
{
  bool tmp = dev->isTimeout;
  dev->isTimeout = false;
  return tmp;
}

void setTimeout(VL53L0X_Dev_t *dev, uint16_t timeout){
  dev->ioTimeout = timeout;
}

uint16_t getTimeout(VL53L0X_Dev_t *dev){
  return dev->ioTimeout;
}

// Private Helpers
static bool getSpadInfo(VL53L0X_Dev_t *dev, uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;
  writeReg(dev, 0x80, 0x01);
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x00);

  writeReg(dev, 0xFF, 0x06);
  writeReg(dev, 0x83, readReg(dev, 0x83) | 0x04);
  writeReg(dev, 0xFF, 0x07);
  writeReg(dev, 0x81, 0x01);

  writeReg(dev, 0x80, 0x01);

  writeReg(dev, 0x94, 0x6b);
  writeReg(dev, 0x83, 0x00);
  startTimeout(dev);
  while (readReg(dev, 0x83) == 0x00)
  {
    if (checkTimeoutExpired(dev)) { return false; }
  }
  writeReg(dev, 0x83, 0x01);
  tmp = readReg(dev, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(dev, 0x81, 0x00);
  writeReg(dev, 0xFF, 0x06);
  writeReg(dev, 0x83, readReg(dev, 0x83)  & ~0x04);
  writeReg(dev, 0xFF, 0x01);
  writeReg(dev, 0x00, 0x01);

  writeReg(dev, 0xFF, 0x00);
  writeReg(dev, 0x80, 0x00);

  return true;
}

static void getSequenceStepEnables(VL53L0X_Dev_t *dev, SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(dev, SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

static void getSequenceStepTimeouts(VL53L0X_Dev_t *dev, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(dev, VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(dev, VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

static uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static bool performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte)
{
  writeReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte);
  startTimeout(dev);
  while ((readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired(dev)) { return false; }
  }
  writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(dev, SYSRANGE_START, 0x00);
  return true;
}
