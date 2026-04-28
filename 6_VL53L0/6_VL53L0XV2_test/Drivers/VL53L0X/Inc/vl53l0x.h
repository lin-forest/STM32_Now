/**
 * @file    vl53l0x.h
 * @brief   VL53L0X ToF sensor driver for STM32F103 (HAL-based)
 *
 * Supports:
 *  - Single continuous ranging mode
 *  - Configurable timing budget
 *  - XSHUT control for address assignment (multi-sensor capable)
 *  - Long range mode (up to ~2 m)
 *
 * Usage:
 *  1. Declare VL53L0X_Dev_t  dev;
 *  2. Call VL53L0X_Init(&dev, ...)
 *  3. Call VL53L0X_StartContinuous(&dev, 0)  // 0 = back-to-back
 *  4. Loop: VL53L0X_ReadRangeContinuousMillimeters(&dev, &mm)
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53l0x_platform.h"
#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Register map (partial – all we need for init + ranging)
 * --------------------------------------------------------------------- */
#define REG_SYSRANGE_START                          0x00
#define REG_SYSTEM_THRESH_HIGH                      0x0C
#define REG_SYSTEM_THRESH_LOW                       0x0E
#define REG_SYSTEM_SEQUENCE_CONFIG                  0x01
#define REG_SYSTEM_RANGE_CONFIG                     0x09
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define REG_SYSTEM_INTERRUPT_CLEAR                  0x0B
#define REG_RESULT_INTERRUPT_STATUS                 0x13
#define REG_RESULT_RANGE_STATUS                     0x14
#define REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN   0xBC
#define REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN    0xC0
#define REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF   0xD0
#define REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF    0xD4
#define REG_RESULT_PEAK_SIGNAL_RATE_REF             0xB6
#define REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM       0x28
#define REG_I2C_SLAVE_DEVICE_ADDRESS                0x8A
#define REG_MSRC_CONFIG_CONTROL                     0x60
#define REG_PRE_RANGE_CONFIG_MIN_SNR                0x27
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57
#define REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT      0x64
#define REG_FINAL_RANGE_CONFIG_MIN_SNR              0x67
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI        0x61
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO        0x62
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO      0x52
#define REG_SYSTEM_HISTOGRAM_BIN                    0x81
#define REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT   0x33
#define REG_HISTOGRAM_CONFIG_READOUT_CTRL           0x55
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO    0x72
#define REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS   0x20
#define REG_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define REG_SOFT_RESET_GO2_SOFT_RESET_N             0xBF
#define REG_IDENTIFICATION_MODEL_ID                 0xC0
#define REG_IDENTIFICATION_REVISION_ID              0xC2
#define REG_OSC_CALIBRATE_VAL                       0xF8
#define REG_GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1        0xB1
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2        0xB2
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3        0xB3
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4        0xB4
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5        0xB5
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F
#define REG_POWER_MANAGEMENT_GO1_POWER_FORCE        0x80
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define REG_ALGO_PHASECAL_LIM                       0x30
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

/* -----------------------------------------------------------------------
 * Timing budget presets (µs)
 * --------------------------------------------------------------------- */
#define VL53L0X_TIMING_BUDGET_20MS      20000U
#define VL53L0X_TIMING_BUDGET_33MS      33000U
#define VL53L0X_TIMING_BUDGET_100MS    100000U   /**< Good accuracy */
#define VL53L0X_TIMING_BUDGET_200MS    200000U   /**< High accuracy */

/* Out-of-range sentinel (the device returns 8190 or 8191 when out of range) */
#define VL53L0X_OUT_OF_RANGE            8190U

/* -----------------------------------------------------------------------
 * VCSEL period types
 * --------------------------------------------------------------------- */
typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE  = 0,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE = 1,
} VL53L0X_VcselPeriodType_t;

/* -----------------------------------------------------------------------
 * Device descriptor
 * --------------------------------------------------------------------- */
typedef struct {
    uint8_t  i2c_addr;           /**< 7-bit I2C address                  */
    uint8_t  stop_variable;      /**< Saved during init, used for ranging */
    uint32_t timing_budget_us;   /**< Measurement timing budget in µs     */
    bool     continuous_mode;    /**< True when continuous ranging is on   */
} VL53L0X_Dev_t;

/* -----------------------------------------------------------------------
 * Status codes
 * --------------------------------------------------------------------- */
typedef enum {
    VL53L0X_OK            =  0,
    VL53L0X_ERR_I2C      = -1,
    VL53L0X_ERR_ID       = -2,
    VL53L0X_ERR_TIMEOUT  = -3,
    VL53L0X_ERR_SPAD     = -4,
    VL53L0X_ERR_PARAM    = -5,
} VL53L0X_Status_t;

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/**
 * @brief  Initialise the VL53L0X sensor.
 *
 * Performs soft-reset, SPAD calibration, reference calibration and loads
 * the standard set of register initialisations from ST Application Note.
 *
 * @param  dev       pointer to device descriptor
 * @param  i2c_addr  7-bit I2C address (default: VL53L0X_I2C_ADDR_DEFAULT)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_Init(VL53L0X_Dev_t *dev, uint8_t i2c_addr);

/**
 * @brief  Set the measurement timing budget (controls accuracy vs. speed).
 * @param  dev           pointer to device descriptor
 * @param  budget_us     timing budget in microseconds (min ~20000)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_Dev_t *dev,
                                                      uint32_t budget_us);

/**
 * @brief  Configure VCSEL period (affects range and accuracy).
 * @param  dev          pointer to device descriptor
 * @param  type         pre-range or final-range
 * @param  period_pclks VCSEL period in PCLKs (must be even, 6–18 pre-range,
 *                      8–14 final-range)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetVcselPulsePeriod(VL53L0X_Dev_t *dev,
                                               VL53L0X_VcselPeriodType_t type,
                                               uint8_t period_pclks);

/**
 * @brief  Enable long-range mode (trades accuracy for range, up to ~2 m).
 *
 * Adjusts VCSEL periods and timing budget to 200 ms. Call after Init().
 *
 * @param  dev pointer to device descriptor
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetLongRangeMode(VL53L0X_Dev_t *dev);

/**
 * @brief  Start continuous ranging.
 * @param  dev            pointer to device descriptor
 * @param  period_ms      inter-measurement period in ms; 0 = back-to-back
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_StartContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms);

/**
 * @brief  Stop continuous ranging.
 * @param  dev pointer to device descriptor
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_StopContinuous(VL53L0X_Dev_t *dev);

/**
 * @brief  Read the latest range in continuous mode.
 *
 * Blocks until data is ready (polls interrupt status register).
 *
 * @param  dev       pointer to device descriptor
 * @param  range_mm  pointer to store the range in millimetres
 * @retval VL53L0X_OK on success; VL53L0X_ERR_TIMEOUT if no data within 1 s
 */
VL53L0X_Status_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_Dev_t *dev,
                                                          uint16_t *range_mm);

/**
 * @brief  Perform a single ranging measurement (blocking).
 * @param  dev       pointer to device descriptor
 * @param  range_mm  pointer to store the range in millimetres
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev,
                                                      uint16_t *range_mm);

/**
 * @brief  Check if the last reading is out-of-range.
 * @param  range_mm  value returned by a Read function
 * @retval true if sensor is out of range
 */
static inline bool VL53L0X_IsOutOfRange(uint16_t range_mm)
{
    return (range_mm >= VL53L0X_OUT_OF_RANGE);
}

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_H */
