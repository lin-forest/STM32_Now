/**
 * @file    vl53l0x_platform.h
 * @brief   VL53L0X platform adaptation layer for STM32F103 + HAL I2C
 *
 *  Hardware:
 *      SCL -> PB6 (I2C1_SCL)
 *      SDA -> PB7 (I2C1_SDA)
 *      XSHUT -> PA1
 */

#ifndef VL53L0X_PLATFORM_H
#define VL53L0X_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * I2C bus handle – resolved from i2c.h extern
 * --------------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;

/** Default 7-bit I2C address (STM32 HAL needs it shifted left by 1) */
#define VL53L0X_I2C_ADDR_DEFAULT    (0x29U)
#define VL53L0X_I2C_TIMEOUT_MS      (10U)

/* -----------------------------------------------------------------------
 * Low-level I/O primitives (implemented in vl53l0x_platform.c)
 * --------------------------------------------------------------------- */

/**
 * @brief  Write a single byte to a register.
 * @param  i2c_addr   7-bit device address
 * @param  reg        register address
 * @param  data       byte to write
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t i2c_addr, uint8_t reg, uint8_t data);

/**
 * @brief  Write a 16-bit value to a register (big-endian, as the device expects).
 * @param  i2c_addr   7-bit device address
 * @param  reg        register address
 * @param  data       16-bit value
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_WriteReg16(uint8_t i2c_addr, uint8_t reg, uint16_t data);

/**
 * @brief  Write multiple bytes starting at a register.
 * @param  i2c_addr   7-bit device address
 * @param  reg        starting register address
 * @param  pData      pointer to data buffer
 * @param  len        number of bytes
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_WriteMulti(uint8_t i2c_addr, uint8_t reg,
                                      const uint8_t *pData, uint16_t len);

/**
 * @brief  Read a single byte from a register.
 * @param  i2c_addr   7-bit device address
 * @param  reg        register address
 * @param  pData      pointer to store the byte
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_ReadReg(uint8_t i2c_addr, uint8_t reg, uint8_t *pData);

/**
 * @brief  Read a 16-bit value from a register (big-endian, as the device sends).
 * @param  i2c_addr   7-bit device address
 * @param  reg        register address
 * @param  pData      pointer to store the 16-bit value
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_ReadReg16(uint8_t i2c_addr, uint8_t reg, uint16_t *pData);

/**
 * @brief  Read multiple bytes starting at a register.
 * @param  i2c_addr   7-bit device address
 * @param  reg        starting register address
 * @param  pData      pointer to data buffer
 * @param  len        number of bytes
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef VL53L0X_ReadMulti(uint8_t i2c_addr, uint8_t reg,
                                     uint8_t *pData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_PLATFORM_H */
