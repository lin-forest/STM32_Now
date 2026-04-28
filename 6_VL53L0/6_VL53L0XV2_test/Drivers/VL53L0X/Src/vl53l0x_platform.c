/**
 * @file    vl53l0x_platform.c
 * @brief   VL53L0X HAL I2C platform implementation for STM32F103
 *
 *  All transfers use STM32 HAL I2C with memory-address style writes/reads:
 *  the register address is sent as the 8-bit memory address parameter so
 *  that HAL handles the I2C framing automatically.
 */

#include "vl53l0x_platform.h"

/* -----------------------------------------------------------------------
 * Write operations
 * --------------------------------------------------------------------- */

HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             (uint16_t)(i2c_addr << 1),
                             (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1U,
                             VL53L0X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL53L0X_WriteReg16(uint8_t i2c_addr, uint8_t reg, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);   /* MSB first (big-endian) */
    buf[1] = (uint8_t)(data & 0xFFU);

    return HAL_I2C_Mem_Write(&hi2c1,
                             (uint16_t)(i2c_addr << 1),
                             (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             2U,
                             VL53L0X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL53L0X_WriteMulti(uint8_t i2c_addr, uint8_t reg,
                                      const uint8_t *pData, uint16_t len)
{
    /* HAL_I2C_Mem_Write takes non-const pData; cast is safe here */
    return HAL_I2C_Mem_Write(&hi2c1,
                             (uint16_t)(i2c_addr << 1),
                             (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)pData,
                             len,
                             VL53L0X_I2C_TIMEOUT_MS);
}

/* -----------------------------------------------------------------------
 * Read operations
 * --------------------------------------------------------------------- */

HAL_StatusTypeDef VL53L0X_ReadReg(uint8_t i2c_addr, uint8_t reg, uint8_t *pData)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            (uint16_t)(i2c_addr << 1),
                            (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            1U,
                            VL53L0X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL53L0X_ReadReg16(uint8_t i2c_addr, uint8_t reg, uint16_t *pData)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c1,
                           (uint16_t)(i2c_addr << 1),
                           (uint16_t)reg,
                           I2C_MEMADD_SIZE_8BIT,
                           buf,
                           2U,
                           VL53L0X_I2C_TIMEOUT_MS);
    if (ret == HAL_OK) {
        *pData = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1]; /* big-endian */
    }
    return ret;
}

HAL_StatusTypeDef VL53L0X_ReadMulti(uint8_t i2c_addr, uint8_t reg,
                                     uint8_t *pData, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            (uint16_t)(i2c_addr << 1),
                            (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            len,
                            VL53L0X_I2C_TIMEOUT_MS);
}
