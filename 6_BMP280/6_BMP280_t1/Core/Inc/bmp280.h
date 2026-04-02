#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f1xx_hal.h"

// I2C地址（SDO接GND）
#define BMP280_ADDR (0x76 << 1)

// 寄存器
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_TEMP_MSB  0xFA

typedef struct bmp280_t
{
    // 原始数据
    int32_t raw_temp;
    int32_t raw_press;

    // 校准参数
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    // 补偿后的数据
    float temperature;
    float pressure;
    float altitude;
} bmp280_t;

// API
uint8_t BMP280_ReadID(I2C_HandleTypeDef *hi2c);
void BMP280_Init(I2C_HandleTypeDef *hi2c);
int32_t BMP280_ReadTempRaw(I2C_HandleTypeDef *hi2c);
void BMP280_Read_Float(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure, float *altitude);

#endif