#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f1xx_hal.h"

// I2C地址（SDO接GND）
#define BMP280_ADDR (0x76 << 1)

// 寄存器
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_XLSB   0xF9
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_XLSB    0xFC

// Oversampling settings
#define BMP280_OVERSAMPLING_SKIPPED 0x00
#define BMP280_OVERSAMPLING_1       0x01
#define BMP280_OVERSAMPLING_2       0x02
#define BMP280_OVERSAMPLING_4       0x03
#define BMP280_OVERSAMPLING_8       0x04
#define BMP280_OVERSAMPLING_16      0x05

// Mode settings
#define BMP280_MODE_SLEEP   0x00
#define BMP280_MODE_FORCED  0x01
#define BMP280_MODE_NORMAL  0x03

// Filter settings
#define BMP280_FILTER_OFF   0x00
#define BMP280_FILTER_2     0x01
#define BMP280_FILTER_4     0x02
#define BMP280_FILTER_8     0x03
#define BMP280_FILTER_16    0x04

// Standby time settings
#define BMP280_STANDBY_0_5      0x00
#define BMP280_STANDBY_62_5     0x01
#define BMP280_STANDBY_125      0x02
#define BMP280_STANDBY_250      0x03
#define BMP280_STANDBY_500      0x04
#define BMP280_STANDBY_1000     0x05
#define BMP280_STANDBY_10       0x06
#define BMP280_STANDBY_20       0x07

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

    // 基准海拔
    float base_altitude;
} bmp280_t;

// API
uint8_t BMP280_ReadID(I2C_HandleTypeDef *hi2c);
void BMP280_Init(I2C_HandleTypeDef *hi2c);
void BMP280_Read_Calibration_Data(I2C_HandleTypeDef *hi2c, bmp280_t *bmp);
void BMP280_Set_Mode(I2C_HandleTypeDef *hi2c, uint8_t mode);
void BMP280_Set_Filter(I2C_HandleTypeDef *hi2c, uint8_t filter);
void BMP280_Set_Standby(I2C_HandleTypeDef *hi2c, uint8_t standby);
void BMP280_Set_Oversampling_Pressure(I2C_HandleTypeDef *hi2c, uint8_t os);
void BMP280_Set_Oversampling_Temperature(I2C_HandleTypeDef *hi2c, uint8_t os);
void BMP280_Read_Float(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure, float *altitude);
void BMP280_Set_Base_Altitude(I2C_HandleTypeDef *hi2c);

#endif