#include "bmp280.h"
#include <math.h>

static bmp280_t bmp280;
static int32_t t_fine;

uint8_t BMP280_ReadID(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    HAL_I2C_Mem_Read(hi2c,
                     BMP280_ADDR,
                     BMP280_REG_ID,
                     I2C_MEMADD_SIZE_8BIT,
                     &id,
                     1,
                     100);
    return id;
}

void BMP280_Read_Calibration_Data(I2C_HandleTypeDef *hi2c, bmp280_t *bmp);

void BMP280_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // 复位
    data = 0xB6;
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_RESET, 1, &data, 1, 100);
    HAL_Delay(100);

    // ctrl_meas：温度x1，压力x1，normal模式
    data = 0x27;
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, 1, &data, 1, 100);

    // config：滤波
    data = 0xA0;
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, 1, &data, 1, 100);

    BMP280_Read_Calibration_Data(hi2c, &bmp280);
}

void BMP280_Read_Calibration_Data(I2C_HandleTypeDef *hi2c, bmp280_t *bmp)
{
    uint8_t calib_data[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, calib_data, 24, 100);

    bmp->dig_T1 = (uint16_t)(((uint16_t)calib_data[1] << 8) | calib_data[0]);
    bmp->dig_T2 = (int16_t)(((int16_t)calib_data[3] << 8) | calib_data[2]);
    bmp->dig_T3 = (int16_t)(((int16_t)calib_data[5] << 8) | calib_data[4]);
    bmp->dig_P1 = (uint16_t)(((uint16_t)calib_data[7] << 8) | calib_data[6]);
    bmp->dig_P2 = (int16_t)(((int16_t)calib_data[9] << 8) | calib_data[8]);
    bmp->dig_P3 = (int16_t)(((int16_t)calib_data[11] << 8) | calib_data[10]);
    bmp->dig_P4 = (int16_t)(((int16_t)calib_data[13] << 8) | calib_data[12]);
    bmp->dig_P5 = (int16_t)(((int16_t)calib_data[15] << 8) | calib_data[14]);
    bmp->dig_P6 = (int16_t)(((int16_t)calib_data[17] << 8) | calib_data[16]);
    bmp->dig_P7 = (int16_t)(((int16_t)calib_data[19] << 8) | calib_data[18]);
    bmp->dig_P8 = (int16_t)(((int16_t)calib_data[21] << 8) | calib_data[20]);
    bmp->dig_P9 = (int16_t)(((int16_t)calib_data[23] << 8) | calib_data[22]);
}

int32_t BMP280_ReadTempRaw(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[3];
    int32_t raw;

    HAL_I2C_Mem_Read(hi2c,
                     BMP280_ADDR,
                     BMP280_REG_TEMP_MSB,
                     I2C_MEMADD_SIZE_8BIT,
                     buf,
                     3,
                     100);

    raw = ((int32_t)buf[0] << 12) |
          ((int32_t)buf[1] << 4)  |
          ((int32_t)buf[2] >> 4);

    return raw;
}

static void BMP280_Read_Raw(I2C_HandleTypeDef *hi2c, bmp280_t *bmp)
{
    uint8_t raw_data[6];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

    bmp->raw_press = (int32_t)raw_data[0] << 12 | (int32_t)raw_data[1] << 4 | (int32_t)raw_data[2] >> 4;
    bmp->raw_temp = (int32_t)raw_data[3] << 12 | (int32_t)raw_data[4] << 4 | (int32_t)raw_data[5] >> 4;
}

static void bmp280_compensate_T_float(bmp280_t *bmp)
{
    float var1, var2;
    var1 = (((float)bmp->raw_temp) / 16384.0f - ((float)bmp->dig_T1) / 1024.0f) * ((float)bmp->dig_T2);
    var2 = ((((float)bmp->raw_temp) / 131072.0f - ((float)bmp->dig_T1) / 8192.0f) *
            (((float)bmp->raw_temp) / 131072.0f - ((float)bmp->dig_T1) / 8192.0f)) * ((float)bmp->dig_T3);
    t_fine = (int32_t)(var1 + var2);
    bmp->temperature = (var1 + var2) / 5120.0f;
}

static void bmp280_compensate_P_float(bmp280_t *bmp)
{
    float var1, var2;
    var1 = ((float)t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)bmp->dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)bmp->dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)bmp->dig_P4) * 65536.0f);
    var1 = (((float)bmp->dig_P3) * var1 * var1 / 524288.0f + ((float)bmp->dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)bmp->dig_P1);

    if (var1 == 0.0f)
    {
        bmp->pressure = 0.0f; // avoid exception caused by division by zero
        return;
    }

    float p = 1048576.0f - (float)bmp->raw_press;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)bmp->dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)bmp->dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)bmp->dig_P7)) / 16.0f;
    bmp->pressure = p;
}


static void bmp280_calculate_altitude(bmp280_t *bmp)
{
    const float sea_level_pressure = 101325.0f; // hPa
    bmp->altitude = 44330.0f * (1.0f - powf(bmp->pressure / sea_level_pressure, 1.0f / 5.255f));
}

void BMP280_Read_Float(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure, float *altitude)
{
    BMP280_Read_Raw(hi2c, &bmp280);
    bmp280_compensate_T_float(&bmp280);
    bmp280_compensate_P_float(&bmp280);
    bmp280_calculate_altitude(&bmp280);

    *temperature = bmp280.temperature;
    *pressure = bmp280.pressure;
    *altitude = bmp280.altitude;
}