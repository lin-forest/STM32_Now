#include "bmp280.h"
#include "i2c.h"
#include <math.h>

// 内部静态函数的原型声明 (前向声明)
static void BMP280_Read_Raw(I2C_HandleTypeDef *hi2c, bmp280_t *bmp);
static void bmp280_compensate_T_float(bmp280_t *bmp);
static void bmp280_compensate_P_float(bmp280_t *bmp);
static void bmp280_calculate_altitude(bmp280_t *bmp);

// 全局传感器实例
bmp280_t bmp280;
// 全局温度补偿中间值
int32_t t_fine;

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

void BMP280_Init(I2C_HandleTypeDef *hi2c)
{
    BMP280_Read_Calibration_Data(hi2c, &bmp280);
    BMP280_Set_Mode(hi2c, BMP280_MODE_NORMAL);
    BMP280_Set_Standby(hi2c, BMP280_STANDBY_1000);
    BMP280_Set_Filter(hi2c, BMP280_FILTER_4);
    BMP280_Set_Oversampling_Pressure(hi2c, BMP280_OVERSAMPLING_4);
    BMP280_Set_Oversampling_Temperature(hi2c, BMP280_OVERSAMPLING_1);

    HAL_Delay(100); // 等待传感器稳定并完成第一次测量

    // --- IIR 滤波器初始化 ---
    // 1. 执行一次初始读数，为滤波器提供“种子”值
    //    这避免了从0开始的漫长收敛过程
    BMP280_Read_Raw(hi2c, &bmp280);
    bmp280_compensate_T_float(&bmp280);
    bmp280_compensate_P_float(&bmp280);
    bmp280_calculate_altitude(&bmp280);

    // 2. 此时，bmp280.temperature, .pressure, .altitude 中存储的是第一个原始值
    //    它们将作为下一次滤波计算的“上一次滤波值”
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

void BMP280_Set_Mode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    reg_data &= ~0x03; // Clear mode bits
    reg_data |= mode;
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
}

void BMP280_Set_Standby(I2C_HandleTypeDef *hi2c, uint8_t standby)
{
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    reg_data &= ~0xE0; // Clear standby bits
    reg_data |= (standby << 5);
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
}

void BMP280_Set_Filter(I2C_HandleTypeDef *hi2c, uint8_t filter)
{
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    reg_data &= ~0x1C; // Clear filter bits
    reg_data |= (filter << 2);
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
}

void BMP280_Set_Oversampling_Pressure(I2C_HandleTypeDef *hi2c, uint8_t os)
{
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    reg_data &= ~0x1C; // Clear pressure oversampling bits
    reg_data |= (os << 2);
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
}

void BMP280_Set_Oversampling_Temperature(I2C_HandleTypeDef *hi2c, uint8_t os)
{
    uint8_t reg_data;
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
    reg_data &= ~0xE0; // Clear temperature oversampling bits
    reg_data |= (os << 5);
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
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

// IIR 滤波器系数 (alpha)，取值范围 0.0 < alpha <= 1.0
// alpha 越小，滤波效果越平滑，但响应越慢
// alpha 越大，响应越快，但滤波效果越弱
#define BMP280_IIR_ALPHA 0.2f

void BMP280_Read_Float(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure, float *altitude)
{
    // 1. 保存上一次的滤波输出值
    float prev_temp = bmp280.temperature;
    float prev_press = bmp280.pressure;
    float prev_alt = bmp280.altitude;

    // 2. 读取原始值并计算出当前的“瞬时”数据 (这会覆盖结构体中的值)
    BMP280_Read_Raw(hi2c, &bmp280);
    bmp280_compensate_T_float(&bmp280);
    bmp280_compensate_P_float(&bmp280);
    bmp280_calculate_altitude(&bmp280);

    // 3. 应用一阶IIR低通滤波器
    // 公式: y(t) = alpha * x(t) + (1 - alpha) * y(t-1)
    // x(t) 是当前瞬时值 (已在第2步存入结构体), y(t-1) 是上一次的滤波输出值 (在第1步保存)
    bmp280.temperature = BMP280_IIR_ALPHA * bmp280.temperature + (1.0f - BMP280_IIR_ALPHA) * prev_temp;
    bmp280.pressure = BMP280_IIR_ALPHA * bmp280.pressure + (1.0f - BMP280_IIR_ALPHA) * prev_press;
    bmp280.altitude = BMP280_IIR_ALPHA * bmp280.altitude + (1.0f - BMP280_IIR_ALPHA) * prev_alt;

    // 4. 返回最终的滤波值（海拔为相对海拔）
    *temperature = bmp280.temperature;
    *pressure = bmp280.pressure;
    *altitude = bmp280.altitude - bmp280.base_altitude;
}

/**
 * @brief 将当前滤波后的稳定海拔值设为基准海拔零点
 */
void BMP280_Set_Base_Altitude(I2C_HandleTypeDef *hi2c)
{
    // 直接将当前经过IIR滤波后的稳定海拔值设为基准
    // 此函数应在BMP280_Init()之后，主循环开始之前调用
    // (hi2c 参数保留以与函数原型保持一致，但在此函数中未使用)
    (void)hi2c; // 避免编译器产生“未使用参数”的警告
    bmp280.base_altitude = bmp280.altitude;
}