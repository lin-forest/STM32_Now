#include "mpu6050.h"

// MPU6050 寄存器地址
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // 1. 唤醒 MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

    // 2. 设置陀螺仪满量程范围: ±2000 dps
    data = 0x18;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    // 3. 设置加速度计满量程范围: ±2g
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t *accel, int16_t *gyro)
{
    uint8_t data[14];

    // 从 ACCEL_XOUT_H (0x3B) 开始，连续读取 14 个字节
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 14, HAL_MAX_DELAY);

    // 组合数据
    // 加速度计数据
    accel[0] = (int16_t)(data[0] << 8 | data[1]);  // X
    accel[1] = (int16_t)(data[2] << 8 | data[3]);  // Y
    accel[2] = (int16_t)(data[4] << 8 | data[5]);  // Z

    // 跳过温度数据 (data[6] 和 data[7])

    // 陀螺仪数据
    gyro[0] = (int16_t)(data[8] << 8 | data[9]);   // X
    gyro[1] = (int16_t)(data[10] << 8 | data[11]); // Y
    gyro[2] = (int16_t)(data[12] << 8 | data[13]); // Z
}
