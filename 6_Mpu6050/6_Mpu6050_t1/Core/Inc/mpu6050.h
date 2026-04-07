#ifndef __MPU6050_H
#define __MPU6050_H

#include "i2c.h"
#include "stdint.h"

/* MPU6050 设备地址 (AD0 = 0) */
#define MPU6050_ADDR (0x68 << 1)

/**
 * @brief 初始化 MPU6050
 * @param hi2c I2C 句柄
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取 MPU6050 的所有原始数据
 * @param hi2c I2C 句柄
 * @param accel 加速度计数据指针 (int16_t[3])
 * @param gyro 陀螺仪数据指针 (int16_t[3])
 */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t *accel, int16_t *gyro);

#endif /* __MPU6050_H */
