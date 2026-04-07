#ifndef __IMU_PROCESS_H
#define __IMU_PROCESS_H

#include "i2c.h"
#include "mpu6050.h"

// 定义灵敏度（根据 mpu6050.c 中的配置）
// 加速度计量程: ±2g -> 16384 LSB/g
// 陀螺仪量程: ±2000 dps -> 16.4 LSB/(dps)
#define ACCEL_SENSITIVITY   16384.0f
#define GYRO_SENSITIVITY    16.4f

// 姿态数据结构体
typedef struct {
    float Accel[3]; // x, y, z 加速度 (g)
    float Gyro[3];  // x, y, z 角速度 (dps)
    float Pitch;    // 俯仰角 (°)
    float Roll;     // 横滚角 (°)
    float Yaw;      // 航向角 (°) - 暂不使用
} IMU_Data_t;

void IMU_Process_Init(I2C_HandleTypeDef *hi2c);
void IMU_Process_Update(I2C_HandleTypeDef *hi2c, IMU_Data_t *data, float dt);

#endif //__IMU_PROCESS_H
