#ifndef __IMU_PROCESS_H
#define __IMU_PROCESS_H

#include "i2c.h"
#include "mpu6050.h"

// 定义灵敏度（根据 mpu6050.c 中的配置）
// 加速度计量程: ±2g -> 16384 LSB/g
// 陀螺仪量程: ±2000 dps -> 16.4 LSB/(dps)
#define ACCEL_SENSITIVITY   16384.0f
#define GYRO_SENSITIVITY    16.4f
#define GRAVITY_MSS         9.80665f
#define DEG_TO_RAD          0.01745329251994329576923690768489f


/**
 * @brief IMU 内部计算使用的数据结构
 * @note  单位: g, °/s, °
 */
typedef struct {
    float Accel[3]; // x, y, z 加速度 (g)
    float Gyro[3];  // x, y, z 角速度 (dps)
    float Pitch;    // 俯仰角 (°)
    float Roll;     // 横滚角 (°)
    float Yaw;      // 航向角 (°) - 暂不使用
} IMU_Data_t;


/**
 * @brief 标准化输出的数据结构 (为 CAN/ROS 准备)
 * @note  单位: m/s^2, rad/s, rad
 */
typedef struct
{
    // ROS 标准单位
    float linear_acceleration[3];   // m/s^2
    float angular_velocity[3];      // rad/s
    float attitude[3];              // 俯仰, 横滚, 航向 (rad)

    uint32_t timestamp;             // 时间戳 (ms)
} IMU_Output_t;


void IMU_Process_Init(I2C_HandleTypeDef *hi2c);
void IMU_Process_Update(I2C_HandleTypeDef *hi2c, IMU_Data_t *data, IMU_Output_t *output, float dt);

#endif //__IMU_PROCESS_H