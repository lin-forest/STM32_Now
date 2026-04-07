#include "imu_process.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>

// 静态变量，用于存储零点偏移
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};

/**
 * @brief IMU 校准函数
 * @param hi2c: I2C 句柄
 * @note  上电后静止调用，采集 500 次数据计算平均偏移
 */
void IMU_Process_Init(I2C_HandleTypeDef *hi2c)
{
    printf("IMU calibration starting...\r\n");

    const int CALIB_SAMPLES = 500;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    long gyro_sum[3] = {0, 0, 0};
    long accel_sum[3] = {0, 0, 0};

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        MPU6050_Read_All(hi2c, accel_raw, gyro_raw);
        accel_sum[0] += accel_raw[0];
        accel_sum[1] += accel_raw[1];
        accel_sum[2] += accel_raw[2];
        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        osDelay(2); // 短暂延时
    }

    // 计算平均值
    for (int i = 0; i < 3; i++)
    {
        gyro_bias[i] = (float)gyro_sum[i] / CALIB_SAMPLES;
        accel_bias[i] = (float)accel_sum[i] / CALIB_SAMPLES;
    }

    // Z 轴加速度的偏移需要特殊处理，减去 1g (16384)
    accel_bias[2] -= ACCEL_SENSITIVITY;

    printf("IMU calibration finished.\r\n");
    printf("Gyro Bias (x100): x=%ld, y=%ld, z=%ld\r\n", (long)(gyro_bias[0] * 100), (long)(gyro_bias[1] * 100), (long)(gyro_bias[2] * 100));
    printf("Accel Bias (x100): x=%ld, y=%ld, z=%ld\r\n", (long)(accel_bias[0] * 100), (long)(accel_bias[1] * 100), (long)(accel_bias[2] * 100));
}

/**
 * @brief 更新并处理 IMU 数据
 * @param hi2c: I2C 句柄
 * @param data: 指向用于存储结果的 IMU_Data_t 结构体
 * @param dt:   两次测量之间的时间差 (秒)
 */
void IMU_Process_Update(I2C_HandleTypeDef *hi2c, IMU_Data_t *data, float dt)
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    // 1. 读取原始数据
    MPU6050_Read_All(hi2c, accel_raw, gyro_raw);

    // 2. 移除零偏并转换单位
    for (int i = 0; i < 3; i++)
    {
        data->Gyro[i] = ((float)gyro_raw[i] - gyro_bias[i]) / GYRO_SENSITIVITY;
        data->Accel[i] = ((float)accel_raw[i] - accel_bias[i]) / ACCEL_SENSITIVITY;
    }

    // 3. 姿态解算
    float ax = data->Accel[0], ay = data->Accel[1], az = data->Accel[2];
    const float rad_to_deg = 57.295779513f;

    // 3.1. 加速度解算角度 (静态参考)
    // 根据 goal.md 中的公式
    float pitch_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * rad_to_deg;
    float roll_acc  = atan2f(ay, sqrtf(ax * ax + az * az)) * rad_to_deg;

    // 3.2. 互补滤波
    const float alpha = 0.98f;
    // Pitch: 绕 Y 轴旋转, 使用 Gyro[1]
    data->Pitch = alpha * (data->Pitch + data->Gyro[1] * dt) + (1.0f - alpha) * pitch_acc;
    // Roll: 绕 X 轴旋转, 使用 Gyro[0]
    data->Roll  = alpha * (data->Roll + data->Gyro[0] * dt) + (1.0f - alpha) * roll_acc;

    // 3.3. 航向角 (Yaw) - 仅陀螺仪积分
    // 注意：没有磁力计校准，Yaw 会随时间漂移
    // Yaw: 绕 Z 轴旋转, 使用 Gyro[2]
    data->Yaw += data->Gyro[2] * dt;
}