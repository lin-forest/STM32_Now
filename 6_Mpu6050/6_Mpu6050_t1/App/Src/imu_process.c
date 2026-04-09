#include "imu_process.h"
#include "mpu6050.h"
#include <math.h>

#define CALIB_SAMPLES 500
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};

void IMU_Process_Init(I2C_HandleTypeDef *hi2c)
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    long accel_sum[3] = {0, 0, 0};
    long gyro_sum[3] = {0, 0, 0};

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        MPU6050_Read_All(hi2c, accel_raw, gyro_raw);
        accel_sum[0] += accel_raw[0];
        accel_sum[1] += accel_raw[1];
        accel_sum[2] += accel_raw[2];
        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        HAL_Delay(2);
    }

    accel_bias[0] = (float)accel_sum[0] / CALIB_SAMPLES;
    accel_bias[1] = (float)accel_sum[1] / CALIB_SAMPLES;
    accel_bias[2] = (float)accel_sum[2] / CALIB_SAMPLES - ACCEL_SENSITIVITY; // Z轴减去1g
    gyro_bias[0] = (float)gyro_sum[0] / CALIB_SAMPLES;
    gyro_bias[1] = (float)gyro_sum[1] / CALIB_SAMPLES;
    gyro_bias[2] = (float)gyro_sum[2] / CALIB_SAMPLES;
}

void IMU_Process_Update(I2C_HandleTypeDef *hi2c, IMU_Data_t *data, IMU_Output_t *output, float dt)
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    // 1. 读取原始数据
    MPU6050_Read_All(hi2c, accel_raw, gyro_raw);

    // 2. 校准并转换为物理单位 (g, °/s)
    float accel_cal[3], gyro_cal[3];
    accel_cal[0] = (accel_raw[0] - accel_bias[0]) / ACCEL_SENSITIVITY;
    accel_cal[1] = (accel_raw[1] - accel_bias[1]) / ACCEL_SENSITIVITY;
    accel_cal[2] = (accel_raw[2] - accel_bias[2]) / ACCEL_SENSITIVITY;
    gyro_cal[0] = (gyro_raw[0] - gyro_bias[0]) / GYRO_SENSITIVITY;
    gyro_cal[1] = (gyro_raw[1] - gyro_bias[1]) / GYRO_SENSITIVITY;
    gyro_cal[2] = (gyro_raw[2] - gyro_bias[2]) / GYRO_SENSITIVITY;

    // 3. 姿态解算 (互补滤波)
    float pitch_acc = atan2f(accel_cal[0], sqrtf(accel_cal[1] * accel_cal[1] + accel_cal[2] * accel_cal[2])) * 57.29578f;
    float roll_acc = atan2f(accel_cal[1], sqrtf(accel_cal[0] * accel_cal[0] + accel_cal[2] * accel_cal[2])) * 57.29578f;

    data->Pitch = COMPLEMENTARY_FILTER_ALPHA * (data->Pitch + gyro_cal[1] * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * pitch_acc;
    data->Roll = COMPLEMENTARY_FILTER_ALPHA * (data->Roll - gyro_cal[0] * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * roll_acc; // 注意 gx 积分方向
    data->Yaw += gyro_cal[2] * dt;

    // 4. 填充内部数据结构 (用于调试或特定应用)
    data->Accel[0] = accel_cal[0];
    data->Accel[1] = accel_cal[1];
    data->Accel[2] = accel_cal[2];
    data->Gyro[0] = gyro_cal[0];
    data->Gyro[1] = gyro_cal[1];
    data->Gyro[2] = gyro_cal[2];

    // --- 工程化输出 ---

    // 5. 坐标系统一 (TASK 4.1)
    // !! 关键 !!: 这里的变换关系取决于IMU在机器人上的实际安装方向
    // 示例假设: 机器人前进(X) = IMU的Y轴, 机器人左侧(Y) = -IMU的X轴, 机器人上方(Z) = IMU的Z轴
    float final_accel_x = accel_cal[1];
    float final_accel_y = -accel_cal[0];
    float final_accel_z = accel_cal[2];

    float final_gyro_x = gyro_cal[1];
    float final_gyro_y = -gyro_cal[0];
    float final_gyro_z = gyro_cal[2];

    float final_pitch = data->Roll;  // 注意Roll和Pitch也根据轴向变换
    float final_roll = -data->Pitch;
    float final_yaw = data->Yaw; // Yaw通常不受影响, 但需根据实际情况确认


    // 6. 填充标准化输出结构 (TASK 4.2)
    if (output)
    {
        // 转换单位为 ROS 标准 (m/s^2, rad/s, rad)
        output->linear_acceleration[0] = final_accel_x * GRAVITY_MSS;
        output->linear_acceleration[1] = final_accel_y * GRAVITY_MSS;
        output->linear_acceleration[2] = final_accel_z * GRAVITY_MSS;

        output->angular_velocity[0] = final_gyro_x * DEG_TO_RAD;
        output->angular_velocity[1] = final_gyro_y * DEG_TO_RAD;
        output->angular_velocity[2] = final_gyro_z * DEG_TO_RAD;

        output->attitude[0] = final_pitch * DEG_TO_RAD;
        output->attitude[1] = final_roll * DEG_TO_RAD;
        output->attitude[2] = final_yaw * DEG_TO_RAD;

        output->timestamp = HAL_GetTick();
    }
}