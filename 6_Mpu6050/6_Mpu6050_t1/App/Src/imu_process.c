#include "imu_process.h"
#include "mpu6050.h"
#include <math.h>

#define CALIB_SAMPLES 500

// Mahony AHRS 算法参数
#define twoKpDef (2.0f * 0.5f) // 2 * 比例增益
#define twoKiDef (2.0f * 0.0f) // 2 * 积分增益

// Mahony 算法全局变量
static volatile float twoKp = twoKpDef;
static volatile float twoKi = twoKiDef;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};

// --- Mahony AHRS 核心更新函数 ---
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt, volatile float* q) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果加速度计的读数无效（例如，自由落体），则不使用
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 标准化加速度计测量值
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向
        halfvx = q[1] * q[3] - q[0] * q[2];
        halfvy = q[0] * q[1] + q[2] * q[3];
        halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // 计算误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分误差
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用比例增益
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 积分陀螺仪速率并更新四元数
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // 标准化四元数
    recipNorm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}


void IMU_Process_Init(I2C_HandleTypeDef *hi2c, IMU_Data_t *data)
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

    // 初始化四元数
    data->q[0] = 1.0f;
    data->q[1] = 0.0f;
    data->q[2] = 0.0f;
    data->q[3] = 0.0f;
}

void IMU_Process_Update(I2C_HandleTypeDef *hi2c, IMU_Data_t *data, IMU_Output_t *output, float dt)
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    // 1. 读取原始数据
    MPU6050_Read_All(hi2c, accel_raw, gyro_raw);

    // 2. 校准并转换为物理单位 (g, °/s)
    float accel_g[3], gyro_dps[3];
    accel_g[0] = (accel_raw[0] - accel_bias[0]) / ACCEL_SENSITIVITY;
    accel_g[1] = (accel_raw[1] - accel_bias[1]) / ACCEL_SENSITIVITY;
    accel_g[2] = (accel_raw[2] - accel_bias[2]) / ACCEL_SENSITIVITY;
    gyro_dps[0] = (gyro_raw[0] - gyro_bias[0]) / GYRO_SENSITIVITY;
    gyro_dps[1] = (gyro_raw[1] - gyro_bias[1]) / GYRO_SENSITIVITY;
    gyro_dps[2] = (gyro_raw[2] - gyro_bias[2]) / GYRO_SENSITIVITY;

    // 3. 姿态解算 (Mahony AHRS)
    // 将单位从 °/s 转换为 rad/s
    float gyro_rad[3];
    gyro_rad[0] = gyro_dps[0] * DEG_TO_RAD;
    gyro_rad[1] = gyro_dps[1] * DEG_TO_RAD;
    gyro_rad[2] = gyro_dps[2] * DEG_TO_RAD;

    Mahony_Update(gyro_rad[0], gyro_rad[1], gyro_rad[2],
                  accel_g[0], accel_g[1], accel_g[2],
                  dt, data->q);

    // 4. 将四元数转换为欧拉角 (用于调试)
    data->Pitch = asinf(-2.0f * (data->q[1] * data->q[3] - data->q[0] * data->q[2])) * 57.29578f;
    data->Roll = atan2f(2.0f * (data->q[0] * data->q[1] + data->q[2] * data->q[3]),
                       data->q[0] * data->q[0] - data->q[1] * data->q[1] - data->q[2] * data->q[2] + data->q[3] * data->q[3]) * 57.29578f;
    data->Yaw = atan2f(2.0f * (data->q[1] * data->q[2] + data->q[0] * data->q[3]),
                      data->q[0] * data->q[0] + data->q[1] * data->q[1] - data->q[2] * data->q[2] - data->q[3] * data->q[3]) * 57.29578f;


    // 5. 填充内部数据结构 (用于调试或特定应用)
    data->Accel[0] = accel_g[0];
    data->Accel[1] = accel_g[1];
    data->Accel[2] = accel_g[2];
    data->Gyro[0] = gyro_dps[0];
    data->Gyro[1] = gyro_dps[1];
    data->Gyro[2] = gyro_dps[2];

    // --- 工程化输出 ---

    // 6. 坐标系统一 (TASK 4.1)
    // !! 关键 !!: 这里的变换关系取决于IMU在机器人上的实际安装方向
    // 示例假设: 机器人前进(X) = IMU的Y轴, 机器人左侧(Y) = -IMU的X轴, 机器人上方(Z) = IMU的Z轴
    float final_accel_x = accel_g[1];
    float final_accel_y = -accel_g[0];
    float final_accel_z = accel_g[2];

    float final_gyro_x = gyro_dps[1];
    float final_gyro_y = -gyro_dps[0];
    float final_gyro_z = gyro_dps[2];

    // 根据轴变换，欧拉角也需要相应地交换和变号
    float final_roll  = -data->Pitch; // 原Pitch对应IMU的X轴旋转，现变为-Roll(绕机器人Y轴)
    float final_pitch = data->Roll;   // 原Roll对应IMU的Y轴旋转，现变为Pitch(绕机器人X轴)
    float final_yaw   = data->Yaw;    // Yaw通常不受影响, 但需根据实际情况确认


    // 7. 填充标准化输出结构 (TASK 4.2)
    if (output)
    {
        // 转换单位为 ROS 标准 (m/s^2, rad/s, rad)
        output->linear_acceleration[0] = final_accel_x * GRAVITY_MSS;
        output->linear_acceleration[1] = final_accel_y * GRAVITY_MSS;
        output->linear_acceleration[2] = final_accel_z * GRAVITY_MSS;

        output->angular_velocity[0] = final_gyro_x * DEG_TO_RAD;
        output->angular_velocity[1] = final_gyro_y * DEG_TO_RAD;
        output->angular_velocity[2] = final_gyro_z * DEG_TO_RAD;

        // 填充四元数 (注意: ROS中w在后, 此处我们遵循w,x,y,z顺序)
        // 坐标变换不影响四元数本身，但解释它时需要对应新的坐标系
        // 假设Mahony输出的q就是相对于变换后坐标系的，如果不是，则q也需要变换
        // 简单处理：直接输出原始q，在ROS端用tf变换
        output->orientation[0] = data->q[0]; // w
        output->orientation[1] = data->q[1]; // x
        output->orientation[2] = data->q[2]; // y
        output->orientation[3] = data->q[3]; // z

        output->attitude[0] = final_pitch * DEG_TO_RAD;
        output->attitude[1] = final_roll * DEG_TO_RAD;
        output->attitude[2] = final_yaw * DEG_TO_RAD;

        output->timestamp = HAL_GetTick();
    }
}