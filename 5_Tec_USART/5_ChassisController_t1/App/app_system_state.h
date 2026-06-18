#ifndef APP_SYSTEM_STATE_H
#define APP_SYSTEM_STATE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ===================== IMU State ===================== */
typedef struct {
    float roll;          // 横滚角 (deg)
    float pitch;         // 俯仰角 (deg)
    float yaw;           // 偏航角 (deg)
    float quat[4];       // 四元数 (w, x, y, z)
    float gyro[3];       // 角速度 (rad/s)
    float accel[3];      // 线加速度 (m/s^2)
    uint32_t timestamp;  // 最后更新时间戳 (ms)
} IMU_State_t;

/* ===================== Motor State ===================== */

#define MOTOR_FLAG_STALL      0x01U  // 堵转标志
#define MOTOR_FLAG_SATURATED  0x02U  // PWM 饱和标志
#define MOTOR_MAX_COUNT       8      // 最多支持 8 个电机

typedef struct {
    int16_t current_speed;     // 当前逻辑速度 (-100..100)
    int16_t target_speed;      // 目标逻辑速度
    int16_t pwm_output;        // PWM 输出值
    uint8_t flags;             // 状态标志 (MOTOR_FLAG_*)
    uint32_t last_update_tick; // 最后更新时刻 (ms)
} Motor_State_t;

/* ===================== System Flags ===================== */
typedef struct {
    uint8_t  mode;         // 系统模式
    uint8_t  estop;        // 紧急停止标志 (1=已触发)
    uint32_t uptime_ms;    // 系统运行时间 (ms)
} System_Flag_t;

/* ===================== System State ===================== */
typedef struct {
    IMU_State_t    imu;
    Motor_State_t  motor[MOTOR_MAX_COUNT];
    System_Flag_t  flag;
} System_State_t;

/* ===================== Global Instance ===================== */
extern System_State_t g_system_state;

#endif /* APP_SYSTEM_STATE_H */
