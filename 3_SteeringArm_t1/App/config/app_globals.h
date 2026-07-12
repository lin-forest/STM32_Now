/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_globals.h
  * @brief          : 全局类型定义与 extern 声明
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#include <stdint.h>
#include "cmsis_os2.h"

/* ================= CAN 消息类型 ================= */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} App_CAN_Message_t;

extern osMessageQueueId_t canRxQueueHandle;

/* ================= 机械臂全局状态 ================= */
typedef struct {
    /* J1 舵机 */
    float   j1_target;          // 目标角度 (°)
    float   j1_current;         // 当前角度 (°)
    float   j1_offset;          // 角度补偿 (°), 弥补机械安装误差
    uint16_t j1_raw;            // MT6701 原始值 0~16383

    /* J2 舵机 */
    float   j2_target;
    float   j2_current;
    float   j2_offset;          // 角度补偿 (°)
    uint16_t j2_raw;

    /* 舵机运动速度 */
    float   j1_speed_dps;        // 角速度 (°/s), 如 180 = 180°/s
    float   j2_speed_dps;

    /* J3 夹爪 (单舵机, R1/R2 共用) */
    uint16_t gripper_target;    // 目标脉宽 1000~5000
    uint16_t ch1_target;        // R1 左舵机独立目标 (0=跟随 gripper_target)
    uint16_t ch2_target;        // R1 右舵机独立目标 (0=跟随 gripper_target)
    int16_t gripper_offset;     // J3 脉宽偏移, R1/R2 共用

    /* 预留 J0 */
    int16_t j0_target;
    int16_t j0_current_speed;
    uint16_t j0_encoder_raw;    // TIM2 编码器原始值 (用于 PC 端位置控制)
} ArmState_t;

extern ArmState_t g_arm_state;
extern volatile uint8_t g_servo_active;   // 0=未激活(上电), 1=已收到首次命令
extern volatile uint8_t g_system_locked;  // 0=正常, 1=锁定(430#03), 覆盖 g_servo_active

#endif
