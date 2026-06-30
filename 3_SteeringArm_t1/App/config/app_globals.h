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
    uint16_t j1_raw;            // MT6701 原始值 0~16383

    /* J2 舵机 */
    float   j2_target;
    float   j2_current;
    uint16_t j2_raw;

    /* 舵机运动速度 */
    float   j1_speed_dps;        // 角速度 (°/s), 如 180 = 180°/s
    float   j2_speed_dps;

    /* 夹爪 */
    uint16_t gripper_target;    // 目标脉宽 1000~5000

    /* 预留 J0 */
    int16_t j0_target;
    int16_t j0_current_speed;
} ArmState_t;

extern ArmState_t g_arm_state;
extern volatile uint8_t g_servo_active;

#endif
