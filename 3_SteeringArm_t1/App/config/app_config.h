#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* =================================================================================
 *   机械臂构型选择 (ARM_CFG_SET)
 *
 *   同一套固件支持两种机械臂，编译时通过此宏选择功能。
 *
 *   1 = R1 腕部: 双夹爪舵机 ×2 (平行间距10~15cm) + J0 DC电机 (基座旋转)
 *        - TIM4_CH1 → 夹爪左 (PB6)
 *        - TIM4_CH2 → 夹爪右 (PB7)
 *        - TIM4_CH3/CH4 → 保留
 *        - joint_id=3 → 双夹爪同步开合
 *        - J0 DC: TIM1_CH1 + TB6612 + TIM2编码器
 *
 *   2 = R2 舵机臂: J1 大臂 + J2 小臂 + J3 夹爪 (J0 基座旋转已锁死)
 *        - TIM4_CH1 → J1 大臂舵机 (PB6)
 *        - TIM4_CH2 → J2 小臂舵机 (PB7)
 *        - TIM4_CH3/CH4 → J3 夹爪 (PB8/PB9)
 *        - J0 基座旋转 → ⛔ 机械锁死
 *
 *   选择方式: 改下面这个数字后重新编译烧录
 *     cmake -S . -B build && cmake --build build --target flash
 * ================================================================================= */
#define ARM_CFG_SET  2


/* =================================================================================
 *   CAN 通信参数
 * ================================================================================= */
#define CAN_ARM_CMD_STDID           0x130   // 关节角度控制
#define CAN_ARM_VEL_STDID           0x131   // 关节速度设置
#define CAN_ARM_QUERY_STDID         0x230   // 状态查询
#define CAN_ARM_STATUS_STDID        0x330   // 状态上报
#define CAN_ARM_CONFIG_STDID        0x430   // 参数配置

#define CAN_DATA_INDEX_CMD          0       // 命令字节索引
#define CAN_DATA_INDEX_JOINT_ID     1       // 关节ID索引
#define CAN_DATA_INDEX_VALUE_L      2       // 目标值低字节
#define CAN_DATA_INDEX_VALUE_H      3       // 目标值高字节


/* =================================================================================
 *   舵机参数
 * ================================================================================= */
#define SERVO_PULSE_MIN             1000    // 500μs → -150°
#define SERVO_PULSE_MID             3000    // 1500μs → 0° (中位)
#define SERVO_PULSE_MAX             5000    // 2500μs → +150°

/* 夹爪脉宽范围 (实测标定) */
#define GRIPPER_OPEN                1000    // 全开
#define GRIPPER_CLOSE               5000    // 全闭


/* =================================================================================
 *   按构型加载配置
 * ================================================================================= */
#if ARM_CFG_SET == 1
  #include "app_arm_cfg_r1.h"
#elif ARM_CFG_SET == 2
  #include "app_arm_cfg_r2.h"
#else
  #error "ARM_CFG_SET must be 1 (R1) or 2 (R2)"
#endif


#endif // __APP_CONFIG_H__
