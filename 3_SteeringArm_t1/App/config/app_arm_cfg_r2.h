#ifndef __APP_ARM_CFG_R2_H__
#define __APP_ARM_CFG_R2_H__

/* =================================================================================
 *   R2 舵机臂配置 — J1/J2 关节舵机 + 夹爪 (J0 已机械锁死)
 *
 *   硬件:
 *     TIM4_CH1 (PB6) → J1 舵机
 *     TIM4_CH2 (PB7) → J2 舵机
 *     TIM4_CH3 (PB8) → 夹爪舵机1
 *     TIM4_CH4 (PB9) → 夹爪舵机2
 *     SPI1 (PA5/PA6) → MT6701 J1/J2 角度反馈
 *     J0 (DC电机+IBT4) → ⛔ 机械锁死，不供电、不驱动
 * ================================================================================= */

/* ── 关节限位 ── */
#define J1_ANGLE_MIN            -150.0f
#define J1_ANGLE_MAX             150.0f
#define J2_ANGLE_MIN            -150.0f
#define J2_ANGLE_MAX             150.0f

/* ── 默认角速度 ── */
#define DEFAULT_SPEED_DPS       180.0f   // 默认 180°/s

/* ── 夹爪 (实测 2026-07-14: 5000=全开, 3200=中位开口, 3000=安全底线) ── */
#define GRIPPER_PULSE_OPEN      5000     // 实测: 全开
#define GRIPPER_PULSE_CLOSE     3000     // 实测: 中位安全底线(再小即夹紧)

/* ── MT6701 零位 (实测标定值) ── */
#define J1_ZERO_RAW             12041    // J1 0° 对应的 MT6701 raw
#define J2_ZERO_RAW             7637     // J2 0° 对应的 MT6701 raw

/* ── 舵机角度补偿 (机械安装误差, 标定后填入) ── */
#define J1_ANGLE_OFFSET         -20.0f     // J1 发 0° 时实际偏多少, 如 +2.5°
#define J2_ANGLE_OFFSET         0.0f     // J2 同理
#define GRIPPER_PULSE_OFFSET    0        // J3 脉宽偏移


#endif // __APP_ARM_CFG_R2_H__
