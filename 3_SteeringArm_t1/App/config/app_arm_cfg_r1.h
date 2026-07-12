#ifndef __APP_ARM_CFG_R1_H__
#define __APP_ARM_CFG_R1_H__

/* =================================================================================
 *   R1 腕部配置 — 双夹爪(平行10~15cm) + J0 DC电机(第三关节旋转)
 *
 *   硬件:
 *     TIM4_CH1 (PB6) → 夹爪左舵机
 *     TIM4_CH2 (PB7) → 夹爪右舵机
 *     TIM1_CH1 (PA8) → J0 IBT4 RPWM
 *     TIM1_CH2 (PA9) → J0 IBT4 LPWM (如需要独立引脚, 否则 PB12)
 *     TIM2 (PA0/PA1) → J0 霍尔编码器
 * ================================================================================= */

/* ── 夹爪 (实测标定) ── */
#define GRIPPER_PULSE_OPEN      5000    // 实测: 全开
#define GRIPPER_PULSE_CLOSE     2800    // 实测: 锁紧(再小有堵转风险)
#define GRIPPER_PULSE_MID       3000    // 实测: 刚接触物体

/* ── 夹爪舵机独立偏移校准 ── */
/* 机械安装公差导致左右舵机在相同脉宽下角度不同。
   设置偏移量使夹爪对称闭合: 正值=更靠近闭合方向 */
#define GRIPPER_LEFT_OFFSET     (-156)  // TIM4_CH1 (PB6) 左舵机偏移量
#define GRIPPER_RIGHT_OFFSET    (-48)   // TIM4_CH2 (PB7) 右舵机偏移量

/* ── J0 DC 电机 (IBT4) ── */
#define J0_PWM_MAX              7200    // TIM1 ARR (与 MCLM 一致)
#define J0_SPEED_LOGIC_MAX      100     // 逻辑速度最大值 (-100~100)
#define J0_ENCODER_PPR          12      // 霍尔编码器物理 PPR
#define J0_ENCODER_TICK_PER_REV (J0_ENCODER_PPR * 4)  // 48 ticks/电机轴圈

/* J0 PID 参数 (速度内环) */
#define J0_SPEED_KP             0.5f
#define J0_SPEED_KI             0.05f
#define J0_SPEED_KD             0.001f
#define J0_SPEED_INTEGRAL_LIMIT 20.0f
#define J0_SPEED_OUTPUT_LIMIT   100.0f
#define J0_SPEED_TS             0.01f    // 10ms 采样周期

/* J0 PID 参数 (位置外环，可选) */
#define J0_POSITION_KP          2.0f
#define J0_POSITION_OUTPUT_LIMIT   50.0f   // 最大输出速度 50%
#define J0_POSITION_DEADBAND    0.5f        // 到位判定死区 (°)


#endif // __APP_ARM_CFG_R1_H__
