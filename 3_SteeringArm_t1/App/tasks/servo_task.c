/* ============================================================
 *  servo_task.c — 舵机平滑插值控制 (20ms)
 *
 *  ARM_CFG_SET=1 (R1): TIM4_CH1/CH2 → 双夹爪同步, 无 MT6701
 *  ARM_CFG_SET=2 (R2): TIM4_CH1=J1, CH2=J2, CH3/CH4=J3(Gripper)
 *                     + MT6701 反馈
 *
 *  串口输出格式: J1/J2 角度(×10), J3 CCR, 各通道 PWM CCR
 * ============================================================ */
#include <stdio.h>
#include <math.h>
#include "app_config.h"
#include "app_globals.h"
#include "servo.h"
#include "mt6701.h"
#include "tim.h"
#include "cmsis_os.h"

extern ArmState_t g_arm_state;
extern volatile uint8_t g_servo_active;
extern volatile uint8_t g_system_locked;

void Servo_Task_Impl(void *argument)
{
    Servo_Init();

#if ARM_CFG_SET == 2
    /* ── R2: 舵机臂模式 ── */
    MT6701_Init();
    g_arm_state.j1_speed_dps = DEFAULT_SPEED_DPS;
    g_arm_state.j2_speed_dps = DEFAULT_SPEED_DPS;
    g_arm_state.j1_offset = J1_ANGLE_OFFSET;
    g_arm_state.j2_offset = J2_ANGLE_OFFSET;
    g_arm_state.gripper_offset = GRIPPER_PULSE_OFFSET;
    g_arm_state.gripper_target = 3200;  /* 夹爪初始中位(开口刚可进物)，防零值 clamp 到极限堵转 */
    printf("Servo_Task (R2 arm), speed=%.0f°/s", DEFAULT_SPEED_DPS);
    if (g_arm_state.j1_offset != 0.0f || g_arm_state.j2_offset != 0.0f || g_arm_state.gripper_offset != 0)
        printf(" offset: J1=%+.1f° J2=%+.1f° GRIP=%+d",
               (double)g_arm_state.j1_offset, (double)g_arm_state.j2_offset, g_arm_state.gripper_offset);
    printf("\r\n");
#else
    /* ── R1: 双夹爪模式 ── */
    g_arm_state.gripper_target = GRIPPER_PULSE_MID;
    printf("Servo_Task (R1 dual gripper)\r\n");
#endif

    for (;;)
    {
#if ARM_CFG_SET == 2
        /* ===== R2: 读 MT6701 关节角度 ===== */
        g_arm_state.j1_raw = MT6701_ReadRaw(J1_CS_GPIO_Port, J1_CS_Pin);
        g_arm_state.j2_raw = MT6701_ReadRaw(J2_CS_GPIO_Port, J2_CS_Pin);
#endif

        /* ===== 舵机输出 ===== */
#if ARM_CFG_SET == 2
        /* ── R2: J1/J2 + 夹爪 ── */
        if (g_servo_active && !g_system_locked) {
            /* 首次激活：启动全部 PWM 通道，直接到目标位置，不经过 0° */
            static uint8_t activated = 0;
            if (!activated) {
                Servo_StartAll();
                g_arm_state.j1_current = g_arm_state.j1_target;
                g_arm_state.j2_current = g_arm_state.j2_target;
                activated = 1;
                printf("Servo activated → J1=%.1f° J2=%.1f°\r\n",
                       (double)g_arm_state.j1_current, (double)g_arm_state.j2_current);
            }

            float diff1 = g_arm_state.j1_target - g_arm_state.j1_current;
            float step1 = g_arm_state.j1_speed_dps * 0.02f;
            if (step1 > fabsf(diff1)) step1 = fabsf(diff1);
            g_arm_state.j1_current += (diff1 > 0.0f) ? step1 : -step1;

            float diff2 = g_arm_state.j2_target - g_arm_state.j2_current;
            float step2 = g_arm_state.j2_speed_dps * 0.02f;
            if (step2 > fabsf(diff2)) step2 = fabsf(diff2);
            g_arm_state.j2_current += (diff2 > 0.0f) ? step2 : -step2;

            Servo_SetAngle(&htim4, TIM_CHANNEL_1,
                           g_arm_state.j1_current + g_arm_state.j1_offset);
            Servo_SetAngle(&htim4, TIM_CHANNEL_2,
                           g_arm_state.j2_current + g_arm_state.j2_offset);

            /* J3 夹爪 (带补偿) */
            {
                int16_t grip = (int16_t)g_arm_state.gripper_target
                             + g_arm_state.gripper_offset;
                if (grip < 1000) grip = 1000;
                if (grip > 5000) grip = 5000;
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)grip);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)grip);
            }
        }

        int16_t j1_deg_x10 = MT6701_RawToAngleX10(g_arm_state.j1_raw, J1_ZERO_RAW);
        int16_t j2_deg_x10 = MT6701_RawToAngleX10(g_arm_state.j2_raw, J2_ZERO_RAW);
        uint16_t pwm1 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
        uint16_t pwm2 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
        uint16_t pwm3 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
        printf("J1=%+5d° J2=%+5d° J3(G)=%4u | PWM: CH1=%4u CH2=%4u CH3=%4u\r\n",
               j1_deg_x10, j2_deg_x10, g_arm_state.gripper_target,
               pwm1, pwm2, pwm3);
#else
        /* ── R1: 双夹爪 (独立控制 + 偏移 + g_servo_active 锁定) ── */
        if (g_servo_active) {
            static uint8_t activated = 0;
            if (!activated) {
                Servo_StartAll();
                activated = 1;
                printf("Servo activated (R1)\r\n");
            }
            uint16_t ch1 = g_arm_state.ch1_target ? g_arm_state.ch1_target
                          : (g_arm_state.gripper_target + GRIPPER_LEFT_OFFSET);
            uint16_t ch2 = g_arm_state.ch2_target ? g_arm_state.ch2_target
                          : (g_arm_state.gripper_target + GRIPPER_RIGHT_OFFSET);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ch1);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ch2);
        }
        /* 降低 printf 频率, 避免淹没 CAN_Rx 输出 */
        static uint32_t _p = 0;
        if (++_p % 10 == 0)
            printf("GRIPPER=%u\r\n", g_arm_state.gripper_target);
#endif

        osDelay(20);
    }
}
