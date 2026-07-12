/* ============================================================
 *  can_rx_task.c — CAN 接收与命令分发
 *
 *  处理命令:
 *    0x130 — ARM_CMD (单关节 0x11 / 多关节 0x12 / 增量 0x21)
 *    0x430 — ARM_CONFIG (回中/设速/锁定/解锁/设限位)
 *    0x230 — ARM_QUERY (状态查询)
 *
 *  关节映射 (由 ARM_CFG_SET 决定):
 *    R1: joint 1/2/3 → 双夹爪同步
 *    R2: joint 1=J1, 2=J2, 3=Gripper
 * ============================================================ */
#include <stdio.h>
#include "app_config.h"
#include "app_globals.h"
#include "cmsis_os.h"

extern ArmState_t g_arm_state;
extern volatile uint8_t g_servo_active;    // 首次激活标记
extern volatile uint8_t g_system_locked;   // 锁定标记(430#03), 覆盖 g_servo_active

void CAN_Rx_Task_Impl(void *argument)
{
    App_CAN_Message_t msg;
    uint32_t idle_count = 0;

    for (;;)
    {
        if (osMessageQueueGet(canRxQueueHandle, &msg, NULL, 100) == osOK)
        {
            /* ======== 0x130 — ARM_CMD ======== */
            if (msg.id == CAN_ARM_CMD_STDID && msg.len >= 4)
            {
                uint8_t  cmd      = msg.data[0];
                uint8_t  joint_id = msg.data[1];
                int16_t  value    = (int16_t)(msg.data[2] | (msg.data[3] << 8));

                if (cmd == 0x12)
                {
                    /* ── 多关节模式 (MULTI) ── */
                    uint8_t mask = msg.data[1];
                    printf("ARM_MULTI: mask=0x%02X\r\n", mask);

                    if (mask & 0x01) {  /* J0 */
                        int8_t j0_spd = (int8_t)msg.data[7];
                        g_arm_state.j0_target = j0_spd;
                    }
                    if (mask & 0x02) {  /* J1 */
                        int16_t j1_val = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                        float ang = (float)j1_val / 10.0f;
#if ARM_CFG_SET == 2
                        if (ang < J1_ANGLE_MIN) ang = J1_ANGLE_MIN;
                        if (ang > J1_ANGLE_MAX) ang = J1_ANGLE_MAX;
#endif
                        g_arm_state.j1_target = ang;
                    }
                    if (mask & 0x04) {  /* J2 */
                        int16_t j2_val = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                        float ang = (float)j2_val / 10.0f;
#if ARM_CFG_SET == 2
                        if (ang < J2_ANGLE_MIN) ang = J2_ANGLE_MIN;
                        if (ang > J2_ANGLE_MAX) ang = J2_ANGLE_MAX;
#endif
                        g_arm_state.j2_target = ang;
                    }
                    if (mask & 0x08) {  /* Gripper */
                        /* uint8 0~200 → CCR 5000~3000 (全开→中位) */
                        uint8_t grip_pct = msg.data[6];
                        if (grip_pct > 200) grip_pct = 200;
                        g_arm_state.gripper_target = (uint16_t)(5000u - (uint32_t)grip_pct * 2000u / 200u);
                    }
                    if (!g_servo_active) g_servo_active = 1;  /* 首次激活 */
                }
                else if (cmd == 0x21)
                {
                    /* ── 增量模式 (INCREMENTAL) ── */
                    float delta = (float)value / 10.0f;
                    if (!g_servo_active) g_servo_active = 1;  /* 首次激活 */
                    switch (joint_id) {
#if ARM_CFG_SET == 2
                        case 1: {
                            float ang = g_arm_state.j1_target + delta;
                            if (ang < J1_ANGLE_MIN) ang = J1_ANGLE_MIN;
                            if (ang > J1_ANGLE_MAX) ang = J1_ANGLE_MAX;
                            g_arm_state.j1_target = ang;
                            break;
                        }
                        case 2: {
                            float ang = g_arm_state.j2_target + delta;
                            if (ang < J2_ANGLE_MIN) ang = J2_ANGLE_MIN;
                            if (ang > J2_ANGLE_MAX) ang = J2_ANGLE_MAX;
                            g_arm_state.j2_target = ang;
                            break;
                        }
                        case 3:  g_arm_state.gripper_target = (uint16_t)((int16_t)g_arm_state.gripper_target + value); break;
#else
                        case 1:
                        case 2:
                        case 3:  g_arm_state.gripper_target += (uint16_t)value; break;
#endif
                        default: break;
                    }
                    printf("ARM_INC: j%d += %d\r\n", joint_id, value);
                }
                else
                {
                    /* ── 绝对模式 (0x11 等) ── */
                    printf("ARM_CMD: j%d = %d\r\n", joint_id, value);
                    if (!g_servo_active) g_servo_active = 1;  /* 首次激活 */

                    switch (joint_id) {
                        case 0:  g_arm_state.j0_target = (int16_t)value; break;
#if ARM_CFG_SET == 2
                        case 1: {
                            float ang = (float)value / 10.0f;
                            if (ang < J1_ANGLE_MIN) ang = J1_ANGLE_MIN;
                            if (ang > J1_ANGLE_MAX) ang = J1_ANGLE_MAX;
                            g_arm_state.j1_target = ang;
                            break;
                        }
                        case 2: {
                            float ang = (float)value / 10.0f;
                            if (ang < J2_ANGLE_MIN) ang = J2_ANGLE_MIN;
                            if (ang > J2_ANGLE_MAX) ang = J2_ANGLE_MAX;
                            g_arm_state.j2_target = ang;
                            break;
                        }
                        case 3:  g_arm_state.gripper_target = (uint16_t)value; break;
#else
                        /* R1: joint 1=左舵机, 2=右舵机, 3=双爪同步 */
                        case 1:  g_arm_state.ch1_target = (uint16_t)value; break;
                        case 2:  g_arm_state.ch2_target = (uint16_t)value; break;
                        case 3:  g_arm_state.gripper_target = (uint16_t)value; break;
#endif
                        default: break;
                    }
                }
            }
            /* ======== 0x430 — ARM_CONFIG ======== */
            else if (msg.id == CAN_ARM_CONFIG_STDID && msg.len >= 1)
            {
                switch (msg.data[0])
                {
                    case 0x01:  /* 回中 */
                        if (!g_servo_active) g_servo_active = 1;  /* 首次激活 */
#if ARM_CFG_SET == 2
                        g_arm_state.j1_target = 0.0f;
                        g_arm_state.j2_target = 0.0f;
                        g_arm_state.gripper_target = 3200;  /* 夹爪中位(开口刚可进物) */
#else
                        g_arm_state.gripper_target = 3000;  /* 中位 */
                        g_arm_state.ch1_target = 0;         /* 退出独立模式 */
                        g_arm_state.ch2_target = 0;
#endif
                        printf("=== HOME ===\r\n");
                        break;

                    case 0x02:  /* 设置角速度 (°/s × 10) */
                        if (msg.len >= 3) {
                            int16_t spd = (int16_t)(msg.data[1] | (msg.data[2] << 8));
                            g_arm_state.j1_speed_dps = (float)spd / 10.0f;
                            g_arm_state.j2_speed_dps = (float)spd / 10.0f;
                            printf("Speed = %.0f°/s\r\n", g_arm_state.j1_speed_dps);
                        }
                        break;

                    case 0x03:  /* 锁定 */
                        g_system_locked = 1;
                        printf("=== LOCKED (关节指令被拦截) ===\r\n");
                        break;

                    case 0x04:  /* 解锁 */
                        g_system_locked = 0;
                        printf("=== UNLOCKED ===\r\n");
                        break;
                }
            }
            /* ======== 0x230 — ARM_QUERY ======== */
            else if (msg.id == CAN_ARM_QUERY_STDID)
            {
                printf("ARM_QUERY: type=%d\r\n", msg.data[0]);
            }
            /* ======== 其他 ======== */
            else
            {
                printf("CAN RX: ID=0x%03lX Len=%d ", msg.id, msg.len);
                for (int i = 0; i < msg.len; i++)
                    printf("%02X ", msg.data[i]);
                printf("\r\n");
            }
        }
        else {
            /* 无消息超时, 每秒打印一次表明任务存活 */
            if (++idle_count % 10 == 0)
                printf("[CAN_Rx alive]\r\n");
        }
    }
}
