/* ============================================================
 *  arm_state_task.c — 机械臂状态 CAN 上报 (50ms / 20Hz)
 *
 *  CAN ID: 0x330 (ARM_STATUS)
 *  格式:
 *    [0-1] J1 raw_angle 或 夹爪脉宽 (uint16 LE)
 *    [2-3] J2 raw_angle 或 0 (uint16 LE)
 *    [4]   夹爪脉宽 / 200 (uint8)
 *    [5]   J0 current_speed (int8)
 *    [6]   flags: bit0=J1_IN_RANGE, bit1=J2_IN_RANGE
 *    [7]   0
 *
 *  参考: 3_MCLM_t2 command_task.c send_motor_status()
 * ============================================================ */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "app_globals.h"
#include <math.h>
#include "can.h"
#include "cmsis_os.h"

extern ArmState_t g_arm_state;
extern volatile uint8_t g_system_locked;

void Arm_State_Task_Impl(void *argument)
{
    CAN_TxHeaderTypeDef txHeader = {
        .StdId = CAN_ARM_STATUS_STDID,
        .DLC   = 8,
        .IDE   = CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
    };
    uint8_t txData[8];

    printf("Arm_State_Task started (ID=0x%03lX, 50ms)\r\n", (unsigned long)CAN_ARM_STATUS_STDID);

    for (;;)
    {
        memset(txData, 0, 8);

#if ARM_CFG_SET == 2
        /* R2: 上报关节角度 */
        txData[0] =  g_arm_state.j1_raw & 0xFF;
        txData[1] = (g_arm_state.j1_raw >> 8) & 0xFF;
        txData[2] =  g_arm_state.j2_raw & 0xFF;
        txData[3] = (g_arm_state.j2_raw >> 8) & 0xFF;
        txData[4] = (uint8_t)(g_arm_state.gripper_target / 200);
#else
        /* R1: 上报夹爪脉宽 + J0 速度 + J0 编码器位置 */
        txData[0] =  g_arm_state.gripper_target & 0xFF;
        txData[1] = (g_arm_state.gripper_target >> 8) & 0xFF;
        txData[2] =  g_arm_state.j0_encoder_raw & 0xFF;       /* 编码器低字节 */
        txData[3] = (g_arm_state.j0_encoder_raw >> 8) & 0xFF; /* 编码器高字节 */
        txData[4] = (uint8_t)(g_arm_state.gripper_target / 200);
#endif
        txData[5] = (int8_t)g_arm_state.j0_current_speed;

        /* flags */
        uint8_t flags = 0;
#if ARM_CFG_SET == 2
        float j1_err = g_arm_state.j1_target - g_arm_state.j1_current;
        float j2_err = g_arm_state.j2_target - g_arm_state.j2_current;
        if (fabsf(j1_err) < 1.0f) flags |= 0x01;
        if (fabsf(j2_err) < 1.0f) flags |= 0x02;
        if (g_system_locked)      flags |= 0x04;
#endif
        txData[6] = flags;

        uint32_t txMailbox;
        if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
        {
            printf("CAN_TX_FAIL: mailbox full or CAN not ready\r\n");
        }

        osDelay(50);  /* 50ms = 20Hz */
    }
}
