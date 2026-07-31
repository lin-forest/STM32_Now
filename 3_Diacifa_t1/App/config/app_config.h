#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* =================================================================================
 *   1. 气动系统硬件参数
 * ================================================================================= */
#define PWM_PERIOD          7200    // TIM2 ARR 值，对应 10kHz (72MHz/7200)
#define PWM_MAX_DUTY        100     // 占空比 0-100（百分比），对应 CCR = duty * 7200 / 100

/* =================================================================================
 *   2. IBT4 通道映射
 *      每块 IBT4 PWM1=正转调速，PWM2=反转（保持 0），共用一个马达
 * ================================================================================= */
#define PUMP1_ID            0       // IBT4 #1 PWM1(CH1/PA0)=正转 → 气泵1
#define PUMP2_ID            1       // IBT4 #2 PWM1(CH3/PA2)=正转 → 气泵2
#define VALVE1_ID           0       // GPIO PB0（后续 IBT4）
#define VALVE2_ID           1       // GPIO PB1（后续 IBT4）

/* =================================================================================
 *   3. CAN 通信参数
 * ================================================================================= */

/* CAN ID */
#define CAN_AIR_CMD_STDID       0x141   // RX：气动系统控制命令（避开 0x1XX 电机命令段）
#define CAN_AIR_STATUS_STDID    0x341   // TX：气动系统状态上报（避开 0x3XX 电机状态段）

/* CAN 命令帧字节索引（标准帧 8 字节） */
#define CMD_IDX_PUMP1_PWM       0       // [0] 气泵1 PWM (0-100, 0=停止)
#define CMD_IDX_PUMP2_PWM       1       // [1] 气泵2 PWM (0-100, 0=停止)
#define CMD_IDX_VALVE1          2       // [2] 电磁阀1 (0=失电, 1=得电)
#define CMD_IDX_VALVE2          3       // [3] 电磁阀2 (0=失电, 1=得电)
// [4-7] 保留

/* CAN 状态帧字节索引 */
#define STA_IDX_PUMP1_PWM       0       // [0] 气泵1 当前 PWM
#define STA_IDX_PUMP2_PWM       1       // [1] 气泵2 当前 PWM
#define STA_IDX_VALVE1          2       // [2] 电磁阀1 状态
#define STA_IDX_VALVE2          3       // [3] 电磁阀2 状态
#define STA_IDX_FLAGS           4       // [4] 故障标志
// [5-7] 保留

/* 故障标志位 */
#define AIR_FLAG_FAULT          0x01    // 通用故障
#define AIR_FLAG_PUMP1_ERR      (1<<1)  // 气泵1 异常
#define AIR_FLAG_PUMP2_ERR      (1<<2)  // 气泵2 异常
#define AIR_FLAG_12V_UNDER      (1<<3)  // 12V 欠压

/* CAN 硬件参数（MCLM 一致） */
#define CAN_PRESCALER               4
#define CAN_SYNC_JUMP_WIDTH         CAN_SJW_1TQ
#define CAN_TIME_SEG1               CAN_BS1_13TQ
#define CAN_TIME_SEG2               CAN_BS2_4TQ
#define CAN_AUTO_BUS_OFF            ENABLE
#define CAN_AUTO_RETRANSMISSION     ENABLE
#define CAN_TRANSMIT_FIFO_PRIORITY  ENABLE

/* CAN 过滤器 */
#define CAN_FILTER_BANK             0
#define CAN_FILTER_MODE             CAN_FILTERMODE_IDMASK
#define CAN_FILTER_SCALE            CAN_FILTERSCALE_32BIT
#define CAN_FILTER_FIFO             CAN_RX_FIFO0
#define CAN_FILTER_ACTIVATION       ENABLE

/* =================================================================================
 *   4. 任务参数
 * ================================================================================= */
#define AIR_CONTROL_PERIOD_MS   20      // 气动控制任务周期 (ms)
#define HEARTBEAT_PERIOD_MS     200     // LED 心跳周期 (ms)
#define CAN_REPORT_PERIOD_MS    500     // CAN 状态上报周期 (ms)

/* =================================================================================
 *   5. 队列定义
 * ================================================================================= */
#define CAN_CMD_QUEUE_LENGTH    16      // CAN 命令队列长度

#endif // __APP_CONFIG_H__
