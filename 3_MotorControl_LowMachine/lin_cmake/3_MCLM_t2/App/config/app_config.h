#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* =================================================================================
 *   1. 电机驱动选择 (Motor Driver Selection)
 * ================================================================================= */
#define MOTOR_DRIVER_TB6612 1
#define MOTOR_DRIVER_AT8236 2
#define MOTOR_DRIVER_IBT4   3

/*
 *  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *  <<<                                                                       >>>
 *  <<<   请在此处选择当前使用的电机驱动型号                                    >>>
 *  <<<   (ONLY CHANGE THE LINE BELOW TO SELECT THE ACTIVE MOTOR DRIVER)        >>>
 *  <<<                                                                       >>>
 *  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 */
#define ACTIVE_MOTOR_DRIVER MOTOR_DRIVER_TB6612
// #define ACTIVE_MOTOR_DRIVER MOTOR_DRIVER_AT8236
// #define ACTIVE_MOTOR_DRIVER MOTOR_DRIVER_IBT4


/* =================================================================================
 *   3. 电机1: 共享控制参数 (Motor 1: Shared Control Parameters)
 * ================================================================================= */

/* ------------------- 速度与PWM范围定义 (Speed & PWM Normalization) ------------------- */
#define SPEED_TICKS_MAX     90    // 单个控制周期内编码器的最大计数值 (用于速度计算)
#define ENCODER_FILTER_ALPHA    0.1f  // 编码器IIR滤波系数，越小越平滑，响应越慢
#define SPEED_LOGIC_MAX     100   // 统一的逻辑速度最大值 (例如 0-100)
#define MOTOR_CMD_DEFAULT_SPEED  50.0f  // CMD_FORWARD / CMD_REVERSE 的默认逻辑速度 (占满量程 50%)
#define PWM_MAX             7200    // PWM最大值, 对应定时器的ARR寄存器值 (TIM3 Period=100-1=99), 代表100%占空比

/* =================================================================================
 *   3. 电机1: 驱动器特定配置 (Motor 1: Driver-Specific Configuration)
 * ================================================================================= */

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
    /* --- TB6612 硬件配置 --- */
    #define MOTOR1_TIM_HANDLE           &htim1
    #define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1 // PWM信号引脚
    #define MOTOR1_IN1_PORT             GPIOB         // 方向控制引脚1
    #define MOTOR1_IN1_PIN              GPIO_PIN_0
    #define MOTOR1_IN2_PORT             GPIOB         // 方向控制引脚2
    #define MOTOR1_IN2_PIN              GPIO_PIN_1
    #define MOTOR1_EN_PORT              NULL         // 使能引脚
    #define MOTOR1_EN_PIN               0
    #define MOTOR1_DEFAULT_STOP_MODE    TB6612_MOTOR_STOP_BRAKE // 默认停止模式

    /* ------------------- 电机控制限制 (Motor Control Limits) ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX        // 应用于电机的最大PWM输出值
    #define MOTOR1_MIN_PWM_OUTPUT       0              // 应用于电机的最小PWM输出值
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX// 最大逻辑速度 (用于速度映射)
    #define MOTOR1_DEAD_ZONE            10             // PWM死区, 低于此值的PWM输出将被忽略, 防止电机在零速时抖动

    /* ------------------- PID控制器参数 (PID Controller Constants) ------------------- */
    #define MOTOR1_PID_KP                0.4584f
    // #define MOTOR1_PID_KP                0.01362f   // 过小，Kp贡献不足
    // #define MOTOR1_PID_KP                2.5f          // 提高比例项，让Kp主导动态响应
    #define MOTOR1_PID_KI                17.66f
    // #define MOTOR1_PID_KI                10.01f      // 过大，积分快速饱和
    // #define MOTOR1_PID_KI                0.5f          // 降低积分，仅用于消除稳态误差
    // #define MOTOR1_PID_KD                0.002976f
    #define MOTOR1_PID_KD                0.0025f
    // integral_limit 的含义: 积分累积量上限（单位与误差×时间相同）
    // 积分项最大贡献 = Ki * integral_limit = 17.66 * 5.66 ≈ 100 = OUTPUT_LIMIT, 防止 windup 同时保留稳态消差能力
    #define MOTOR1_PID_INTEGRAL_LIMIT    5.66f         // Ki * INTEGRAL_LIMIT ≈ OUTPUT_LIMIT (100/17.66)
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f        // PID输出限制 (通常等于SPEED_LOGIC_MAX)
    #define MOTOR1_PID_TS                0.01f         // PID采样时间 (秒), 此处为10ms
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f    // 微分项的低通滤波器系数 (0.0 to 1.0)

    /* ── 电机2 (动力电机) TB6612 配置 ── */
    #define MOTOR2_TIM_HANDLE           &htim1
    #define MOTOR2_TIM_CHANNEL          TIM_CHANNEL_2   // TIM1_CH2（确认 CubeMX 已使能）
    #define MOTOR2_IN1_PORT             GPIOB
    #define MOTOR2_IN1_PIN              GPIO_PIN_12
    #define MOTOR2_IN2_PORT             GPIOB
    #define MOTOR2_IN2_PIN              GPIO_PIN_13
    #define MOTOR2_EN_PORT              NULL            // 无独立使能引脚时填 NULL
    #define MOTOR2_EN_PIN               0
    #define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR2_DEAD_ZONE            10

    /* 电机2 PID参数（初始与电机1相同，后续独立整定） */
    #define MOTOR2_PID_KP               0.4584f
    #define MOTOR2_PID_KI               17.66f
    #define MOTOR2_PID_KD               0.0025f
    #define MOTOR2_PID_INTEGRAL_LIMIT   5.66f
    #define MOTOR2_PID_OUTPUT_LIMIT     100.0f
    #define MOTOR2_PID_TS               0.01f
    #define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f

    /* 编码器定时器（CubeMX 需同步配置 TIM3 为 Encoder 模式） */
    #define MOTOR1_ENCODER_TIM          &htim2
    #define MOTOR2_ENCODER_TIM          &htim3


#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)
    /* --- AT8236 硬件配置 --- */
    #define MOTOR1_TIM_HANDLE           &htim3
    #define MOTOR1_PWM_CHANNEL1         TIM_CHANNEL_1 // IN1 的 PWM
    #define MOTOR1_PWM_CHANNEL2         TIM_CHANNEL_2 // IN2 的 PWM
    #define MOTOR1_DEFAULT_STOP_MODE    AT8236_STOP_BRAKE // 默认停止模式

    /* ------------------- 电机控制限制 (Motor Control Limits) ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX        // 应用于电机的最大PWM输出值
    #define MOTOR1_MIN_PWM_OUTPUT       0              // 应用于电机的最小PWM输出值
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX// 最大逻辑速度 (用于速度映射)
    #define MOTOR1_DEAD_ZONE            10             // PWM死区, 低于此值的PWM输出将被忽略, 防止电机在零速时抖动

    /* ------------------- PID控制器参数 (PID Controller Constants) ------------------- */
    #define MOTOR1_PID_KP                0.4584f
    #define MOTOR1_PID_KI                17.66f
    #define MOTOR1_PID_KD                0.002976f
    #define MOTOR1_PID_INTEGRAL_LIMIT    500.0f       // PID积分项限制
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f       // PID输出限制 (通常等于SPEED_LOGIC_MAX)
    #define MOTOR1_PID_TS                0.01f        // PID采样时间 (秒), 此处为10ms
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f   // 微分项的低通滤波器系数 (0.0 to 1.0)


#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_IBT4)
    /* --- IBT-4 硬件配置 (请根据实际连接修改) --- */
    #define MOTOR1_TIM_HANDLE           &htim3 1      // 示例: TIM3
    #define MOTOR1_PWM_CHANNEL1         TIM_CHANNEL_3 // 示例: TIM3_CH3
    #define MOTOR1_PWM_CHANNEL2         TIM_CHANNEL_4 // 示例: TIM3_CH4
    #define MOTOR1_DEFAULT_STOP_MODE    IBT4_STOP_BRAKE // 默认停止模式

    /* ------------------- 电机控制限制 (Motor Control Limits) ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX        // 应用于电机的最大PWM输出值
    #define MOTOR1_MIN_PWM_OUTPUT       0              // 应用于电机的最小PWM输出值
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX// 最大逻辑速度 (用于速度映射)
    #define MOTOR1_DEAD_ZONE            10             // PWM死区, 低于此值的PWM输出将被忽略, 防止电机在零速时抖动

    /* ------------------- PID控制器参数 (PID Controller Constants) ------------------- */
    #define MOTOR1_PID_KP                0.4584f
    #define MOTOR1_PID_KI                17.66f
    #define MOTOR1_PID_KD                0.002976f
    #define MOTOR1_PID_INTEGRAL_LIMIT    500.0f       // PID积分项限制
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f       // PID输出限制 (通常等于SPEED_LOGIC_MAX)
    #define MOTOR1_PID_TS                0.01f        // PID采样时间 (秒), 此处为10ms
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f   // 微分项的低通滤波器系数 (0.0 to 1.0)

#endif


/* =================================================================================
 *   4. 系统与通信配置 (System & Communication Configuration)
 * ================================================================================= */

/* ------------------- ACK消息缓冲区 ------------------- */
#define ACK_MSG_BUF_SIZE            128

/* ------------------- CAN总线配置 ------------------- */
/* CAN 协议 */
// #define CAN_MOTOR_TURN_CMD_STDID             0x125   // 方向电机控制指令的CAN ID
// #define CAN_MOTOR_TURN_CMD_STATUS_STDID      0x225   // 方向上层控制电机状态的CAN ID
// #define CAN_MOTOR_TURN_STATUS_STDID          0x325   // 方向电机状态反馈的CAN ID

// #define CAN_MOTOR_POWER_CMD_STDID            0x126   // 动力电机控制指令的CAN ID
// #define CAN_MOTOR_POWER_CMD_STATUS_STDID     0x226   // 动力上层控制电机状态的CAN ID
// #define CAN_MOTOR_POWER_STATUS_STDID         0x326   // 动力电机状态反馈的CAN ID

// 第二组
#define CAN_MOTOR_TURN_CMD_STDID             0x123   // 方向电机控制指令的CAN ID (第二组)
#define CAN_MOTOR_TURN_CMD_STATUS_STDID      0x223   // 方向上层控制电机状态的CAN ID (第二组)
#define CAN_MOTOR_TURN_STATUS_STDID          0x323   // 方向电机状态反馈的CAN ID (第二组)

#define CAN_MOTOR_POWER_CMD_STDID            0x124   // 动力电机控制指令的CAN ID (第二组)
#define CAN_MOTOR_POWER_CMD_STATUS_STDID     0x224   // 动力上层控制电机状态的CAN ID (第二组)
#define CAN_MOTOR_POWER_STATUS_STDID         0x324   // 动力电机状态反馈的CAN ID (第二组)

#define CAN_CMD_SET_SPEED_T2            0x11    // 新的设置速度命令 
#define CAN_CMD_REVERSE_BYTE            0x02    // 独立倒转命令字节（宏，避免与枚举 CMD_REVERSE=2 在 switch 中混用）
#define CAN_CMD_QUERY_STATUS            0x01    // 查询状态命令 (兼容旧协议)
#define CAN_CMD_LOG_START               0x04    // 开始发送实时电机数据
#define CAN_CMD_LOG_STOP                0x05    // 停止发送实时电机数据

// 全车停止
#define CAN_CMD_STOP_STDID              0x101   // 全车停止命令的CAN ID
// 全车转向命令
#define CAN_CMD_TURN_STDID              0x102   // 转向命令的CAN ID
// 全车动力命令
#define CAN_CMD_POWER_STDID             0x103   // 动力命令的CAN ID

/* CAN 硬件初始化 */
#define CAN_PRESCALER               4
#define CAN_MODE                    CAN_MODE_NORMAL
#define CAN_SYNC_JUMP_WIDTH         CAN_SJW_1TQ
#define CAN_TIME_SEG1               CAN_BS1_13TQ
#define CAN_TIME_SEG2               CAN_BS2_4TQ
#define CAN_TIME_TRIGGERED_MODE     DISABLE
#define CAN_AUTO_BUS_OFF            ENABLE
#define CAN_AUTO_WAKE_UP            DISABLE
#define CAN_AUTO_RETRANSMISSION     ENABLE
#define CAN_RECEIVE_FIFO_LOCKED     DISABLE
#define CAN_TRANSMIT_FIFO_PRIORITY  ENABLE

/* CAN 过滤器配置 */
#define CAN_FILTER_BANK             0
#define CAN_FILTER_MODE             CAN_FILTERMODE_IDMASK
#define CAN_FILTER_SCALE            CAN_FILTERSCALE_32BIT
#define CAN_FILTER_ID_HIGH          0x0000
#define CAN_FILTER_ID_LOW           0x0000
#define CAN_FILTER_MASK_ID_HIGH     0x0000
#define CAN_FILTER_MASK_ID_LOW      0x0000
#define CAN_FILTER_FIFO             CAN_RX_FIFO0
#define CAN_FILTER_ACTIVATION       ENABLE
#define CAN_SLAVE_START_FILTER_BANK 14

/* ================= CAN Protocol Definitions ================= */
// 定义CAN数据帧中不同数据字段的索引
#define CAN_DATA_INDEX_CMD      0   // 命令类型在数据帧中的索引
#define CAN_DATA_INDEX_SPEED    1   // 速度值在数据帧中的索引




#endif // __APP_CONFIG_H__