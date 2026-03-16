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
#define SPEED_TICKS_MAX     80    // 单个控制周期内编码器的最大计数值 (用于速度计算)
#define SPEED_LOGIC_MAX     100   // 统一的逻辑速度最大值 (例如 0-100)
#define PWM_MAX             999   // PWM最大值, 对应定时器的ARR寄存器值, 代表100%占空比

/* =================================================================================
 *   3. 电机1: 驱动器特定配置 (Motor 1: Driver-Specific Configuration)
 * ================================================================================= */

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
    /* --- TB6612 硬件配置 --- */
    #define MOTOR1_TIM_HANDLE           &htim3
    #define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1 // PWM信号引脚
    #define MOTOR1_IN1_PORT             GPIOB         // 方向控制引脚1
    #define MOTOR1_IN1_PIN              GPIO_PIN_0
    #define MOTOR1_IN2_PORT             GPIOB         // 方向控制引脚2
    #define MOTOR1_IN2_PIN              GPIO_PIN_1
    #define MOTOR1_EN_PORT              GPIOB         // 使能引脚
    #define MOTOR1_EN_PIN               GPIO_PIN_10
    #define MOTOR1_DEFAULT_STOP_MODE    TB6612_MOTOR_STOP_BRAKE // 默认停止模式

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
    #define MOTOR1_TIM_HANDLE           &htim3        // 示例: TIM3
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
#define CAN_MOTOR_CMD_STDID         0x123  // 电机控制指令的CAN ID

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