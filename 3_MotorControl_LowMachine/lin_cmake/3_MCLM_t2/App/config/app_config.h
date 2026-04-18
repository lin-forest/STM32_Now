#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* --- 全局电机数量配置 --- */
#define MOTOR_COUNT 2

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
 *   2. 共享控制参数 (Shared Control Parameters)
 *      对所有驱动型号通用的速度、PWM 量程定义
 * ================================================================================= */

/* ------------------- 速度与PWM范围定义 (Speed & PWM Normalization) ------------------- */
#define SPEED_TICKS_MAX     90    // 单个控制周期内编码器的最大计数值 (用于速度计算) // 依据: 13PPR x 4倍频 x 34减速比 = 1768ticks/rev, 300RPM -> (300/60)*1768*0.01 ≈ 88
#define ENCODER_FILTER_ALPHA    0.1f  // 编码器IIR滤波系数，越小越平滑，响应越慢
#define SPEED_LOGIC_MAX     100   // 统一的逻辑速度最大值 (例如 0-100)
#define PWM_MAX             999   // PWM最大值, 对应定时器的ARR寄存器值, 代表100%占空比

/* =================================================================================
 *   3. 驱动器特定配置 (Driver-Specific Configuration)
 *      3.1 TB6612  |  3.2 AT8236  |  3.3 IBT4
 * ================================================================================= */

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
    /* --- TB6612 硬件配置 --- */
    /* 电机 1 (M1) 配置: PWM on TIM1_CH1, Encoder on TIM2, Pins: PB0, PB1, PB10 */
    #define MOTOR1_PWM_TIM              &htim1
    #define MOTOR1_PWM_CH               TIM_CHANNEL_1
    #define MOTOR1_IN1_PORT             GPIOB
    #define MOTOR1_IN1_PIN              GPIO_PIN_0
    #define MOTOR1_IN2_PORT             GPIOB
    #define MOTOR1_IN2_PIN              GPIO_PIN_1
    #define MOTOR1_STBY_PORT            GPIOB
    #define MOTOR1_STBY_PIN             GPIO_PIN_10
    #define MOTOR1_ENCODER_TIM          &htim2
    #define MOTOR1_POLARITY             0 
    #define MOTOR1_DEFAULT_STOP_MODE    TB6612_MOTOR_STOP_BRAKE // 默认停止模式


    /* 电机 2 (M2) 配置: PWM on TIM1_CH2, Encoder on TIM3, Pins: PB12, PB13, PB14 */
    #define MOTOR2_PWM_TIM              &htim1
    #define MOTOR2_PWM_CH               TIM_CHANNEL_2
    #define MOTOR2_IN1_PORT             GPIOB
    #define MOTOR2_IN1_PIN              GPIO_PIN_12
    #define MOTOR2_IN2_PORT             GPIOB
    #define MOTOR2_IN2_PIN              GPIO_PIN_13
    #define MOTOR2_STBY_PORT            GPIOB
    #define MOTOR2_STBY_PIN             GPIO_PIN_14
    #define MOTOR2_ENCODER_TIM          &htim3

    #define MOTOR2_POLARITY             0 
    #define MOTOR2_DEFAULT_STOP_MODE    TB6612_MOTOR_STOP_BRAKE // 默认停止模式

    /* ------------------- 电机1控制限制 (Motor 1 Control Limits) ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX        // 应用于电机的最大PWM输出值
    #define MOTOR1_MIN_PWM_OUTPUT       0              // 应用于电机的最小PWM输出值
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX// 最大逻辑速度 (用于速度映射)
    #define MOTOR1_DEAD_ZONE            1           // PWM死区, 低于此值的PWM输出将被忽略, 防止电机在零速时抖动

    /* ------------------- 电机1 PID 控制器参数 (PID Controller Constants) ------------------- */
    // ── 参数设计原则（纯 PID，无前馈）────────────────────────────────────────
    // 稳态时 error≈0，output 主要由积分提供：
    //   output_ss ≈ Ki × integral
    // 要求 Ki × integral_limit >> setpoint_max（90），使超调时 output 不归零触发刹车
    //   Ki=4, integral_limit=20 → I_max=80；output=0 需 error=-53（speed=101），物理不可达
    // ──────────────────────────────────────────────────────────────────────────
    #define MOTOR1_PID_KP                0.8f
    // #define MOTOR1_PID_KP                1.5f
    // #define MOTOR1_PID_KP                0.4584f
    // #define MOTOR1_PID_KP                0.01362f   // 
    // #define MOTOR1_PID_KP                4.183f     // 
    #define MOTOR1_PID_KI                4.0f          // 
    // #define MOTOR1_PID_KI                0.05f       // 配合1.5fkp,0kd,10ilimit稳10，20，30.再高震荡
    // #define MOTOR1_PID_KI                10.0f      // 
    #define MOTOR1_PID_KD                0.0f
    // #define MOTOR1_PID_KD                0.0000041f
    #define MOTOR1_PID_INTEGRAL_LIMIT    20.0f         // Ki×limit = 4×20 = 80 >> setpoint_max(90)
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f        // 必须 == MaxSpeed(100)，与 TB6612_Motor_SetSpeed 一致
    #define MOTOR1_PID_TS                0.01f         // PID采样时间 (秒), 此处为10ms
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f    // 微分项的低通滤波器系数 (0.0 to 1.0)

    /* ------------------- 电机2 PID 控制器参数 (PID Controller Constants) ------------------- */
    #define MOTOR2_PID_KP                0.8f          // Fix 14C: 与M1一致
    #define MOTOR2_PID_KI                0.3f          // Fix 14C: 与M1一致
    #define MOTOR2_PID_KD                0.0025f
    #define MOTOR2_PID_INTEGRAL_LIMIT    10.0f
    #define MOTOR2_PID_OUTPUT_LIMIT      100.0f
    #define MOTOR2_PID_TS                0.01f
    #define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f

    /* ------------------- 电机 2 控制限制 ------------------- */
    #define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR2_MIN_PWM_OUTPUT       0
    #define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR2_DEAD_ZONE            10   //死区影响，原本为10
    // MOTOR2_DEFAULT_STOP_MODE 已在电机2 GPIO 配置区定义（见上方第69行），此处不再重复


#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)
    /* --- AT8236 硬件配置 --- */
    /* 电机 1 (M1) */
    #define MOTOR1_TIM_HANDLE           &htim3
    #define MOTOR1_PWM_CHANNEL1         TIM_CHANNEL_1  // IN1 的 PWM
    #define MOTOR1_PWM_CHANNEL2         TIM_CHANNEL_2  // IN2 的 PWM
    #define MOTOR1_ENCODER_TIM          &htim2         // 编码器定时器  [Fix #10]
    #define MOTOR1_DEFAULT_STOP_MODE    AT8236_STOP_BRAKE

    /* 电机 2 (M2) — MOTOR_COUNT=2 时必须定义  [Fix #5] */
    #define MOTOR2_TIM_HANDLE           &htim4         // 示例: 请按实际连接修改
    #define MOTOR2_PWM_CHANNEL1         TIM_CHANNEL_1  // IN1 的 PWM
    #define MOTOR2_PWM_CHANNEL2         TIM_CHANNEL_2  // IN2 的 PWM
    #define MOTOR2_ENCODER_TIM          &htim3         // 编码器定时器  [Fix #10]
    #define MOTOR2_DEFAULT_STOP_MODE    AT8236_STOP_BRAKE

    /* ------------------- 电机 1 控制限制 ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR1_MIN_PWM_OUTPUT       0
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR1_DEAD_ZONE            5

    /* ------------------- 电机 2 控制限制  [Fix #5] ------------------- */
    #define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR2_MIN_PWM_OUTPUT       0
    #define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR2_DEAD_ZONE            5

    /* ------------------- 电机 1 PID 控制器参数 ------------------- */
    #define MOTOR1_PID_KP                0.4584f
    #define MOTOR1_PID_KI                17.66f
    #define MOTOR1_PID_KD                0.002976f
    #define MOTOR1_PID_INTEGRAL_LIMIT    500.0f
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f
    #define MOTOR1_PID_TS                0.01f
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f

    /* ------------------- 电机 2 PID 控制器参数  [Fix #5] ------------------- */
    #define MOTOR2_PID_KP                0.4584f
    #define MOTOR2_PID_KI                17.66f
    #define MOTOR2_PID_KD                0.002976f
    #define MOTOR2_PID_INTEGRAL_LIMIT    500.0f
    #define MOTOR2_PID_OUTPUT_LIMIT      100.0f
    #define MOTOR2_PID_TS                0.01f
    #define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f


#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_IBT4)
    /* --- IBT-4 硬件配置 (请根据实际连接修改) --- */
    /* 电机 1 (M1) */
    #define MOTOR1_TIM_HANDLE           &htim3         // 示例: TIM3
    #define MOTOR1_PWM_CHANNEL1         TIM_CHANNEL_3  // 示例: TIM3_CH3
    #define MOTOR1_PWM_CHANNEL2         TIM_CHANNEL_4  // 示例: TIM3_CH4
    #define MOTOR1_ENCODER_TIM          &htim2         // 编码器定时器  [Fix #10]
    #define MOTOR1_DEFAULT_STOP_MODE    IBT4_STOP_BRAKE

    /* 电机 2 (M2) — MOTOR_COUNT=2 时必须定义  [Fix #5] */
    #define MOTOR2_TIM_HANDLE           &htim4         // 示例: 请按实际连接修改
    #define MOTOR2_PWM_CHANNEL1         TIM_CHANNEL_1  // 示例: 请按实际连接修改
    #define MOTOR2_PWM_CHANNEL2         TIM_CHANNEL_2  // 示例: 请按实际连接修改
    #define MOTOR2_ENCODER_TIM          &htim3         // 编码器定时器  [Fix #10]
    #define MOTOR2_DEFAULT_STOP_MODE    IBT4_STOP_BRAKE

    /* ------------------- 电机 1 控制限制 ------------------- */
    #define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR1_MIN_PWM_OUTPUT       0
    #define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR1_DEAD_ZONE            10

    /* ------------------- 电机 2 控制限制  [Fix #5] ------------------- */
    #define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
    #define MOTOR2_MIN_PWM_OUTPUT       0
    #define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
    #define MOTOR2_DEAD_ZONE            10

    /* ------------------- 电机 1 PID 控制器参数 ------------------- */
    #define MOTOR1_PID_KP                0.4584f
    #define MOTOR1_PID_KI                17.66f
    #define MOTOR1_PID_KD                0.002976f
    #define MOTOR1_PID_INTEGRAL_LIMIT    500.0f
    #define MOTOR1_PID_OUTPUT_LIMIT      100.0f
    #define MOTOR1_PID_TS                0.01f
    #define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f

    /* ------------------- 电机 2 PID 控制器参数  [Fix #5] ------------------- */
    #define MOTOR2_PID_KP                0.4584f
    #define MOTOR2_PID_KI                17.66f
    #define MOTOR2_PID_KD                0.002976f
    #define MOTOR2_PID_INTEGRAL_LIMIT    500.0f
    #define MOTOR2_PID_OUTPUT_LIMIT      100.0f
    #define MOTOR2_PID_TS                0.01f
    #define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f

#endif


/* =================================================================================
 *   4. 系统与通信配置 (System & Communication Configuration)
 * ================================================================================= */

/* ------------------- ACK消息缓冲区 ------------------- */
#define ACK_MSG_BUF_SIZE            128

/* ------------------- CAN总线配置 ------------------- */
/* CAN 协议 */
#define CAN_MOTOR1_CMD_STDID            0x125   // 电机1控制指令 (转向电机)
#define CAN_MOTOR1_CMD_STATUS_STDID     0x225   // 电机1查询/反馈 ID
#define CAN_MOTOR1_STATUS_STDID         0x325   // 电机1状态发送 ID

#define CAN_MOTOR2_CMD_STDID            0x135   // 电机2控制指令 (动力电机)
#define CAN_MOTOR2_CMD_STATUS_STDID     0x235   // 电机2查询/反馈 ID
#define CAN_MOTOR2_STATUS_STDID         0x335   // 电机2状态发送 ID

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