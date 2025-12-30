#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* ===================== Motor Driver Selection ===================== */
#define MOTOR_DRIVER_TB6612 1
#define MOTOR_DRIVER_AT8236 2
// #define MOTOR_DRIVER_IBT4    3 // Example for future extension

/* Select the active motor driver */
#define ACTIVE_MOTOR_DRIVER MOTOR_DRIVER_TB6612 // <<<<<<< CHANGE THIS TO SWITCH DRIVERS
// #define ACTIVE_MOTOR_DRIVER MOTOR_DRIVER_AT8236 // <<<<<<< CHANGE THIS TO SWITCH DRIVERS

/* ===================== Speed Normalization ===================== */
/* 编码器在一个控制周期内的最大可达 tick */
#define SPEED_TICKS_MAX     80

/* 逻辑速度范围（对外接口统一） */
#define SPEED_LOGIC_MAX    100

/* PWM 输出上限（与 TIM 配置一致） */
#define PWM_MAX            999

// PID Constants for Motor Control
#define MOTOR_PID_KP 0.4584f
#define MOTOR_PID_KI 17.66f
#define MOTOR_PID_KD 0.002976f
#define MOTOR_PID_INTEGRAL_LIMIT 500.0f
#define MOTOR_PID_OUTPUT_LIMIT 100.0f
#define PID_TS                      0.01f // PID 采样周期 (10ms)
#define PID_DERIVATIVE_FILTER_ALPHA 0.3f  // 微分项滤波系数 (0.0 - 1.0)

// Motor Initialization Parameters
#define MOTOR1_TIM_HANDLE           &htim3
#define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1
#define MOTOR1_IN1_PORT             GPIOB
#define MOTOR1_IN1_PIN              GPIO_PIN_0
#define MOTOR1_IN2_PORT             GPIOB
#define MOTOR1_IN2_PIN              GPIO_PIN_1
#define MOTOR1_EN_PORT              GPIOB
#define MOTOR1_EN_PIN               GPIO_PIN_10
#define MOTOR1_MAX_SPEED_LOGIC      100
#define MOTOR1_MAX_PWM_OUTPUT       100
#define MOTOR1_MIN_PWM_OUTPUT       10
#define MOTOR1_DEAD_ZONE            0
#define MOTOR1_STOP_MODE            TB6612_MOTOR_STOP_BRAKE // Updated to use TB6612_MOTOR_STOP_BRAKE

// ACK Message Buffer Size
#define ACK_MSG_BUF_SIZE            128 // 定义ACK消息缓冲区的最大大小

// CAN Configuration
#define CAN_MOTOR_CMD_STDID         0x40 // 定义电机控制命令的CAN标准ID

// CAN Initialization Parameters
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

// CAN Filter Configuration
#define CAN_FILTER_BANK             0    // CAN过滤器组
#define CAN_FILTER_MODE             CAN_FILTERMODE_IDMASK // CAN过滤器模式，修改为 IDMASK
#define CAN_FILTER_SCALE            CAN_FILTERSCALE_32BIT // CAN过滤器位宽
#define CAN_FILTER_ID_HIGH          0x0000
#define CAN_FILTER_ID_LOW           0x0000
#define CAN_FILTER_MASK_ID_HIGH     0x0000
#define CAN_FILTER_MASK_ID_LOW      0x0000
#define CAN_FILTER_FIFO             CAN_RX_FIFO0 // CAN过滤器FIFO
#define CAN_FILTER_ACTIVATION       ENABLE // 激活CAN过滤器
#define CAN_SLAVE_START_FILTER_BANK 14

// #define CAN_CMD_QUEUE_SIZE          10   // 新增：CAN 命令队列大小

#endif // APP_CONFIG_H