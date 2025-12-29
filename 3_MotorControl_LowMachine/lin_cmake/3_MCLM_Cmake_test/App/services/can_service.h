// #ifndef __CAN_SERVICE_H
// #define __CAN_SERVICE_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// /* Includes ------------------------------------------------------------------*/
// #include "FreeRTOS.h"
// #include "queue.h"
// #include "command.h" // 包含 command.h 以使用 CommandMsg_t 和 CommandType_t
// #include "stm32f1xx_hal_can.h" // 确保包含此头文件，它定义了 CAN_HandleTypeDef
// extern CAN_HandleTypeDef hcan; // 声明 hcan 为外部变量

// void CanService_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// /* USER CODE BEGIN Includes */

// /* USER CODE END Includes */

// /* ===================== CAN 命令类型定义 (已移至 command.h) ===================== */
// // typedef enum {
// //     CAN_CMD_NONE = 0,
// //     CAN_CMD_SET_SPEED = 0x01,
// //     CAN_CMD_STOP = 0x02,
// //     // ... 其他 CAN 命令
// // } CanCmdType_t;

// /* ===================== CAN 反馈消息结构体 ===================== */
// typedef struct {
//     float current_logic_speed;
//     int16_t pwm_output;
//     uint8_t motor_state; // 0: 停止, 1: 运行
// } CanFeedback_t;

// /* ===================== CAN 命令消息队列句柄 ===================== */
// extern osMessageQueueId_t CanMotorCmdQueueHandle; // 统一为 osMessageQueueId_t 类型

// /* ===================== 函数声明 ===================== */
// void CanService_Init(void);
// void CanService_SendFeedback(CanFeedback_t *feedback);
// void CanService_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// /* USER CODE BEGIN Prototypes */

// /* USER CODE END Prototypes */

// #ifdef __cplusplus
// }
// #endif

// #endif /* __CAN_SERVICE_H */