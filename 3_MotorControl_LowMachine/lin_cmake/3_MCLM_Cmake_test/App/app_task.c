#include "app_includes.h"

// Define the global motor status object
MotorStatus_t g_motor_status;

// TB6612_Motor_t tb6612_motor1; // REMOVED - This will be local to the task that uses it.
uint8_t tx_buf[64];      // 日志缓存

// 移除这里的 ticks_to_logic, logic_to_pwm, logic_to_ticks 函数声明
// 它们现在统一在 speed_map.h 中

// 移除被注释掉的 AppTask_Init 函数
// {
//     // 先初始化 CAN 服务，再创建电机任务
//     CanService_Init();
    
//     // 现有任务创建逻辑（不变）
// //     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);
// //     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);
// //     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);

// }