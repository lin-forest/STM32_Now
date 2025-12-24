#include "app_includes.h"
#include <stdint.h>
#include "can_service.h" // Add this line

Motor_t motor1;          // 电机对象
uint8_t tx_buf[64];      // 日志缓存


/* 编码器 tick → 逻辑速度 */
int16_t ticks_to_logic(int16_t ticks);

/* 逻辑速度 → PWM */
int16_t logic_to_pwm(int16_t logic);

/* 逻辑速度 → 编码器目标 tick（给 PID 用） */
int16_t logic_to_ticks(int16_t logic);


void AppTask_Init(void)
{
    // 先初始化 CAN 服务，再创建电机任务
    CanService_Init();
    
    // 现有任务创建逻辑（不变）
//     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);
//     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);
//     osThreadNew(tb6612_DC_Task, NULL, &tb6612_DC_TaskAttributes);

}