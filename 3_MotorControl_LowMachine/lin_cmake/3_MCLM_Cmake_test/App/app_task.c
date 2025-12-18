#include "app_includes.h"
#include <stdint.h>

Motor_t motor1;          // 电机对象
uint8_t tx_buf[64];      // 日志缓存


/* 编码器 tick → 逻辑速度 */
int16_t ticks_to_logic(int16_t ticks);

/* 逻辑速度 → PWM */
int16_t logic_to_pwm(int16_t logic);

/* 逻辑速度 → 编码器目标 tick（给 PID 用） */
int16_t logic_to_ticks(int16_t logic);

