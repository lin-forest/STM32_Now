#include "app_includes.h"

/* =================================================================================
 *  气泵控制
 *  每块 IBT4 用 2 路 PWM 控制一个马达的方向：
 *    PWM1（正转）= 设定占空比 → 马达正转调速
 *    PWM2（反转）= 保持 0    → 马达不反转
 *
 *  硬件映射：
 *    气泵1 → IBT4#1 PWM1(CH1/PA0)正转 + PWM2(CH2/PA1)反转(0)
 *    气泵2 → IBT4#2 PWM1(CH3/PA2)正转 + PWM2(CH4/PA3)反转(0)
 *
 *  CCR = duty_100 * PWM_PERIOD / 100
 *    duty_100=0   → 输出 0V  → 停止
 *    duty_100=50  → 输出 ~6V  → 半速
 *    duty_100=100 → 输出 12V  → 全速
 * ================================================================================= */

static uint8_t s_pump_speed[2] = {0, 0};    // 当前气泵占空比
static uint8_t s_valve_state[2] = {0, 0};    // 当前电磁阀状态

void IBT4_Pump_SetSpeed(uint8_t pump_id, uint8_t duty_100)
{
    if (pump_id > 1) return;
    if (duty_100 > 100) duty_100 = 100;

    s_pump_speed[pump_id] = duty_100;

    uint32_t ccr = (uint32_t)duty_100 * PWM_PERIOD / 100;

    if (pump_id == PUMP1_ID) {
        /* 气泵1：PWM1=正转占空比，PWM2=反转关闭 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    } else {
        /* 气泵2：PWM1=正转占空比，PWM2=反转关闭 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
}

uint8_t IBT4_Pump_GetSpeed(uint8_t pump_id)
{
    if (pump_id > 1) return 0;
    return s_pump_speed[pump_id];
}

/* =================================================================================
 *  电磁阀控制
 *  PB0 → 电磁阀1, PB1 → 电磁阀2
 *
 *  IBT4 模块：PWM 输入 = GPIO 高电平 → 通道导通 → 阀得电
 *            PWM 输入 = GPIO 低电平 → 通道关断 → 阀失电
 * ================================================================================= */

void IBT4_Valve_Set(uint8_t valve_id, uint8_t on)
{
    if (valve_id > 1) return;

    s_valve_state[valve_id] = on ? 1 : 0;

    if (valve_id == VALVE1_ID) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

uint8_t IBT4_Valve_GetState(uint8_t valve_id)
{
    if (valve_id > 1) return 0;
    return s_valve_state[valve_id];
}
