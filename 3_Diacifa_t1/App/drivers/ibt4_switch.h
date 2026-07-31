#ifndef __IBT4_SWITCH_H__
#define __IBT4_SWITCH_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================================
 *  IBT4 气动负载控制接口
 *
 *  硬件映射（每块 IBT4 用 2 路 PWM 控制一个马达的方向）：
 *    气泵1 → IBT4#1 PWM1(CH1/PA0)=正转  + PWM2(CH2/PA1)=反转(保持0)
 *    气泵2 → IBT4#2 PWM1(CH3/PA2)=正转  + PWM2(CH4/PA3)=反转(保持0)
 *    电磁阀1 → GPIO PB0        （后续 IBT4）
 *    电磁阀2 → GPIO PB1        （后续 IBT4）
 *
 *  IBT4 模块仅露出 PWM 输入引脚（无 EN 引脚），
 *  PWM1 正转占空比控制输出功率，PWM2 反转保持 0：
 *    占空比=0 → 停止
 *    占空比>0 → 正转调速
 * ================================================================================= */

/**
 * @brief  设置气泵 PWM 占空比
 * @param  pump_id  泵 ID (PUMP1_ID=0, PUMP2_ID=1)
 * @param  duty_100 占空比 0-100 (0=停止, 100=全速)
 */
void IBT4_Pump_SetSpeed(uint8_t pump_id, uint8_t duty_100);

/**
 * @brief  获取气泵当前 PWM 值
 * @param  pump_id  泵 ID
 * @return 0-100
 */
uint8_t IBT4_Pump_GetSpeed(uint8_t pump_id);

/**
 * @brief  设置电磁阀开/关
 * @param  valve_id 阀 ID (VALVE1_ID=0, VALVE2_ID=1)
 * @param  on       0=失电(关), 1=得电(开)
 */
void IBT4_Valve_Set(uint8_t valve_id, uint8_t on);

/**
 * @brief  获取电磁阀状态
 * @param  valve_id 阀 ID
 * @return 0=关, 1=开
 */
uint8_t IBT4_Valve_GetState(uint8_t valve_id);

#ifdef __cplusplus
}
#endif

#endif // __IBT4_SWITCH_H__
