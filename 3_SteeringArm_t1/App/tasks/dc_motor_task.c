/* ============================================================
 *  dc_motor_task.c — J0 DC 电机 PID 控制 (ARM_CFG_SET==1)
 *
 *  架构 (串级控制):
 *    位置外环 (50ms) → 速度内环 (10ms) → IBT4 PWM 输出
 *
 *  硬件:
 *    TIM1_CH1 (PA8) → IBT4 RPWM
 *    TIM2 (PA0/PA1) → 霍尔编码器
 *
 *  参考: 3_MCLM_t2 tb6612_DC_task.c
 * ============================================================ */
#include <stdio.h>
#include <math.h>
#include "app_config.h"
#include "app_globals.h"
#include "motor_DC_IBT4.h"
#include "pid.h"
#include "tim.h"
#include "cmsis_os.h"

extern ArmState_t g_arm_state;

#if ARM_CFG_SET == 1

/* ── IBT4 电机实例 ── */
static IBT4_Motor_t j0_motor = {
    .htim            = &htim1,
    .Channel_Forward = TIM_CHANNEL_1,   /* RPWM */
    .Channel_Reverse = TIM_CHANNEL_2,   /* LPWM (或 PB12 独立引脚) */
    .EN_Port         = NULL,
    .EN_Pin          = 0,
    .MaxPWM          = J0_PWM_MAX,
    .MaxSpeed        = J0_SPEED_LOGIC_MAX,
    .DeadZone        = 0,
    .Polarity        = 0,
    .pwm_output      = 0,
};

/* ── PID 实例 ── */
static PID_Controller speed_pid;

/* ── 编码器 ── */
static int16_t last_cnt = 0;
static int32_t abs_ticks = 0;

static float ticks_to_speed(int16_t diff)
{
    /* diff: 10ms 内的编码器差值
       48 ticks/rev, 假设最高 ~9000 RPM → ~150 rev/s
       逻辑速度范围: diff × 换算系数 */
    return (float)diff * 0.5f;  /* 粗略换算，需实测标定 */
}

#endif /* ARM_CFG_SET == 1 */


void DC_Motor_Task_Impl(void *argument)
{
#if ARM_CFG_SET == 1
    /* ── R1: 初始化 J0 DC 电机 ── */
    IBT4_Motor_Init(&j0_motor,
                    &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                    NULL, 0,
                    J0_PWM_MAX, J0_SPEED_LOGIC_MAX,
                    0, 0);
    PID_Init(&speed_pid,
             J0_SPEED_KP, J0_SPEED_KI, J0_SPEED_KD,
             J0_SPEED_INTEGRAL_LIMIT, J0_SPEED_OUTPUT_LIMIT,
             J0_SPEED_TS, 0.3f);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    last_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    printf("DC_Motor_Task (R1 J0) started\r\n");

    for (;;)
    {
        /* 1. 读编码器 */
        int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t diff = now - last_cnt;
        last_cnt = now;
        abs_ticks += diff;
        g_arm_state.j0_encoder_raw = (uint16_t)now;

        float current_speed = ticks_to_speed(diff);
        g_arm_state.j0_current_speed = (int16_t)current_speed;

        /* 2. 设目标速度 → PID 计算 → IBT4 输出 */
        PID_SetSetpoint(&speed_pid, (float)g_arm_state.j0_target);
        float output = PID_Compute(&speed_pid, current_speed);
        IBT4_Motor_SetSpeed(&j0_motor, (int16_t)output);

        /* 4. 调试输出 (降低频率, 整数拆分免 %f) */
        static uint32_t print_cnt = 0;
        if (++print_cnt % 50 == 0) {
            int32_t cur_i = (int32_t)current_speed;
            int32_t cur_d = (int32_t)(fabsf(current_speed - (float)cur_i) * 10.0f);
            printf("J0: target=%d current=%d.%d pwm=%d\r\n",
                   (int)g_arm_state.j0_target, (int)cur_i, (int)cur_d, j0_motor.pwm_output);
        }

        osDelay(10);  /* 10ms 速度内环 */
    }

#else  /* ARM_CFG_SET != 1 */
    /* ── R2: J0 暂缓 ── */
    printf("DC_Motor_Task (R2): J0 disabled\r\n");
    for (;;)
    {
        osDelay(1000);
    }
#endif
}
