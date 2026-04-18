#include "motor_DC_tb6612.h"
#include "stdlib.h" // Uncommented this line
#include "app_includes.h"


/**
 * @brief 初始化电机结构体和硬件
 * @param motor      电机结构体指针
 * @param htim       PWM 定时器句柄
 * @param Channel    PWM 通道
 * @param IN1_Port   IN1 GPIO 端口
 * @param IN1_Pin    IN1 GPIO 引脚
 * @param IN2_Port   IN2 GPIO 端口
 * @param IN2_Pin    IN2 GPIO 引脚
 * @param EN_Port    EN 使能 GPIO 端口（可选）
 * @param EN_Pin     EN 使能 GPIO 引脚
 * @param MaxPWM     TIM PWM 最大计数值
 * @param MaxSpeed   用户逻辑最大速度
 * @param DeadZone   PWM 死区
 * @param Polarity   极性反转标志（0=正常,1=反转）
 * @param StopMode   停止模式
 */
void TB6612_Motor_Init(TB6612_Motor_t *motor, // Renamed from Motor_Init
                TIM_HandleTypeDef *htim, uint32_t Channel,
                GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
                uint16_t MaxPWM, int16_t MaxSpeed, uint16_t DeadZone,
                uint8_t Polarity, TB6612_MotorStopMode_t StopMode) // Renamed from MotorStopMode_t
{
    // 硬件绑定
    motor->htim = htim;
    motor->Channel = Channel;
    motor->IN1_Port = IN1_Port;
    motor->IN1_Pin = IN1_Pin;
    motor->IN2_Port = IN2_Port;
    motor->IN2_Pin = IN2_Pin;
    motor->EN_Port = EN_Port;
    motor->EN_Pin = EN_Pin;

    // 参数绑定
    motor->MaxPWM = MaxPWM;
    motor->MaxSpeed = MaxSpeed;
    motor->DeadZone = DeadZone;
    motor->Polarity = Polarity;
    motor->StopMode = StopMode;
    // motor->Polarity = (Polarity > 0) ? 1 : 0;
    // motor->StopMode = (StopMode == TB6612_MOTOR_STOP_BRAKE) ? TB6612_MOTOR_STOP_BRAKE : TB6612_MOTOR_STOP_COAST; // Updated enum members

    // 初始化状态
    motor->current_ticks = 0;
    motor->measured_speed      = 0; // 确保初始化
    motor->target_logic_speed  = 0;
    motor->pwm_output    = 0;

    // 启动 PWM
    HAL_TIM_PWM_Start(motor->htim, motor->Channel);

    // 使能电机（如果有 EN 引脚）
    if (motor->EN_Port != NULL)
    {
        HAL_GPIO_WritePin(motor->EN_Port, motor->EN_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 设置电机速度
 * @param motor 电机结构体
 * @param speed 目标速度 [-MaxSpeed, MaxSpeed]
 */
void TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int16_t speed) // Renamed from Motor_SetSpeed
{
    // 限幅
    if (speed > motor->MaxSpeed) speed = motor->MaxSpeed;
    if (speed < -motor->MaxSpeed) speed = -motor->MaxSpeed;

    // 极性反转
    if (motor->Polarity) speed = -speed;

    // 死区处理
    if (speed > 0 && speed < motor->DeadZone) speed = motor->DeadZone;
    if (speed < 0 && speed > -motor->DeadZone) speed = -motor->DeadZone;

    // 转换为 PWM 值
    uint16_t pwmVal = 0;
    if (motor->MaxSpeed != 0)
        pwmVal = (uint16_t)((abs(speed) * motor->MaxPWM) / motor->MaxSpeed);

    // 输出 PWM 并设置方向
    if (speed > 0)
    {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, pwmVal);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, pwmVal);
    }
    else
    {
        // Fix 14A: speed==0时只停PWM，不调用TB6612_Motor_Stop()
        // 因为Stop()会清零current_ticks等共享状态，破坏Encoder_Task的数据
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, 0);
        if (motor->StopMode == TB6612_MOTOR_STOP_COAST)
        {
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        }
        pwmVal = 0;
    }

    // 更新结构体状态
    motor->target_logic_speed = speed;
    motor->pwm_output = pwmVal;
}

/**
 * @brief 停止电机
 * @param motor 电机结构体
 */
void TB6612_Motor_Stop(TB6612_Motor_t *motor) // Renamed from Motor_Stop
{
    // 停止 PWM 输出
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, 0);

    // 选择停止模式
    if (motor->StopMode == TB6612_MOTOR_STOP_COAST) // Updated enum member
    {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    }

    // Fix 14A: 不在 Stop() 中清零共享状态
    // current_ticks / measured_speed 由 Encoder_Task 维护，Stop() 不能覆盖
    // target_logic_speed / pid.setpoint 由 Motor_Task 维护，Stop() 不能覆盖
    // 只更新 pwm_output 反映实际 PWM 已为 0
    motor->pwm_output = 0;
}