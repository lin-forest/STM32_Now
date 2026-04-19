#include "app_includes.h"
#include "pid.h"
// #include "can_service.h"
#include "motor_DC_tb6612.h" // Added this include
#include "math.h"   // For fabsf
#include "float.h"  // For FLT_EPSILON

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)

void TB6612_DC_Task(void *argument)
{
    // 获取电机实例，若 argument 为空则默认指向第一个
    Motor_t *motor = (argument != NULL) ? (Motor_t *)argument : &g_motors[0];

    Motor_PID_Init(motor); // 初始化该实例的 PID

    CommandMsg_t cmdMsg;
    // uint64_t queueData; // 用于接收两个队列的数据
    // CanFeedback_t canFeedback;
    // uint8_t feedbackCounter = 0;

    // main.c迁移
    // 参数顺序: MaxPWM, MaxSpeed, DeadZone, Polarity(1=反转,由硬件接线决定), StopMode
    // 注意: 此电机硬件接线需要极性反转(Polarity=1)，使编码器正方向与指令正方向一致
    TB6612_Motor_Init(&(motor->hardware), MOTOR1_TIM_HANDLE, MOTOR1_TIM_CHANNEL,\
                MOTOR1_IN1_PORT, MOTOR1_IN1_PIN,\
                MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,\
                MOTOR1_EN_PORT, MOTOR1_EN_PIN,\
                MOTOR1_MAX_PWM_OUTPUT, MOTOR1_MAX_SPEED_LOGIC, MOTOR1_DEAD_ZONE,\
                1, TB6612_MOTOR_STOP_BRAKE);

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status;
        uint8_t messageProcessed = 0;

        status = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (status == osOK)
        {
            messageProcessed = 1;
            if (cmdMsg.type == CMD_SET_SPEED || cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                float new_setpoint = (float)cmdMsg.value;
                // setpoint 符号反转时清除积分，避免积分残留导致超调
                if ((new_setpoint > 0.0f && motor->pid.setpoint < 0.0f) ||
                    (new_setpoint < 0.0f && motor->pid.setpoint > 0.0f))
                {
                    PID_Reset(&(motor->pid));
                }
                motor->pid.setpoint = new_setpoint;
            }
            else if (cmdMsg.type == CMD_STOP || cmdMsg.type == CAN_CMD_STOP)
            {
                motor->pid.setpoint = 0.0f;
                PID_Reset(&(motor->pid));
            }
        }

        if (osMutexAcquire(motor0_mutexHandle, osWaitForever) == osOK)
        {
            motor->target_logic_speed = motor->pid.setpoint;
            if (fabsf(motor->pid.setpoint) > FLT_EPSILON)
            {
                float current_speed = motor->current_logic_speed;
                float output = PID_Compute(&(motor->pid), current_speed);
                TB6612_Motor_SetSpeed(&(motor->hardware), (int16_t)output);
                motor->pwm_output = motor->hardware.pwm_output;
            }
            else
            {
                TB6612_Motor_Stop(&(motor->hardware));
                motor->pwm_output = 0;
            }
            osMutexRelease(motor0_mutexHandle);
        }

        osDelay(10);
    }
}

#endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)