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
    uint8_t my_idx = (motor == &g_motors[0]) ? 0 : 1;

    Motor_PID_Init(motor); // 初始化该实例的 PID
    CommandMsg_t cmdMsg;
    // 硬件结构体已在 app_task.c 中初始化，此处不再重复调用 TB6612_Motor_Init

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status;

        status = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (status == osOK)
        {
            // 检查该指令是否是发给本电机的
            if (cmdMsg.motor_index != my_idx) {
                // 如果不是本电机的指令，放回队列并让出微小时间片给另一个任务
                osMessageQueuePut(MotorQueueHandle, &cmdMsg, 0, 0);
                osDelay(1); 
            }
            else {
                if (cmdMsg.type == CMD_SET_SPEED || cmdMsg.type == CAN_CMD_SET_SPEED)
                {
                    float new_setpoint = (float)cmdMsg.value;
                    // setpoint 符号反转时清除积分
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
        }

        // 锁内：只读共享数据
        float current_speed = 0.0f;
        if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
        {
            motor->target_logic_speed = motor->pid.setpoint;
            current_speed = motor->measured_speed;
            osMutexRelease(motor_mutexHandle);
        }

        // 锁外：PID 计算（浮点运算，不持锁）
        if (fabsf(motor->pid.setpoint) > FLT_EPSILON)
        {
            float output = PID_Compute(&(motor->pid), current_speed);
            TB6612_Motor_SetSpeed(&(motor->hardware), (int16_t)output);

            // 锁内：只写结果
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                motor->pwm_output = motor->hardware.pwm_output;
                osMutexRelease(motor_mutexHandle);
            }
        }
        else
        {
            TB6612_Motor_Stop(&(motor->hardware));
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                motor->pwm_output = 0;
                osMutexRelease(motor_mutexHandle);
            }
        }

        osDelay(10);
    }
}

#endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)