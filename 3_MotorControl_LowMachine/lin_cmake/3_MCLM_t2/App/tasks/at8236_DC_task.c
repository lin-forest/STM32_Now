#include "app_includes.h"
#include "pid.h"
#include "motor_DC_at8236.h"
#include "math.h"
#include "float.h"
#include "stm32f1xx_hal_tim.h"

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)

// Make the motor object local and static to this task
static At8236_Motor_t at8236_A;

// PID_Controller is now globally defined, so we don't need a local one.
extern PID_Controller motor_pid;

void AT8236_DC_Task(void *argument)
{
    // Initialize PID controller (assuming Motor_PID_Init is globally available)
    Motor_PID_Init();

    CommandMsg_t cmdMsg;

    // Initialize the AT8236 motor with parameters from app_config.h
    // Note: AT8236 uses two PWM channels. We need to define these in app_config.h
    // For now, I'll use placeholders like MOTOR1_TIM_CHANNEL_2
    At8236_Motor_Init(&at8236_A, 
                      MOTOR1_TIM_HANDLE, 
                      MOTOR1_PWM_CHANNEL1, // Corresponds to IN1
                      MOTOR1_PWM_CHANNEL2,      // Corresponds to IN2 - Placeholder!
                      MOTOR1_MAX_PWM_OUTPUT, 
                      MOTOR1_MAX_SPEED_LOGIC);
    at8236_A.MinPwm = MOTOR1_MIN_PWM_OUTPUT;

    for (;;)
    {
        osStatus_t status;
        uint8_t messageProcessed = 0;

        // 1. 从统一的指令队列获取指令 (串口和CAN指令现在都通过MotorQueueHandle发送)
        status = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (status == osOK)
        {
            messageProcessed = 1;

            // 2. 根据指令类型处理
            if (cmdMsg.type == CMD_SET_SPEED || cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = motor_pid.setpoint;
                    osMutexRelease(motor_mutexHandle);
                }
            }
            else if (cmdMsg.type == CMD_STOP || cmdMsg.type == CAN_CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = 0.0f;
                    osMutexRelease(motor_mutexHandle);
                }
            }
            // 注意: ACK 消息由各自的命令源任务（如 Command_Task, Can_Service_Task）处理，
            // 此任务只负责执行命令。
        }

        // 3. Execute PID control loop
        if (fabsf(motor_pid.setpoint) > FLT_EPSILON)
        {
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                float current_speed = g_motor_status.current_logic_speed;
                float output = PID_Compute(&motor_pid, current_speed);
                At8236_Motor_SetSpeed(&at8236_A, (int16_t)output);
                g_motor_status.pwm_output = at8236_A.pwm_output; // Update global status
                osMutexRelease(motor_mutexHandle);
            }
        }
        else
        {
            At8236_Motor_Stop(&at8236_A, AT8236_STOP_BRAKE);
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                g_motor_status.pwm_output = 0;
                osMutexRelease(motor_mutexHandle);
            }
        }

        // 如果本次循环没有处理任何消息，就短暂休眠以让出CPU
        if (!messageProcessed) {
            osDelay(5);
        }
        // The main loop delay should be consistent with the PID computation period.
        osDelay(10);
    }
}

#endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)