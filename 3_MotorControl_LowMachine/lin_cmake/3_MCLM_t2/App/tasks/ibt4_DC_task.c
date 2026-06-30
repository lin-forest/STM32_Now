#include "app_includes.h"
#include "pid.h"
#include "motor_DC_IBT4.h"
#include "math.h"
#include "float.h"

/* 允许两电机分别使用 IBT4，也可混合使用（任一驱动类型为 IBT4 即编译本文件） */
#if (MOTOR1_DRIVER == MOTOR_DRIVER_IBT4) || (MOTOR2_DRIVER == MOTOR_DRIVER_IBT4)

void IBT4_DC_Task(void *argument)
{
    Motor_t *motor = (argument != NULL) ? (Motor_t *)argument : &g_motors[0];
    uint8_t idx = (uint8_t)(motor - &g_motors[0]);

    Motor_PID_Init(motor);

    IBT4_Motor_t hw;
#if (MOTOR1_DRIVER == MOTOR_DRIVER_IBT4)
    if (idx == 0) {
        IBT4_Motor_Init(&hw,
                        MOTOR1_IBT4_TIM,
                        MOTOR1_IBT4_CH_F,
                        MOTOR1_IBT4_CH_R,
                        MOTOR1_IBT4_EN_PORT, MOTOR1_IBT4_EN_PIN,
                        MOTOR1_MAX_PWM_OUTPUT, MOTOR1_MAX_SPEED_LOGIC,
                        MOTOR1_DEAD_ZONE, MOTOR1_IBT4_POLARITY);
    } else
#endif
    {
        IBT4_Motor_Init(&hw,
                        MOTOR2_IBT4_TIM,
                        MOTOR2_IBT4_CH_F,
                        MOTOR2_IBT4_CH_R,
                        MOTOR2_IBT4_EN_PORT, MOTOR2_IBT4_EN_PIN,
                        MOTOR2_MAX_PWM_OUTPUT, MOTOR2_MAX_SPEED_LOGIC,
                        MOTOR2_DEAD_ZONE, MOTOR2_IBT4_POLARITY);
    }

    osMessageQueueId_t myQueue = (idx == 0) ? MotorQueueHandle : MotorQueue1Handle;
    osMutexId_t        myMutex = (idx == 0) ? motor0_mutexHandle : motor1_mutexHandle;

    CommandMsg_t cmdMsg;

    for (;;)
    {
        uint8_t messageProcessed = 0;

        if (osMessageQueueGet(myQueue, &cmdMsg, NULL, 0) == osOK)
        {
            messageProcessed = 1;
            if (cmdMsg.type == CMD_SET_SPEED || cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                float new_setpoint = (float)cmdMsg.value;
                if ((new_setpoint > 0.0f && motor->pid.setpoint < 0.0f) ||
                    (new_setpoint < 0.0f && motor->pid.setpoint > 0.0f))
                {
                    PID_Reset(&(motor->pid));
                }
                motor->pid.setpoint = new_setpoint;
            }
            else if (cmdMsg.type == CMD_FORWARD)
            {
                if (motor->pid.setpoint < 0.0f)
                    PID_Reset(&(motor->pid));
                motor->pid.setpoint = MOTOR_CMD_DEFAULT_SPEED;
            }
            else if (cmdMsg.type == CMD_REVERSE)
            {
                if (motor->pid.setpoint > 0.0f)
                    PID_Reset(&(motor->pid));
                motor->pid.setpoint = -MOTOR_CMD_DEFAULT_SPEED;
            }
            else if (cmdMsg.type == CMD_STOP || cmdMsg.type == CAN_CMD_STOP)
            {
                motor->pid.setpoint = 0.0f;
                PID_Reset(&(motor->pid));
            }
        }

        if (osMutexAcquire(myMutex, osWaitForever) == osOK)
        {
            motor->target_logic_speed = motor->pid.setpoint;
            if (fabsf(motor->pid.setpoint) > FLT_EPSILON)
            {
                float current_speed = motor->current_logic_speed;

                if (fabsf(current_speed) < 1.0f)
                {
                    if (motor->stall_counter < 255)
                        motor->stall_counter++;
                    if (motor->stall_counter > 5)
                        motor->flags |= MOTOR_FLAG_STALL;
                }
                else
                {
                    motor->stall_counter = 0;
                    motor->flags &= ~MOTOR_FLAG_STALL;
                }

                float output = PID_Compute(&(motor->pid), current_speed);
                IBT4_Motor_SetSpeed(&hw, (int16_t)output);
                motor->pwm_output = hw.pwm_output;

                if (hw.pwm_output >= (PWM_MAX - 10) &&
                    fabsf(current_speed - motor->pid.setpoint) > 10.0f)
                {
                    motor->flags |= MOTOR_FLAG_SATURATED;
                }
                else
                {
                    motor->flags &= ~MOTOR_FLAG_SATURATED;
                }
            }
            else
            {
                IBT4_Motor_Stop(&hw);
                motor->pwm_output = 0;
                motor->stall_counter = 0;
                motor->flags = 0;
            }
            osMutexRelease(myMutex);
        }

        osDelay(10);
    }
}

#endif // (MOTOR1_DRIVER == MOTOR_DRIVER_IBT4) || (MOTOR2_DRIVER == MOTOR_DRIVER_IBT4)
