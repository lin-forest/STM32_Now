
#include "app_includes.h"

void MotorControl_Task(void *argument)
{
  /* USER CODE BEGIN Start_MotorControl */

  // uint8_t cmd;
  CommandMsg_t cmdMsg;
  // AckMsg_t ack;

  // main.c迁移
    Motor_Init(&motor1, &htim3, TIM_CHANNEL_1,
              GPIOB, GPIO_PIN_0,
              GPIOB, GPIO_PIN_1,
              GPIOA, GPIO_PIN_7,              // EN (如果没有独立的使能引脚，则为 NULL, 0)
              1000, 1000, 10,
              0, MOTOR_STOP_BRAKE);

  /* Infinite loop */

  for(;;)
  {
    if (osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, osWaitForever) == osOK)
    {
      /* 默认 ACK */
      // ack.type  = cmdMsg.type;
      // ack.value = cmdMsg.value;
      // ack.ok    = 1;   // 先假设成功

      switch (cmdMsg.type)
      {
        case CMD_FORWARD:
            Motor_SetSpeed(&motor1, 80);
            // UART2_Print("CMD: Forward\r\n");
            break;
        case CMD_REVERSE:
            Motor_SetSpeed(&motor1, -80);
            // UART2_Print("CMD: Reverse\r\n");
            break;
        case CMD_STOP:
            Motor_Stop(&motor1);
            // UART2_Print("CMD: Stop\r\n");
            break;
        case CMD_SET_SPEED:
            // 用的最多
            Motor_SetSpeed(&motor1, cmdMsg.value);
            break;

        default:
            // Motor_Stop(&motor1);
            // ack.ok = 0;   // 未支持命令
            break;
      }

      /* ======== 关键：投递 ACK ======== */
      // osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
    }
    osDelay(10);
  }
  /* USER CODE END Start_MotorControl */
}

// #include "cmsis_os.h"
// #include "tb6612_DC.h"
// #include "command.h"
// #include "app_task.h"
