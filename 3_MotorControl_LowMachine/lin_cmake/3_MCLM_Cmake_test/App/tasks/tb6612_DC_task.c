#include "app_includes.h"
#include "pid.h"
#include <math.h> // 添加 math.h 头文件以使用 fabs 和其他数学函数

void MotorControl_Task(void *argument)
{
  /* USER CODE BEGIN Start_MotorControl */

  CommandMsg_t cmdMsg;
  PID_Controller motor_pid; // 定义一个PID控制器实例

  // main.c迁移
  // 修正 Motor_Init 调用，将注释移到行连接符之外
  Motor_Init(&motor1, &htim3, TIM_CHANNEL_1,\
              GPIOB, GPIO_PIN_0,\
              GPIOB, GPIO_PIN_1,\
              GPIOA, GPIO_PIN_7, /* EN (如果没有独立的使能引脚，则为 NULL, 0) */\
              1000, 1000, 10,\
              0, MOTOR_STOP_BRAKE);

  // 3. 初始化PID控制器
  //    将Kp从10.0f降低到1.0f，作为调试的起点
  //    输出限幅保持1000，对应PWM最大值
  PID_Init(&motor_pid, 1.0f, 0.2f, 0.05f, 500.0f, 1000.0f);


  /* Infinite loop */

  for(;;)
  {
    // 4. 不再永远等待，而是以超时0的方式检查新指令
    // 恢复 osMessageQueueGet 逻辑，确保 cmdMsg 被使用
    if (osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0) == osOK)
    {
      // 如果收到了新指令，就更新PID的目标值
      if (cmdMsg.type == CMD_SET_SPEED)
      {
          motor_pid.setpoint = (float)cmdMsg.value;
      }
      else if (cmdMsg.type == CMD_STOP)
      {
          motor_pid.setpoint = 0.0f; // 设置目标值为0
      }
    }

    // 5. 执行PID闭环控制计算
    // 注意：这里我们假设 encoder_task 已经将计算好的速度放到了 motor1.target_logic_speed
    // 并且这个速度的单位与我们设定的目标值单位一致
    if (motor_pid.setpoint != 0.0f) // 仅在目标值不为0时才进行PID计算和设置速度
    {
        float current_speed = motor1.target_logic_speed;
        float output = PID_Compute(&motor_pid, current_speed);
        Motor_SetSpeed(&motor1, (int16_t)output);
    }
    else // 目标值为0时，明确停止电机
    {
        Motor_Stop(&motor1);
    }
    
    // 6. 以固定周期运行
    osDelay(10); // 保持与编码器测速周期一致
  }
  /* USER CODE END Start_MotorControl */
}

// #include "cmsis_os.h"
// #include "tb6612_DC.h"
// #include "command.h"
// #include "app_task.h"