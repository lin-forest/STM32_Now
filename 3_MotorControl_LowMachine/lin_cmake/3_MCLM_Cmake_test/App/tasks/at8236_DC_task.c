// #include "app_includes.h"
// #include "command.h"
// #include "logger.h"

// // Declare the motor instance and the message queue as external
// extern At8236_Motor_t at8236_A;
// extern osMessageQueueId_t at8236_DC_Queue;
// extern TIM_HandleTypeDef htim3;

// void at8236_DC_Task(void *argument)
// {
//     CommandMsg_t cmdMsg; // Use the correct message type directly

//     // Initialize the AT8236 motor
//     At8236_Motor_Init(&at8236_A, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 100, 100);
//     at8236_A.MinPwm = 30; // Set minimum PWM to 30% to overcome dead zone

//     // LOG_INFO("at8236_DC_Task is running.");

//     for (;;)
//     {
//         // Wait for a command from the queue, receive into the struct directly
//         if (osMessageQueueGet(at8236_DC_Queue, &cmdMsg, NULL, osWaitForever) == osOK)
//         {
//             if (cmdMsg.type == CMD_SET_SPEED)
//             {
//                 // The speed is in the 'value' member
//                 int16_t speed = cmdMsg.value;
//                 At8236_Motor_SetSpeed(&at8236_A, speed);
//                 // LOG_INFO("AT8236 Motor A speed set to %d", speed);
//             }
//             else if (cmdMsg.type == CMD_STOP)
//             {
//                 At8236_Motor_Stop(&at8236_A, AT8236_STOP_BRAKE);
//                 // LOG_INFO("AT8236 Motor A stopped.");
//             }
//             // No need to free memory as the message is copied, not pointed to.
//         }
//     }
// }