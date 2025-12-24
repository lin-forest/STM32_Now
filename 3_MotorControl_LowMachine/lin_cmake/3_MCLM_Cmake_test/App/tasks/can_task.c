// #include "app_includes.h"

// void CAN_Task(void *argument)
// {
//     uint8_t rx_data[8];
//     char cmdStr[9];

//     for (;;)
//     {
//         if (osMessageQueueGet(CanRxQueueHandle, rx_data, NULL, osWaitForever) == osOK)
//         {
//             memset(cmdStr, 0, sizeof(cmdStr));
//             memcpy(cmdStr, rx_data, 8);

//             CommandMsg_t msg = Command_ParseString(cmdStr);
//             msg.src = CMD_SRC_CAN;

//             Command_Push(&msg);
//         }
//     }
// }
