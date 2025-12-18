#include "command.h"
// #include "usart.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

//20251210,为什么这里加入了死定义？明明motor.h结构体内定义了Maxspeed
// #define MAX_SPEED 1000
// #define MIN_SPEED -1000
// 涉及到竟态，解耦合概念

//=================== 新版：字符串命令解析 ===================//
// 例如：
// "S500" → CMD_SET_SPEED, value=500
// "F"    → CMD_FORWARD
// "R"    → CMD_REVERSE
// "X"    → CMD_STOP

// // 后续可实现比如“ls motor”“ls pid”等等


CommandMsg_t Command_ParseString(const char *cmdStr)
{
    CommandMsg_t msg = {CMD_NONE, 0};




    if (cmdStr == NULL || cmdStr[0] == '\0')
        return msg;

        // 新增在最前面

    // if (strcmp(cmdStr, "ls") == 0 || strcmp(cmdStr, "LS") == 0)
    // {
    //     msg.type = CMD_LIST_STATUS;
    //     return msg;
    // }

    if ((cmdStr[0] == 'l' || cmdStr[0] == 'L') &&
    (cmdStr[1] == 's' || cmdStr[1] == 'S'))
    {
        msg.type = CMD_LIST_STATUS;
        return msg;
    }


    char c = cmdStr[0];

    // 统一大写
    if (c >= 'a' && c <= 'z')
        c -= 32;

    switch (c)
    {
        case 'S':   // Sxxx → 设定速度
        {
            const char *numPart = &cmdStr[1];
            if (*numPart == '\0')
                return msg;  // 没数字 → 视为无效

            msg.type  = CMD_SET_SPEED;
            msg.value = atoi(numPart);
            break;
        }

        case 'F':
            msg.type = CMD_FORWARD;
            break;

        case 'R':
            msg.type = CMD_REVERSE;
            break;

        case 'X':
            msg.type = CMD_STOP;
            break;

        default:
            // 未知命令 → 保持 CMD_NONE
            break;
    }

    return msg;
}

// CommandMsg_t Command_ParseString(const char *cmdStr)
// {
//     CommandMsg_t msg = {CMD_NONE, 0};

//     if (cmdStr == NULL)
//         return msg;

//     // /* ---------- 跳过前导空格 ---------- */
//     // while (*cmdStr == ' ' || *cmdStr == '\t')
//     //     cmdStr++;

//     // if (*cmdStr == '\0')
//     //     return msg;

//     // /* ---------- ls / LS / ls xxx ---------- */
//     // if ((cmdStr[0] == 'l' || cmdStr[0] == 'L') &&
//     //     (cmdStr[1] == 's' || cmdStr[1] == 'S') &&
//     //     (cmdStr[2] == '\0' || cmdStr[2] == ' ' || cmdStr[2] == '\r' || cmdStr[2] == '\n'))
//     // {
//     //     msg.type = CMD_LIST_STATUS;
//     //     return msg;
//     // }

//     /* ---------- 单字符命令 ---------- */
//     char c = cmdStr[0];

//     /* 统一转大写 */
//     if (c >= 'a' && c <= 'z')
//         c -= 32;

//     switch (c)
//     {
//         case 'S':   // Sxxx → 设置速度
//         {
//             const char *numPart = &cmdStr[1];
//             if (*numPart == '\0')
//                 return msg;

//             msg.type  = CMD_SET_SPEED;
//             msg.value = atoi(numPart);
//             break;
//         }

//         case 'F':
//             msg.type = CMD_FORWARD;
//             break;

//         case 'R':
//             msg.type = CMD_REVERSE;
//             break;

//         case 'X':
//             msg.type = CMD_STOP;
//             break;

//         default:
//             break;
//     }

//     return msg;
// }

