#ifndef __APP_TASK_H
#define __APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h" // For Error_Handler and extern queue handles
#include "command.h" // For CanCmd_t and CanCmdType_t
#include "can.h" // For hcan1

/* Exported functions prototypes ---------------------------------------------*/
void App_Tasks_Init(void);
void UartRxTask(void *argument);
void CanTxTask(void *argument);
void UartRxParser(const char *cmd_str);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */