/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : command.h
  * @brief          : Header for command definitions.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H
#define __COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h" // 包含main.h以获取uint32_t等类型定义

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    CMD_CAN_SET_SPEED = 0,
    CMD_CAN_STOP,
    CMD_CAN_UNKNOWN, // Add this line
} CanCmdType_t;

typedef struct {
    uint32_t can_id;
    CanCmdType_t cmd;
    int32_t value;
} CanCmd_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_H */