

#include "app_includes.h"


// void Start_Heartbeat(void)
void Heartbeat_Task(void *argument)
{
  /* USER CODE BEGIN Start_Heartbeat */
  /* Infinite loop */

  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(200);
  }
  /* USER CODE END Start_Heartbeat */
}
