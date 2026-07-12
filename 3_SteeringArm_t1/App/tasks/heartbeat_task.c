/* ============================================================
 *  heartbeat_task.c — LED 心跳指示
 *  周期: 300ms
 * ============================================================ */
#include "main.h"
#include "cmsis_os.h"

void Heartbeat_Task_Impl(void *argument)
{
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(300);
    }
}
