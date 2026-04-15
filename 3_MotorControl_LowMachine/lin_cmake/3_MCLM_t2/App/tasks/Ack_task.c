#include "app_includes.h"
#include "app_config.h" // Include app_config.h for ACK_MSG_BUF_SIZE

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/**
 * @brief  ACK Task.
 * @param  argument: Not used
 * @retval None
 */
void Ack_Task(void *argument) {
    AckMsg_t ack;
    // Use ACK_MSG_BUF_SIZE from app_config.h to define buffer size
    char buf[ACK_MSG_BUF_SIZE]; 

    /* Infinite loop */
    for (;;) {
        if (osMessageQueueGet(AckQueueHandle, &ack, NULL, osWaitForever) == osOK) {
            if (ack.ok) {
                switch (ack.type) { // Changed ack.cmd to ack.type
                    case CMD_SET_SPEED:
                        // Use snprintf to prevent buffer overflow
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Set Speed OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CMD_STOP:
                        // Use snprintf to prevent buffer overflow
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Stop OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CMD_FORWARD: // Added CMD_FORWARD
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Forward OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CMD_REVERSE: // Added CMD_REVERSE
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Reverse OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CMD_LIST_STATUS: // Added CMD_LIST_STATUS
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: List Status OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CMD_LOG_START:
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Logger Enabled (UART1 DMA Stream Started)\r\n");
                        break;
                    case CMD_LOG_STOP:
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Logger Disabled\r\n");
                        break;
                    case CAN_CMD_SET_SPEED: // Add CAN_CMD_SET_SPEED handling
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: CAN Set Speed OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    case CAN_CMD_STOP: // Add CAN_CMD_STOP handling
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: CAN Stop OK, Current Speed: %hd, PWM: %hd\r\n", ack.current_logic_speed, ack.pwm_output);
                        break;
                    // You might want to add cases for CAN_CMD_SET_SPEED and CAN_CMD_STOP if they generate ACKs
                    default:
                        // Use snprintf to prevent buffer overflow
                        snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: Unknown Command Type %d OK, Current Speed: %hd, PWM: %hd\r\n", ack.type, ack.current_logic_speed, ack.pwm_output); // Changed ack.cmd to ack.type
                        break;
                }
            } else {
                // Use snprintf to prevent buffer overflow
                snprintf(buf, ACK_MSG_BUF_SIZE, "NACK: Command Type %d Failed, Current Speed: %hd, PWM: %hd\r\n", ack.type, ack.current_logic_speed, ack.pwm_output); // Changed ack.cmd to ack.type
            }
            // TODO: UART2_Print is a blocking function. Consider replacing it with a non-blocking
            // mechanism (e.g., a FreeRTOS message queue to a dedicated UART transmit task)
            // to avoid blocking the ACK task.
            Logger_Print(buf); // 将 UART2_Print 替换为 Logger_Print
        }
    }
}