#ifndef __ECHO_SERVICE_H__
#define __ECHO_SERVICE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 函数声明 ===================== */

/**
 * @brief  初始化串口回显服务
 * @note   启动 USART1 中断接收，收到字节后原路返回
 */
void Echo_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ECHO_SERVICE_H__ */
