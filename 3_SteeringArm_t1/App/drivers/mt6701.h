/* ============================================================
 *  mt6701.h — MT6701 磁编码器 SPI 驱动
 *  配置：SPI1 Mode 1, Prescaler 64, CS 软件控制
 *  参考：6_mt6701_spi/doc/build_config_260602.md
 * ============================================================ */
#ifndef __MT6701_H__
#define __MT6701_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"

void    MT6701_Init(void);
uint16_t MT6701_ReadRaw(GPIO_TypeDef *cs_port, uint16_t cs_pin);
float   MT6701_GetAngle(GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif
