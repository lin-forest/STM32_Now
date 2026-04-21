// App/Inc/ak09911.h

#ifndef __AK09911_H
#define __AK09911_H

#include "i2c.h"
#include <stdint.h>

/* AK09911 设备地址 (独立模式, AD0=0) */
#define AK09911_ADDR         (0x0C << 1)

/* 磁力计灵敏度：0.15 μT/LSB */
#define AK09911_SENSITIVITY  0.15f

/**
 * @brief  初始化 AK09911，进入连续测量模式2 (100Hz)
 * @param  hi2c  I2C 句柄
 * @retval 0=成功, -1=失败
 */
int AK09911_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取磁场数据
 * @param  hi2c  I2C 句柄
 * @param  mag   输出数组 [mx, my, mz]，单位 μT
 * @retval 0=新数据成功读取, -1=无新数据或失败
 */
int AK09911_Read(I2C_HandleTypeDef *hi2c, float *mag);

#endif /* __AK09911_H */
