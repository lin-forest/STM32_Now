// App/Src/ak09911.c

#include "ak09911.h"

/* 寄存器地址 */
#define AK09911_REG_WIA2   0x01
#define AK09911_REG_CNTL2  0x31
#define AK09911_REG_ST1    0x10
#define AK09911_REG_HXL    0x11
#define AK09911_REG_ST2    0x18

/* 工作模式：连续测量模式2 = 100Hz */
#define AK09911_MODE_CONT2 0x08

int AK09911_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t wia = 0;
    /* 读取 WIA2 验证设备 ID，应为 0x05 */
    HAL_I2C_Mem_Read(hi2c, AK09911_ADDR, AK09911_REG_WIA2,
                     I2C_MEMADD_SIZE_8BIT, &wia, 1, HAL_MAX_DELAY);
    if (wia != 0x05) return -1; /* 设备不匹配 */

    /* 先写 Power-down（0x00），再切换模式（推荐流程） */
    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(hi2c, AK09911_ADDR, AK09911_REG_CNTL2,
                      I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);
    HAL_Delay(1);

    mode = AK09911_MODE_CONT2;
    HAL_I2C_Mem_Write(hi2c, AK09911_ADDR, AK09911_REG_CNTL2,
                      I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);
    HAL_Delay(1);

    return 0;
}

int AK09911_Read(I2C_HandleTypeDef *hi2c, float *mag)
{
    uint8_t st1 = 0;
    /* 读状态寄存器，检查 DRDY bit0 */
    HAL_I2C_Mem_Read(hi2c, AK09911_ADDR, AK09911_REG_ST1,
                     I2C_MEMADD_SIZE_8BIT, &st1, 1, HAL_MAX_DELAY);
    if (!(st1 & 0x01)) return -1; /* 无新数据 */

    /* 连续读取 6 字节磁场数据 (HXL~HZH) */
    uint8_t raw[6];
    HAL_I2C_Mem_Read(hi2c, AK09911_ADDR, AK09911_REG_HXL,
                     I2C_MEMADD_SIZE_8BIT, raw, 6, HAL_MAX_DELAY);

    /* 必须读 ST2 以解锁下次采样 */
    uint8_t st2 = 0;
    HAL_I2C_Mem_Read(hi2c, AK09911_ADDR, AK09911_REG_ST2,
                     I2C_MEMADD_SIZE_8BIT, &st2, 1, HAL_MAX_DELAY);
    if (st2 & 0x08) return -1; /* HOFL: 磁场溢出，数据无效 */

    /* 组合 16 位有符号数，转换为 μT */
    int16_t hx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t hy = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t hz = (int16_t)((raw[5] << 8) | raw[4]);

    mag[0] = hx * AK09911_SENSITIVITY;
    mag[1] = hy * AK09911_SENSITIVITY;
    mag[2] = hz * AK09911_SENSITIVITY;

    return 0;
}
