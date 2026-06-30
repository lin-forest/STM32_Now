/* ============================================================
 *  mt6701.c — MT6701 磁编码器 SPI 读取
 *  两个编码器共用 SPI1 总线，通过独立 CS 分时访问
 *  SSI 帧格式：[Angle(14bit) | Extra(2bit)] → data >> 2
 * ============================================================ */
#include "mt6701.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

#define MT6701_CS_LOW(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define MT6701_CS_HIGH(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

void MT6701_Init(void)
{
    /* SPI 已在 CubeMX 初始化，只需确保 CS 初始为 High（禁用状态） */
    MT6701_CS_HIGH(J1_CS_GPIO_Port, J1_CS_Pin);
    MT6701_CS_HIGH(J2_CS_GPIO_Port, J2_CS_Pin);
}

uint16_t MT6701_ReadRaw(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint8_t rx[2];

    MT6701_CS_LOW(cs_port, cs_pin);
    /* 注意：MOSI 必须送 0xFF，否则输出偏移 */
    HAL_SPI_Receive(&hspi1, rx, 2, 100);
    /* CS 高后等几微秒，让 MT6701 释放 MISO 总线 */
    MT6701_CS_HIGH(cs_port, cs_pin);
    for (volatile uint32_t i = 0; i < 50; i++);

    uint16_t data = ((uint16_t)rx[0] << 8) | rx[1];
    data >>= 2;                    /* 14-bit 角度在 bits 15:2 */
    return data & 0x3FFF;          /* 0~16383 对应 0~360° */
}

float MT6701_GetAngle(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint16_t raw = MT6701_ReadRaw(cs_port, cs_pin);
    return raw * 360.0f / 16384.0f;
}

/* 带偏移标定 + 过零回绕的角度读取
 *  offset_raw: 机械 0° 对应的 raw 值
 *  返回: 角度 × 10 (int16, 如 -936 = -93.6°)
 *  过零回绕: raw 跨过 16383→0 时自动修正
 */
int16_t MT6701_RawToAngleX10(uint16_t raw, uint16_t offset_raw)
{
    int32_t diff = (int32_t)raw - (int32_t)offset_raw;

    /* 回绕处理: 差超过半圈(8192)则修正 */
    if (diff >  8191) diff -= 16384;
    if (diff < -8192) diff += 16384;

    return (int16_t)(diff * 3600 / 16384);
}

int16_t MT6701_GetAngleX10(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint16_t offset_raw)
{
    return MT6701_RawToAngleX10(MT6701_ReadRaw(cs_port, cs_pin), offset_raw);
}
