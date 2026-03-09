#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

// 定义环形缓冲区的大小，建议为2的幂(64, 128, 256)，这样在某些编译器下取模运算可以被优化为更快的位运算
#define UART_RX_BUFFER_SIZE 256

/**
 * @brief 环形缓冲区结构体
 * @note  head由中断服务程序写入，tail由应用程序任务读取。
 *        使用 volatile 关键字确保每次都从内存中读取，防止编译器过度优化。
 */
typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

void ring_buffer_init(RingBuffer_t *rb);
bool ring_buffer_put(RingBuffer_t *rb, uint8_t data);
bool ring_buffer_get(RingBuffer_t *rb, uint8_t *data);

#endif /* RING_BUFFER_H */
