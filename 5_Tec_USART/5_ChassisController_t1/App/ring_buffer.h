#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

// 缓冲区大小必须固定为 256，使 uint8_t 的自然溢出回绕等价于取模运算
// 这样 head/tail 的读写均为单字节操作，在 ARM Cortex-M3 上天然原子，无需临界区
#define UART_RX_BUFFER_SIZE 256

/**
 * @brief 环形缓冲区结构体
 * @note  head 由 ISR 写入，tail 由任务读取。
 *        uint8_t 保证单字节原子读写；volatile 防止编译器缓存寄存器值。
 *        缓冲区大小必须为 256，否则溢出回绕不等价于取模。
 */
typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} RingBuffer_t;

void ring_buffer_init(RingBuffer_t *rb);
bool ring_buffer_put(RingBuffer_t *rb, uint8_t data);
bool ring_buffer_get(RingBuffer_t *rb, uint8_t *data);

#endif /* RING_BUFFER_H */
