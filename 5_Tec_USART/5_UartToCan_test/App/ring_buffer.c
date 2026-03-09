#include "ring_buffer.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向环形缓冲区结构体的指针
 */
void ring_buffer_init(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief 向环形缓冲区中放入一个字节 (通常在ISR中调用)
 * @param rb 指向环形缓冲区结构体的指针
 * @param data 要放入的数据
 * @return bool - true: 成功, false: 缓冲区已满
 */
bool ring_buffer_put(RingBuffer_t *rb, uint8_t data)
{
    uint16_t next_head = (rb->head + 1) % UART_RX_BUFFER_SIZE;

    // 如果缓冲区满了 (head的下一个位置就是tail)，则丢弃数据
    if (next_head == rb->tail) {
        return false;
    }

    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return true;
}

/**
 * @brief 从环形缓冲区中取出一个字节 (通常在任务中调用)
 * @param rb 指向环形缓冲区结构体的指针
 * @param data 用于存放取出数据的指针
 * @return bool - true: 成功, false: 缓冲区为空
 */
bool ring_buffer_get(RingBuffer_t *rb, uint8_t *data)
{
    // 如果缓冲区是空的 (head和tail相等)
    if (rb->head == rb->tail) {
        return false;
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
    return true;
}
