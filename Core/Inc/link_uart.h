#ifndef LINK_UART_H
#define LINK_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Full-duplex UART: RX DMA circular + IDLE/HT/TC, parser in the task.
 * TX DMA is not aborted once started. A newer fast/slow frame replaces
 * only an unstarted slot. Safety and ACK frames stay in FIFOs.
 */

#define LINK_UART_RX_BYTES           512U
#define LINK_UART_FRAME_MAX          120U
#define LINK_UART_FIFO_DEPTH         4U
#define LINK_UART_SLOW_SLOTS         3U

typedef enum {
    LINK_UART_PRI_SAFETY = 0,
    LINK_UART_PRI_ACK,
    LINK_UART_PRI_FAST,
    LINK_UART_PRI_SLOW,
    LINK_UART_PRI_TEXT
} LinkUartPri;

typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef dma_rx;
    DMA_HandleTypeDef dma_tx;
    uint8_t rx[LINK_UART_RX_BYTES];
    uint16_t rx_tail;
    volatile uint8_t rx_kick;
    uint8_t safety[LINK_UART_FIFO_DEPTH][LINK_UART_FRAME_MAX];
    uint16_t safety_len[LINK_UART_FIFO_DEPTH];
    uint8_t safety_head;
    uint8_t safety_tail;
    uint8_t safety_count;
    uint8_t ack[LINK_UART_FIFO_DEPTH][LINK_UART_FRAME_MAX];
    uint16_t ack_len[LINK_UART_FIFO_DEPTH];
    uint8_t ack_head;
    uint8_t ack_tail;
    uint8_t ack_count;
    uint8_t fast[LINK_UART_FRAME_MAX];
    uint16_t fast_len;
    bool fast_pending;
    uint8_t slow[LINK_UART_SLOW_SLOTS][LINK_UART_FRAME_MAX];
    uint16_t slow_len[LINK_UART_SLOW_SLOTS];
    bool slow_pending[LINK_UART_SLOW_SLOTS];
    uint8_t text[LINK_UART_FIFO_DEPTH][LINK_UART_FRAME_MAX];
    uint16_t text_len[LINK_UART_FIFO_DEPTH];
    uint8_t text_head;
    uint8_t text_tail;
    uint8_t text_count;
    uint8_t tx[LINK_UART_FRAME_MAX];
    uint16_t tx_len;
    volatile bool tx_busy;
    volatile bool tx_done;
    volatile uint32_t uart_errors;
} LinkUart;

void LinkUart_Init(LinkUart *link, UART_HandleTypeDef *huart);
bool LinkUart_Submit(LinkUart *link, LinkUartPri pri, uint8_t slow_index,
                     const uint8_t *frame, uint16_t length);
void LinkUart_Poll(LinkUart *link, void (*on_byte)(uint8_t byte, void *ctx),
                   void *ctx);
void LinkUart_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void LinkUart_OnTxCplt(UART_HandleTypeDef *huart);
void LinkUart_OnError(UART_HandleTypeDef *huart);

#endif /* LINK_UART_H */
