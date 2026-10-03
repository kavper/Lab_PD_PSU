#include "link_uart.h"

#include <string.h>

static LinkUart *s_usart1;
static LinkUart *s_usart2;

static LinkUart *LinkUart_FromHandle(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance == NULL)) {
        return NULL;
    }
    if ((s_usart1 != NULL) && (huart->Instance == USART1)) {
        return s_usart1;
    }
    if ((s_usart2 != NULL) && (huart->Instance == USART2)) {
        return s_usart2;
    }
    return NULL;
}

static bool LinkUart_FifoPush(uint8_t queue[][LINK_UART_FRAME_MAX],
                              uint16_t *lengths, uint8_t *head, uint8_t *tail,
                              uint8_t *count, const uint8_t *frame,
                              uint16_t length)
{
    if (*count >= LINK_UART_FIFO_DEPTH) {
        return false;
    }
    (void)tail;
    memcpy(queue[*head], frame, length);
    lengths[*head] = length;
    *head = (uint8_t)((*head + 1U) % LINK_UART_FIFO_DEPTH);
    (*count)++;
    return true;
}

static bool LinkUart_FifoPop(uint8_t queue[][LINK_UART_FRAME_MAX],
                             uint16_t *lengths, uint8_t *head, uint8_t *tail,
                             uint8_t *count, uint8_t *dst, uint16_t *out_len)
{
    if (*count == 0U) {
        return false;
    }
    (void)head;
    memcpy(dst, queue[*tail], lengths[*tail]);
    *out_len = lengths[*tail];
    *tail = (uint8_t)((*tail + 1U) % LINK_UART_FIFO_DEPTH);
    (*count)--;
    return true;
}

static void LinkUart_StartRx(LinkUart *link)
{
    if ((link == NULL) || (link->huart == NULL)) {
        return;
    }
    (void)HAL_UART_AbortReceive(link->huart);
    link->rx_tail = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(link->huart, link->rx,
                                     LINK_UART_RX_BYTES) == HAL_OK) {
        /* HAL arms HT; keep it. IDLE is enabled by ReceiveToIdle. */
        if (link->huart->hdmarx != NULL) {
            __HAL_DMA_ENABLE_IT(link->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
        }
    }
}

static void LinkUart_PumpTx(LinkUart *link)
{
    uint16_t length = 0U;
    bool have = false;

    if ((link == NULL) || (link->huart == NULL)) {
        return;
    }
    if (link->tx_busy) {
        if (!link->tx_done) {
            return;
        }
        link->tx_done = false;
        link->tx_busy = false;
    }

    if (link->safety_count > 0U) {
        have = LinkUart_FifoPop(link->safety, link->safety_len,
                                &link->safety_head, &link->safety_tail,
                                &link->safety_count, link->tx, &length);
    } else if (link->ack_count > 0U) {
        have = LinkUart_FifoPop(link->ack, link->ack_len, &link->ack_head,
                                &link->ack_tail, &link->ack_count, link->tx,
                                &length);
    } else if (link->fast_pending) {
        memcpy(link->tx, link->fast, link->fast_len);
        length = link->fast_len;
        link->fast_pending = false;
        have = true;
    } else if (link->slow_pending[0]) {
        memcpy(link->tx, link->slow[0], link->slow_len[0]);
        length = link->slow_len[0];
        link->slow_pending[0] = false;
        have = true;
    } else if (link->slow_pending[1]) {
        memcpy(link->tx, link->slow[1], link->slow_len[1]);
        length = link->slow_len[1];
        link->slow_pending[1] = false;
        have = true;
    } else if (link->text_count > 0U) {
        have = LinkUart_FifoPop(link->text, link->text_len, &link->text_head,
                                &link->text_tail, &link->text_count, link->tx,
                                &length);
    }

    if (!have || (length == 0U)) {
        return;
    }
    link->tx_len = length;
    link->tx_done = false;
    link->tx_busy = true;
    if (HAL_UART_Transmit_DMA(link->huart, link->tx, length) != HAL_OK) {
        link->tx_busy = false;
    }
}

static void LinkUart_AttachDma(LinkUart *link, DMA_HandleTypeDef *dma,
                               DMA_Channel_TypeDef *channel, uint32_t request,
                               uint32_t direction, uint32_t mode)
{
    dma->Instance = channel;
    dma->Init.Request = request;
    dma->Init.Direction = direction;
    dma->Init.PeriphInc = DMA_PINC_DISABLE;
    dma->Init.MemInc = DMA_MINC_ENABLE;
    dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma->Init.Mode = mode;
    dma->Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(dma) != HAL_OK) {
        Error_Handler();
    }
}

void LinkUart_Init(LinkUart *link, UART_HandleTypeDef *huart)
{
    DMA_Channel_TypeDef *rx_ch;
    DMA_Channel_TypeDef *tx_ch;
    uint32_t rx_req;
    uint32_t tx_req;
    IRQn_Type rx_irq;
    IRQn_Type tx_irq;

    memset(link, 0, sizeof(*link));
    link->huart = huart;
    if (huart->Instance == USART1) {
        rx_ch = DMA1_Channel3;
        tx_ch = DMA1_Channel4;
        rx_req = DMA_REQUEST_USART1_RX;
        tx_req = DMA_REQUEST_USART1_TX;
        rx_irq = DMA1_Channel3_IRQn;
        tx_irq = DMA1_Channel4_IRQn;
        s_usart1 = link;
    } else if (huart->Instance == USART2) {
        rx_ch = DMA1_Channel5;
        tx_ch = DMA1_Channel6;
        rx_req = DMA_REQUEST_USART2_RX;
        tx_req = DMA_REQUEST_USART2_TX;
        rx_irq = DMA1_Channel5_IRQn;
        tx_irq = DMA1_Channel6_IRQn;
        s_usart2 = link;
    } else {
        return;
    }

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    LinkUart_AttachDma(link, &link->dma_rx, rx_ch, rx_req,
                       DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR);
    __HAL_LINKDMA(huart, hdmarx, link->dma_rx);
    LinkUart_AttachDma(link, &link->dma_tx, tx_ch, tx_req,
                       DMA_MEMORY_TO_PERIPH, DMA_NORMAL);
    __HAL_LINKDMA(huart, hdmatx, link->dma_tx);

    HAL_NVIC_SetPriority(rx_irq, 2, 0);
    HAL_NVIC_EnableIRQ(rx_irq);
    HAL_NVIC_SetPriority(tx_irq, 2, 0);
    HAL_NVIC_EnableIRQ(tx_irq);
    LinkUart_StartRx(link);
}

bool LinkUart_Submit(LinkUart *link, LinkUartPri pri, uint8_t slow_index,
                     const uint8_t *frame, uint16_t length)
{
    if ((link == NULL) || (frame == NULL) || (length == 0U) ||
        (length > LINK_UART_FRAME_MAX)) {
        return false;
    }
    if (pri == LINK_UART_PRI_SAFETY) {
        return LinkUart_FifoPush(link->safety, link->safety_len,
                                 &link->safety_head, &link->safety_tail,
                                 &link->safety_count, frame, length);
    }
    if (pri == LINK_UART_PRI_ACK) {
        return LinkUart_FifoPush(link->ack, link->ack_len, &link->ack_head,
                                 &link->ack_tail, &link->ack_count, frame,
                                 length);
    }
    if (pri == LINK_UART_PRI_TEXT) {
        return LinkUart_FifoPush(link->text, link->text_len, &link->text_head,
                                 &link->text_tail, &link->text_count, frame,
                                 length);
    }
    if (pri == LINK_UART_PRI_FAST) {
        memcpy(link->fast, frame, length);
        link->fast_len = length;
        link->fast_pending = true;
        return true;
    }
    if (slow_index > 1U) {
        return false;
    }
    memcpy(link->slow[slow_index], frame, length);
    link->slow_len[slow_index] = length;
    link->slow_pending[slow_index] = true;
    return true;
}

void LinkUart_Poll(LinkUart *link, void (*on_byte)(uint8_t byte, void *ctx),
                   void *ctx)
{
    uint16_t guard = 0U;

    if ((link == NULL) || (link->huart == NULL) ||
        (link->huart->hdmarx == NULL) || (on_byte == NULL)) {
        return;
    }
    while (guard < LINK_UART_RX_BYTES) {
        uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(link->huart->hdmarx);
        uint16_t pos;

        if (remaining > LINK_UART_RX_BYTES) {
            remaining = LINK_UART_RX_BYTES;
        }
        pos = (uint16_t)(LINK_UART_RX_BYTES - remaining);
        if (link->rx_tail == pos) {
            break;
        }
        on_byte(link->rx[link->rx_tail], ctx);
        link->rx_tail = (uint16_t)((link->rx_tail + 1U) % LINK_UART_RX_BYTES);
        guard++;
    }
    link->rx_kick = 0U;
    LinkUart_PumpTx(link);
}

void LinkUart_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    LinkUart *link = LinkUart_FromHandle(huart);

    (void)size;
    if (link != NULL) {
        link->rx_kick = 1U;
    }
}

void LinkUart_OnTxCplt(UART_HandleTypeDef *huart)
{
    LinkUart *link = LinkUart_FromHandle(huart);

    if (link != NULL) {
        link->tx_done = true;
    }
}

void LinkUart_OnError(UART_HandleTypeDef *huart)
{
    LinkUart *link = LinkUart_FromHandle(huart);

    if (link == NULL) {
        return;
    }
    link->uart_errors++;
    /* A started TX DMA runs to completion. Only a dead TX may be reused. */
    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        link->tx_busy = false;
        link->tx_done = false;
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    LinkUart_StartRx(link);
}

void DMA1_Channel3_IRQHandler(void)
{
    if ((s_usart1 != NULL) && (s_usart1->dma_rx.Instance == DMA1_Channel3)) {
        HAL_DMA_IRQHandler(&s_usart1->dma_rx);
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    if ((s_usart1 != NULL) && (s_usart1->dma_tx.Instance == DMA1_Channel4)) {
        HAL_DMA_IRQHandler(&s_usart1->dma_tx);
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if ((s_usart2 != NULL) && (s_usart2->dma_rx.Instance == DMA1_Channel5)) {
        HAL_DMA_IRQHandler(&s_usart2->dma_rx);
    }
}

void DMA1_Channel6_IRQHandler(void)
{
    if ((s_usart2 != NULL) && (s_usart2->dma_tx.Instance == DMA1_Channel6)) {
        HAL_DMA_IRQHandler(&s_usart2->dma_tx);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    LinkUart_OnRxEvent(huart, Size);
}
