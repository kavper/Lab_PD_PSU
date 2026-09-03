#include "debug_uart.h"
#include "host_link.h"
#include "ldo_link.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#define DEBUG_UART_PRINTF_BUFFER_SIZE  512U
#define DEBUG_UART_TX_RING_SIZE        4096U
#define DEBUG_UART_DMA_CHUNK_SIZE      128U

static UART_HandleTypeDef *debug_uart = NULL;
static DMA_HandleTypeDef debug_uart_tx_dma __attribute__((unused));

static uint8_t debug_tx_ring[DEBUG_UART_TX_RING_SIZE];
static uint8_t debug_tx_dma_buffer[DEBUG_UART_DMA_CHUNK_SIZE];

static volatile uint32_t debug_tx_head = 0U;
static volatile uint32_t debug_tx_tail = 0U;
static volatile uint32_t debug_tx_used = 0U;
static volatile uint32_t debug_tx_dma_len = 0U;
static volatile uint32_t debug_tx_dropped = 0U;
static volatile bool debug_tx_busy = false;
static volatile bool debug_dma_ready = false;
static volatile bool debug_blocking_ready = false;
static volatile bool debug_tx_pending_session_eol = false;
static bool debug_enabled = (DEBUG_UART_DEFAULT_ENABLED != 0U);

static void Debug_EnqueueBytes(const uint8_t *data, uint32_t length);

static void Debug_RestoreIrq(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static bool Debug_UartDmaInit(UART_HandleTypeDef *huart)
{
    /* USART2 is the G0 isolator link; host debug stays on USART1 blocking TX.
     * DMA TX for debug is intentionally disabled here. */
    (void)huart;
    return false;
}

static bool Debug_PrepareNextChunk(uint16_t *length)
{
    uint32_t primask;
    uint32_t len;
    uint32_t first;

    if (length == NULL) {
        return false;
    }

    *length = 0U;

    primask = __get_PRIMASK();
    __disable_irq();

    if (((!debug_dma_ready) && (!debug_blocking_ready)) || debug_tx_busy || (debug_tx_used == 0U)) {
        Debug_RestoreIrq(primask);
        return false;
    }

    len = debug_tx_used;
    if (len > DEBUG_UART_DMA_CHUNK_SIZE) {
        len = DEBUG_UART_DMA_CHUNK_SIZE;
    }

    first = DEBUG_UART_TX_RING_SIZE - debug_tx_tail;
    if (first > len) {
        first = len;
    }

    memcpy(debug_tx_dma_buffer, &debug_tx_ring[debug_tx_tail], first);
    if (first < len) {
        memcpy(&debug_tx_dma_buffer[first], debug_tx_ring, len - first);
    }

    debug_tx_tail = (debug_tx_tail + len) % DEBUG_UART_TX_RING_SIZE;
    debug_tx_used -= len;
    debug_tx_dma_len = len;
    debug_tx_busy = true;

    Debug_RestoreIrq(primask);

    *length = (uint16_t)len;
    return true;
}

static void Debug_StartBlockingTx(void)
{
    uint16_t length;

    if ((debug_uart == NULL) || (!debug_blocking_ready) || debug_tx_busy) {
        return;
    }

    if (!Debug_PrepareNextChunk(&length)) {
        return;
    }

    if (HAL_UART_Transmit(debug_uart, debug_tx_dma_buffer, length, 20U) != HAL_OK) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        debug_tx_busy = false;
        debug_tx_dma_len = 0U;
        debug_tx_dropped++;
        Debug_RestoreIrq(primask);
        return;
    }

    debug_tx_busy = false;
    debug_tx_dma_len = 0U;
    if (debug_tx_used == 0U) {
        debug_tx_pending_session_eol = true;
    }
    Debug_StartBlockingTx();
}

static void Debug_StartTx(void)
{
    uint16_t length;

    if (debug_uart == NULL) {
        return;
    }

    if (debug_blocking_ready) {
        Debug_StartBlockingTx();
        return;
    }

    if (!debug_dma_ready) {
        return;
    }

    if (!Debug_PrepareNextChunk(&length)) {
        return;
    }

    if (HAL_UART_Transmit_DMA(debug_uart, debug_tx_dma_buffer, length) != HAL_OK) {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        debug_tx_busy = false;
        debug_tx_dma_len = 0U;
        debug_tx_dropped++;
        Debug_RestoreIrq(primask);
    }
}

static void Debug_QueueSessionEolIfPending(void)
{
    static const uint8_t session_eol[] = "\r\n";

    if (!debug_tx_pending_session_eol) {
        return;
    }

    debug_tx_pending_session_eol = false;
    Debug_EnqueueBytes(session_eol, (uint32_t)(sizeof(session_eol) - 1U));
}

static void Debug_EnqueueBytes(const uint8_t *data, uint32_t length)
{
    uint32_t primask;
    uint32_t free_bytes;
    uint32_t first;

    if ((data == NULL) || (length == 0U)) {
        return;
    }

    if ((!debug_dma_ready) && (!debug_blocking_ready)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    free_bytes = DEBUG_UART_TX_RING_SIZE - debug_tx_used;
    if (length > free_bytes) {
        debug_tx_dropped++;
        Debug_RestoreIrq(primask);
        return;
    }

    first = DEBUG_UART_TX_RING_SIZE - debug_tx_head;
    if (first > length) {
        first = length;
    }

    memcpy(&debug_tx_ring[debug_tx_head], data, first);
    if (first < length) {
        memcpy(debug_tx_ring, &data[first], length - first);
    }

    debug_tx_head = (debug_tx_head + length) % DEBUG_UART_TX_RING_SIZE;
    debug_tx_used += length;

    Debug_RestoreIrq(primask);

    Debug_StartTx();
}

void Debug_Init(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    debug_uart = huart;
    debug_tx_head = 0U;
    debug_tx_tail = 0U;
    debug_tx_used = 0U;
    debug_tx_dma_len = 0U;
    debug_tx_dropped = 0U;
    debug_tx_busy = false;
    debug_dma_ready = false;
    debug_blocking_ready = false;
    debug_tx_pending_session_eol = false;
    debug_enabled = (DEBUG_UART_DEFAULT_ENABLED != 0U);

    Debug_RestoreIrq(primask);

    debug_dma_ready = Debug_UartDmaInit(huart);
    debug_blocking_ready = (!debug_dma_ready) && (huart != NULL);
}

void Debug_SetEnabled(bool enabled)
{
    debug_enabled = enabled;
}

bool Debug_IsEnabled(void)
{
    return debug_enabled;
}

void Debug_Write(const char *text)
{
    size_t length;

    if ((!debug_enabled) || (debug_uart == NULL) || (text == NULL)) {
        return;
    }

    length = strlen(text);

    if (length == 0U) {
        return;
    }

    Debug_QueueSessionEolIfPending();
    Debug_EnqueueBytes((const uint8_t *)text, (uint32_t)length);
}

void Debug_Printf(const char *fmt, ...)
{
    char buffer[DEBUG_UART_PRINTF_BUFFER_SIZE];
    va_list args;
    int length;
    uint32_t tx_length;

    if ((!debug_enabled) || (debug_uart == NULL) || (fmt == NULL)) {
        return;
    }

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }

    tx_length = ((uint32_t)length >= sizeof(buffer)) ?
                (sizeof(buffer) - 1U) :
                (uint32_t)length;

    if ((tx_length > 0U) && (buffer[tx_length - 1U] != '\n')) {
        if (tx_length <= (sizeof(buffer) - 3U)) {
            buffer[tx_length++] = '\r';
            buffer[tx_length++] = '\n';
        } else {
            buffer[sizeof(buffer) - 3U] = '\r';
            buffer[sizeof(buffer) - 2U] = '\n';
            tx_length = sizeof(buffer) - 1U;
        }
    }

    Debug_QueueSessionEolIfPending();
    Debug_EnqueueBytes((const uint8_t *)buffer, tx_length);
}

void Debug_BlankLine(void)
{
#if (DEBUG_LOG_NON_BQ != 0U)
    static const uint8_t blank_line[] = "\r\n";

    Debug_QueueSessionEolIfPending();
    Debug_EnqueueBytes(blank_line, (uint32_t)(sizeof(blank_line) - 1U));
#endif
}

uint32_t Debug_GetTxBufferUsed(void)
{
    uint32_t used;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    used = debug_tx_used + (debug_tx_busy ? debug_tx_dma_len : 0U);
    Debug_RestoreIrq(primask);

    return used;
}

uint32_t Debug_GetDroppedCount(void)
{
    uint32_t dropped;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    dropped = debug_tx_dropped;
    Debug_RestoreIrq(primask);

    return dropped;
}

bool Debug_IsTxBusy(void)
{
    bool busy;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    busy = debug_tx_busy;
    Debug_RestoreIrq(primask);

    return busy;
}

void Debug_UART_DMA_IRQHandler(void)
{
    if ((debug_uart != NULL) && (debug_uart->hdmatx != NULL)) {
        HAL_DMA_IRQHandler(debug_uart->hdmatx);
    }
}

void Debug_UART_IRQHandler(void)
{
    if (debug_uart != NULL) {
        HAL_UART_IRQHandler(debug_uart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart != debug_uart) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    debug_tx_busy = false;
    debug_tx_dma_len = 0U;
    if ((debug_tx_used == 0U) && (debug_tx_dma_len == 0U)) {
        debug_tx_pending_session_eol = true;
    }
    Debug_RestoreIrq(primask);

    Debug_StartTx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    HostLink_OnUartError(huart);
    LdoLink_OnUartError(huart);

    if (huart != debug_uart) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    debug_tx_busy = false;
    debug_tx_dma_len = 0U;
    debug_tx_dropped++;
    Debug_RestoreIrq(primask);

    Debug_StartTx();
}
