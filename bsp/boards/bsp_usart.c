#include "bsp_usart.h"
#include "main.h"

#include "printf.h"

#include <stdarg.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/**
 * @brief 串口调试输出
 *
 * @note 注意输出不要超过 USART1_TX_BUF_LEN 个字符
 */
void usart1_printf(const char *fmt, ...)
{
    static char tx_buf[USART1_TX_BUF_LEN] = {0};        // 必须使用static
    char *ptx_buf                         = &tx_buf[0]; // 不能使用static
    static va_list args;                                // 必须使用static
    static uint16_t len;                                // 必须使用static
    va_start(args, fmt);

    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
        return;
    // return length of string
    // 返回字符串长度
    len = vsnprintf(ptx_buf, USART1_TX_BUF_LEN, fmt, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, len);
    __HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // 要改变 UART 的标志位为 READY (这可能是 HAL 库的 BUG)
        huart->gState = HAL_UART_STATE_READY;
    }
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    // enable the DMA transfer for the receiver and tramsmit request
    // 使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    // enalbe idle interrupt
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    // memory buffer 1
    // 内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // 内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // 数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    // enable double memory buffer
    // 使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}
