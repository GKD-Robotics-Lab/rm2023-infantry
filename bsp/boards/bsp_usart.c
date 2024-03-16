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
 * @brief 涓插彛璋冭瘯杈撳嚭
 *
 * @note 娉ㄦ剰杈撳嚭涓嶈�佽秴杩� USART1_TX_BUF_LEN 涓�瀛楃��
 */
void usart1_printf(const char *fmt, ...)
{
    static char tx_buf[USART1_TX_BUF_LEN] = {0};        // 蹇呴』浣跨敤static
    char *ptx_buf                         = &tx_buf[0]; // 涓嶈兘浣跨敤static
    static va_list args;                                // 蹇呴』浣跨敤static
    static uint16_t len;                                // 蹇呴』浣跨敤static
    va_start(args, fmt);

    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
        return;
    // return length of string
    // 杩斿洖瀛楃�︿覆闀垮害
    len = vsnprintf(ptx_buf, USART1_TX_BUF_LEN, fmt, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, len);
    __HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // 瑕佹敼鍙� UART 鐨勬爣蹇椾綅涓� READY (杩欏彲鑳芥槸 HAL 搴撶殑 BUG)
        huart->gState = HAL_UART_STATE_READY;
    }
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    // enable the DMA transfer for the receiver and tramsmit request
    // 浣胯兘DMA涓插彛鎺ユ敹鍜屽彂閫�
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    // enalbe idle interrupt
    // 浣胯兘绌洪棽涓�鏂�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // disable DMA
    // 澶辨晥DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    // memory buffer 1
    // 鍐呭瓨缂撳啿鍖�1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // memory buffer 2
    // 鍐呭瓨缂撳啿鍖�2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // data length
    // 鏁版嵁闀垮害
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    // enable double memory buffer
    // 浣胯兘鍙岀紦鍐插尯
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    // enable DMA
    // 浣胯兘DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    // disable DMA
    // 澶辨晥DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    // disable DMA
    // 澶辨晥DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}
