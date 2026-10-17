#include "uart.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>

#define UART_MAX_INSTANCES 4
#define UART_DMA_TIMEOUT_CNT 1000000U
#define UART_RX_IRQ_BUDGET 16U

/* ========== 句柄注册表 ========== */
static UART_Handle g_handle_pool[UART_MAX_INSTANCES];
static UART_Handle *g_uart_instances[UART_MAX_INSTANCES];
static uint8_t g_uart_count;

/**
 * @brief 根据 UART 外设基址确定硬件索引
 * @param uart UART 外设基址指针
 * @return 硬件索引 0~3，未匹配返回 UART_MAX_INSTANCES
 */
static uint8_t get_uart_index(UART_Regs *uart)
{
    if (uart == UART0) return 0;
    if (uart == UART1) return 1;
    if (uart == UART2) return 2;
    if (uart == UART3) return 3;
    return UART_MAX_INSTANCES;
}

/**
 * @brief 等待 DMA 释放发送缓冲区，超时不修改正在发送的数据
 * @param h UART 句柄
 * @retval true 已完成
 * @retval false 等待超时
 */
static bool UART_WaitDMA(UART_Handle *h)
{
	uint32_t timeout = UART_DMA_TIMEOUT_CNT;

	while (!h->txDMADone)
	{
		if (--timeout == 0)
		{
			return false;
		}
	}

	return true;
}

/**
 * @brief 等待发送 FIFO 可写并发送一个字节
 * @param h UART 句柄
 * @param data 待发送字节
 * @retval true 已写入 FIFO
 * @retval false 等待超时
 */
static bool UART_SendByteChecked(UART_Handle *h, uint8_t data)
{
	uint32_t timeout = UART_BUSY_TIMEOUT;

	while (DL_UART_isTXFIFOFull(h->uart))
	{
		if (--timeout == 0)
		{
			return false;
		}
	}

	DL_UART_transmitData(h->uart, data);

	return true;
}

/* ========== 通用 API ========== */

UART_Handle* UART_Init(const UART_Config *config)
{
    uint8_t idx;
    uint16_t i;

    if (config == NULL || config->uart == NULL) return NULL;
    idx = get_uart_index(config->uart);
    if (idx >= UART_MAX_INSTANCES) return NULL;
    if (g_uart_instances[idx] != NULL) return NULL;
    if (g_uart_count >= UART_MAX_INSTANCES) return NULL;

    UART_Handle *h = &g_handle_pool[g_uart_count];

    h->uart         = config->uart;
    h->irqNum       = config->irqNum;
    h->dma          = DMA;
    h->dmaTxChanId  = config->dmaTxChanId;
    h->dmaTxTrigger = config->dmaTxTrigger;

    h->txDMADone = 1;
    h->rxDone    = 0;
    h->rxPos     = 0;
    h->rxLen     = 0;
    h->rxOvf     = 0;
    for (i = 0; i < UART_RX_BUF_SIZE; i++)
    {
        h->rxBuf[i] = 0;
    }

    /* 原始模式默认关闭 */
    h->rxMode     = UART_RX_MODE_LINE;
    h->rxRawHead  = 0;
    h->rxRawTail  = 0;
    for (i = 0; i < UART_RX_BUF_SIZE; i++)
    {
        h->rxRawBuf[i] = 0;
    }

    g_uart_instances[idx] = h;
    g_uart_count++;

    NVIC_ClearPendingIRQ(config->irqNum);
    NVIC_EnableIRQ(config->irqNum);

    return h;
}

/* ======================================================================== */
/*  字符串 TX                                                               */
/* ======================================================================== */

int UART_SendStr(UART_Handle *h, const char *str)
{
    int cnt = 0;
    while (*str)
    {
        if (!UART_SendByteChecked(h, (uint8_t)*str))
        {
            break;
        }

        str++;
        cnt++;
    }
    return cnt;
}

int UART_Printf(UART_Handle *h, char *fmt, ...)
{
    static char buf[UART_TX_BUF_SIZE];
    int len;
    va_list args;
    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0)
    {
        return len;
    }

    return UART_SendStr(h, buf);
}

void UART_SendStrDMA(UART_Handle *h, const char *str, uint16_t len)
{
    if (!UART_WaitDMA(h) || len == 0)
    {
        return;
    }

    h->txDMADone = 0;
    DL_DMA_setSrcAddr(h->dma, h->dmaTxChanId, (uint32_t)str);
    DL_DMA_setDestAddr(h->dma, h->dmaTxChanId, (uint32_t)(&h->uart->TXDATA));
    DL_DMA_setTransferSize(h->dma, h->dmaTxChanId, len);
    DL_DMA_enableChannel(h->dma, h->dmaTxChanId);
}

void UART_PrintfDMA(UART_Handle *h, char *fmt, ...)
{
    static char buf[UART_TX_BUF_SIZE];
    int len;
    va_list args;

    if (!UART_WaitDMA(h))
    {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if ((size_t)len >= sizeof(buf))
    {
        len = sizeof(buf) - 1;
    }

    UART_SendStrDMA(h, buf, (uint16_t)len);
}

/* ======================================================================== */
/*  二进制 TX                                                               */
/* ======================================================================== */

void UART_SendByte(UART_Handle *h, uint8_t data)
{
    (void)UART_SendByteChecked(h, data);
}

void UART_SendData(UART_Handle *h, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        if (!UART_SendByteChecked(h, data[i]))
        {
            return;
        }
    }
}

/* ======================================================================== */
/*  行模式 RX                                                               */
/* ======================================================================== */

void UART_StartReceive(UART_Handle *h)
{
    DL_UART_disableInterrupt(h->uart, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    h->rxMode = UART_RX_MODE_LINE;
    h->rxPos  = 0;
    h->rxDone = 0;
}

/* ======================================================================== */
/*  原始模式 RX                                                             */
/* ======================================================================== */

void UART_StartReceiveRaw(UART_Handle *h)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
    h->rxMode     = UART_RX_MODE_RAW;
    h->rxRawHead  = 0;
    h->rxRawTail  = 0;
    h->rxDone     = 0;
	h->rx_status = (uart_rx_status_t){0};
	DL_UART_enableFIFOs(h->uart);
	DL_UART_setRXFIFOThreshold(h->uart, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_setRXInterruptTimeout(h->uart, 4U);
	DL_UART_enableInterrupt(h->uart, DL_UART_INTERRUPT_RX |
		DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
	__set_PRIMASK(primask);
}

uart_rx_status_t UART_GetRxStatus(const UART_Handle *h_p)
{
	uint32_t primask = __get_PRIMASK();
	uart_rx_status_t status = {0};

	__disable_irq();
	status = h_p->rx_status;
	__set_PRIMASK(primask);

	return status;
}

uint16_t UART_RawRxAvailable(UART_Handle *h)
{
    uint16_t head = h->rxRawHead;
    uint16_t tail = h->rxRawTail;

    if (h->rxMode != UART_RX_MODE_RAW) return 0;
    if (head >= tail)
    {
        return head - tail;
    } else
    {
        return UART_RX_BUF_SIZE - tail + head;
    }
}

uint8_t UART_ReadRawByte(UART_Handle *h)
{
    uint8_t data = 0;

    if (h->rxMode != UART_RX_MODE_RAW) return 0;
    if (UART_RawRxAvailable(h) == 0) return 0;

    data = h->rxRawBuf[h->rxRawTail];
    ++h->rxRawTail;
    if (h->rxRawTail >= UART_RX_BUF_SIZE)
    {
        h->rxRawTail = 0;
    }
    return data;
}

void UART_FlushRawRx(UART_Handle *h)
{
    h->rxRawHead = 0;
    h->rxRawTail = 0;
}

/* ======================================================================== */
/*  通用中断处理                                                            */
/* ======================================================================== */

void UART_HandleIRQ(UART_Handle *h)
{
    uint16_t next_head;
    uint8_t byte;
	uint32_t raw = 0U;
	uint8_t budget = UART_RX_IRQ_BUDGET;

    switch (DL_UART_getPendingInterrupt(h->uart))
    {
        case DL_UART_IIDX_DMA_DONE_TX:
            h->txDMADone = 1;
            break;
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            if (h->rxMode == UART_RX_MODE_RAW)
            {
				while (budget > 0U && !DL_UART_isRXFIFOEmpty(h->uart))
				{
					/* RXDATA 读一次，同时取得字节及错误位，避免重复出队。 */
					raw = h->uart->RXDATA;
					h->rx_status.received++;
					h->rx_status.overruns +=
						(raw & DL_UART_ERROR_OVERRUN) != 0U;
					h->rx_status.errors += (raw & (DL_UART_ERROR_FRAMING |
						DL_UART_ERROR_PARITY | DL_UART_ERROR_BREAK)) != 0U;
					next_head = (uint16_t)((h->rxRawHead + 1U) %
						UART_RX_BUF_SIZE);

					if (next_head == h->rxRawTail)
					{
						/* 满时丢弃新字节，ISR 不修改消费者的 tail。 */
						h->rx_status.dropped++;
					}
					else
					{
						h->rxRawBuf[h->rxRawHead] = (uint8_t)raw;
						h->rxRawHead = next_head;
					}

					budget--;
				}
            } else
            {
                /* 行模式：检测结束符 */
                if (!h->rxDone)
                {
                    byte = DL_UART_receiveData(h->uart);

                    if (byte == UART_RX_TERMINATOR)
                    {
                        if (h->rxPos < sizeof(h->rxBuf))
                        {
                            h->rxBuf[h->rxPos] = '\0';
                            h->rxLen = h->rxPos;
                            h->rxDone = 1;
                        }
                        else
                        {
                            /* 丢弃超长行，下一行重新接收。 */
                            h->rxPos = 0;
                            h->rxLen = 0;
                            h->rxBuf[0] = '\0';
                        }
                    }
                    else if (h->rxPos < sizeof(h->rxBuf) - 1)
                    {
                        h->rxBuf[h->rxPos++] = byte;
                    }
                    else
                    {
                        h->rxOvf = byte;
                        h->rxPos = sizeof(h->rxBuf);
                    }
                } else
                {
                    h->rxOvf = DL_UART_receiveData(h->uart);
                }
            }
            break;
        default:
            break;
    }
}

/* ========== ISR 入口 ========== */

void UART0_IRQHandler(void)
{
    if (g_uart_instances[0] != NULL)
    {
        UART_HandleIRQ(g_uart_instances[0]);
    }
}

void UART1_IRQHandler(void)
{
    if (g_uart_instances[1] != NULL)
    {
        UART_HandleIRQ(g_uart_instances[1]);
    }
}

void UART2_IRQHandler(void)
{
    if (g_uart_instances[2] != NULL)
    {
        UART_HandleIRQ(g_uart_instances[2]);
    }
}

void UART3_IRQHandler(void)
{
    if (g_uart_instances[3] != NULL)
    {
        UART_HandleIRQ(g_uart_instances[3]);
    }
}
