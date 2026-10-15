#ifndef __USER_SPI_H__
#define __USER_SPI_H__

#include "ti_msp_dl_config.h"

/* ========== 类型定义 ========== */

/** SPI 初始化配置 */
typedef struct {
    SPI_Regs *spi;   // SPI 外设基址（如 SPI0）
} SPI_Config;

/** SPI 运行时句柄 */
typedef struct {
    SPI_Regs *spi;   // SPI 外设基址
} SPI_Handle;

/* ========== 通用 API ========== */

SPI_Handle* SPI_Init(const SPI_Config *config);

/**
 * @brief 单字节全双工传输，TX、RX和总线空闲等待均有次数上限
 * @param h SPI 句柄
 * @param tx_data 待发送字节
 * @retval 接收字节；超时返回0（与合法的0无法区分）
 */
uint8_t     SPI_Transfer(SPI_Handle *h, uint8_t tx_data);

#endif
