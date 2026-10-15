#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define GRAYSCALE_PORT  GPIO_GRAYSCALEs_PORT
#define GRAYSCALE_NUM   8

void Grayscale_Init(void);

/**
 * @brief 读取所有灰度传感器，返回原始电平取反后的掩码
 * @return 8 位掩码，bit[n] 对应第 n 号传感器（0 起点），
 *         1 = 低电平（压线/黑色），0 = 高电平（未压线/白色）
 */
uint8_t Grayscale_ReadAll(void);

#endif
