#ifndef OLED_TEST_H
#define OLED_TEST_H

/** @brief 仅初始化 OLED，进入强制全亮测试。 @retval 无。 */
void OLED_TestInit(void);

/** @brief 主循环轮询全亮、全灭和文字计数测试。 @retval 无。 */
void OLED_TestUpdate(void);

#endif
