#ifndef CAR_MENU_DATA_H
#define CAR_MENU_DATA_H

#include "car_menu.h"
#include <stddef.h>

/** @brief 查询列表项数。 @param page 二级页面。 @retval 行数。 */
uint8_t MenuData_Count(car_menu_page_e page);

/** @brief 查询固定行名称。
 * @param page 二级页面。
 * @param row 行号。
 * @retval 静态字符串。
 */
const char *MenuData_Label(car_menu_page_e page, uint8_t row);

/** @brief 把只读快照格式化为列表右侧的数值。
 * @param page 二级页面。
 * @param row 行号。
 * @param model_p 业务快照。
 * @param now_ms 当前时间。
 * @param text_p 输出缓冲。
 * @param capacity 输出容量。
 * @retval 无。
 */
void MenuData_Value(car_menu_page_e page, uint8_t row,
	const car_menu_model_t *model_p, uint32_t now_ms,
	char *text_p, size_t capacity);

#endif
