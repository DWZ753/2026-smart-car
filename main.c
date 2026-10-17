/** @file main.c
 * @brief 硬件初始化及分级裸机应用入口。
 */
#include "board.h"
#include "car_app.h"

/** @brief 主程序。 @retval 不返回。 */
int main(void)
{
	/* Board_Init 是唯一的 SYSCFG_DL_init 调用入口。 */
	Board_Init();
	Car_Init();

	while (1)
	{
		Car_Update();
	}
}
