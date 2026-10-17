/** @file main.c
 * @brief 硬件初始化及分级裸机应用入口。
 */
#include "board.h"
#include "car_config.h"

#if (CAR_OLED_TEST == 1U)
#include "oled_test.h"
#else
#include "car_app.h"
#endif

/** @brief 主程序。 @retval 不返回。 */
int main(void)
{
	/* Board_Init 是唯一的 SYSCFG_DL_init 调用入口。 */
	Board_Init();
#if (CAR_OLED_TEST == 1U)
	OLED_TestInit();
#else
	Car_Init();
#endif

	while (1)
	{
#if (CAR_OLED_TEST == 1U)
		OLED_TestUpdate();
#else
		Car_Update();
#endif
	}
}
