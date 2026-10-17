#include "oled_test.h"
#include "board.h"
#include "oled.h"

/** 纯屏幕测试状态，不接入菜单、视觉、IMU 或执行器。 */
typedef struct
{
	uint32_t phase_ms;
	uint32_t count_ms;
	uint32_t count;
	uint8_t phase;
} oled_test_state_t;

static oled_test_state_t oled_test_state;

void OLED_TestInit(void)
{
	OLED_Init();
	/* A5 强制全部像素亮，不依赖字库、画布和显存内容。 */
	OLED_WriteByte(0xA5U, OLED_CMD);
	oled_test_state.phase_ms = Board_GetTickMs();
}

void OLED_TestUpdate(void)
{
	uint32_t now_ms = Board_GetTickMs();
	uint32_t elapsed_ms = now_ms - oled_test_state.phase_ms;
	static const uint8_t title[] = "OLED TEST";
	static const uint8_t wiring[] = "I2C0 ADDR 0x3C";
	static const uint8_t counter[] = "COUNT:";

	if (oled_test_state.phase == 0U && elapsed_ms >= 2000U)
	{
		OLED_WriteByte(0xA4U, OLED_CMD);
		OLED_Clear();
		oled_test_state.phase = 1U;
		oled_test_state.phase_ms = Board_GetTickMs();
	}
	else if (oled_test_state.phase == 1U && elapsed_ms >= 1000U)
	{
		OLED_ShowString(0U, 0U, (uint8_t *)title, 16U);
		OLED_ShowString(0U, 2U, (uint8_t *)wiring, 8U);
		OLED_ShowString(0U, 4U, (uint8_t *)counter, 16U);
		OLED_ShowNum(48U, 4U, 0U, 4U, 16U);
		oled_test_state.phase = 2U;
		oled_test_state.count_ms = Board_GetTickMs();
	}
	else if (oled_test_state.phase == 2U &&
		(uint32_t)(now_ms - oled_test_state.count_ms) >= 1000U)
	{
		oled_test_state.count = (oled_test_state.count + 1U) % 10000U;
		OLED_ShowNum(48U, 4U, oled_test_state.count, 4U, 16U);
		oled_test_state.count_ms = Board_GetTickMs();
	}
}
