#include "car_menu.h"
#include "i2c.h"
#include "oled.h"
#include "oled_canvas.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

TestHardware test_hardware;

/** 模拟 SSD1306 页/列地址及显示 RAM，真实驱动执行全部刷新逻辑。 */
typedef struct
{
	I2C_Handle handle;
	uint8_t screen[OLED_CANVAS_BYTES];
	uint16_t cursor;
	uint32_t short_calls;
	uint32_t wire_us;
	uint32_t recoveries;
	bool fail_data;
} display_mock_t;

static display_mock_t display_mock;

I2C_Handle *I2C_Init(const I2C_Config *config_p)
{
	display_mock.handle.i2c = config_p->i2c;

	return &display_mock.handle;
}

void I2C_Write(I2C_Handle *handle_p, uint8_t address,
	uint8_t *data_p, uint16_t length)
{
	(void)handle_p;
	(void)data_p;
	assert(address == 0x3CU && length == 2U);
}

void I2C_SDAUnlock(I2C_Handle *handle_p)
{
	(void)handle_p;
	display_mock.recoveries++;
}

void delay_ms(unsigned long duration)
{
	(void)duration;
}

bool I2C_TryWrite(I2C_Handle *handle_p, uint8_t address,
	const uint8_t *data_p, uint16_t length)
{
	uint8_t page = 0U;
	uint8_t column = 0U;

	(void)handle_p;
	assert(address == 0x3CU);
	assert(length >= 2U && length <= 8U);
	display_mock.short_calls++;
	display_mock.wire_us += ((length + 1U) * 9U + 2U) *
		5U / 2U;

	if (data_p[0] == 0U)
	{
		assert(length == 4U);
		page = data_p[1] & 0x0FU;
		column = (data_p[3] & 0x0FU) * 16U + data_p[2];
		assert(page < OLED_CANVAS_PAGES);
		assert(column < OLED_CANVAS_WIDTH);
		display_mock.cursor = page * OLED_CANVAS_WIDTH + column;
	}
	else
	{
		assert(data_p[0] == 0x40U);

		if (display_mock.fail_data)
		{
			return false;
		}

		assert(display_mock.cursor + length - 1U <=
			sizeof(display_mock.screen));
		assert(display_mock.cursor % OLED_CANVAS_WIDTH + length - 1U <=
			OLED_CANVAS_WIDTH);
		memcpy(&display_mock.screen[display_mock.cursor], &data_p[1],
			length - 1U);
		display_mock.cursor += length - 1U;
	}

	return true;
}

/** @brief 验证一次调用不会刷整屏，且传输最终与画布完全一致。 */
static void Test_Flush(void)
{
	uint32_t steps = 0U;
	uint32_t before = 0U;

	while (OLED_IsRefreshBusy())
	{
		before = display_mock.short_calls;
		OLED_RefreshStep();
		assert(display_mock.short_calls - before <= 2U);
		steps++;
		assert(steps < 400U);
	}

	assert(memcmp(display_mock.screen, Canvas_GetBuffer(),
		sizeof(display_mock.screen)) == 0);
}

/** @brief 保存真实画布以供 Python 导出预览。
 * @param name_p Debug 测试目录中的文件名。
 * @retval 无。
 */
static void Test_Save(const char *name_p)
{
	FILE *file_p = fopen(name_p, "wb");

	assert(file_p != NULL);
	assert(fwrite(Canvas_GetBuffer(), 1U, OLED_CANVAS_BYTES, file_p) ==
		OLED_CANVAS_BYTES);
	fclose(file_p);
}

/** @brief 验证像素裁剪、字模容错、帧冻结与首尾页增量刷新。 */
static void Test_CanvasAndDriver(void)
{
	uint32_t calls = 0U;
	uint8_t saved[OLED_CANVAS_BYTES] = {0};
	uint32_t before_us = 0U;

	Canvas_Clear();
	Canvas_Pixel(-1, -1, true);
	Canvas_Pixel(128, 64, true);
	Canvas_Pixel(0, 0, true);
	Canvas_Pixel(127, 63, true);
	assert(Canvas_GetBuffer()[0] == 1U);
	assert(Canvas_GetBuffer()[OLED_CANVAS_BYTES - 1U] == 128U);
	Canvas_Pixel(0, 0, false);
	Canvas_Text(-5, -2, "CLIP");
	Canvas_RoundFrame(-3, 60, 20, 10);
	memcpy(saved, Canvas_GetBuffer(), sizeof(saved));
	Canvas_InvertRoundBox(-3, -2, 20, 10);
	assert(memcmp(saved, Canvas_GetBuffer(), sizeof(saved)) != 0);
	Canvas_InvertRoundBox(-3, -2, 20, 10);
	assert(memcmp(saved, Canvas_GetBuffer(), sizeof(saved)) == 0);
	Canvas_Line(-4, 62, 4, 62, true);
	Canvas_Line(127, 60, 127, 63, true);
	assert((Canvas_GetBuffer()[OLED_CANVAS_BYTES - 1U] & 0xF0U) ==
		0xF0U);
	Canvas_Box(126, 62, 6, 6, false);
	assert((Canvas_GetBuffer()[OLED_CANVAS_BYTES - 1U] & 0xC0U) == 0U);
	assert(OLED_GetGlyph(0U) == OLED_GetGlyph('?'));
	assert(OLED_Present(Canvas_GetBuffer()));
	assert(!OLED_Present(Canvas_GetBuffer()));
	Test_Flush();

	calls = display_mock.short_calls;
	assert(OLED_Present(Canvas_GetBuffer()));
	Test_Flush();
	assert(display_mock.short_calls == calls);
	assert(display_mock.recoveries == 0U);

	/* 全屏变化在 400 kHz 下的模型线时应低于 35 ms。 */
	Canvas_Box(0, 0, 128, 64, true);
	before_us = display_mock.wire_us;
	assert(OLED_Present(Canvas_GetBuffer()));
	Test_Flush();
	assert(display_mock.wire_us - before_us < 35000U);
	Canvas_Dim();
	assert(Canvas_GetBuffer()[0] == 0x55U);
	assert(Canvas_GetBuffer()[1] == 0xAAU);
}

/** 以真实传输字节数推进虚拟时钟，避免零耗时 mock 掩盖动画问题。 */
typedef struct
{
	car_menu_model_t model;
	uint32_t now_us;
	uint32_t completed;
} menu_sim_t;

static menu_sim_t menu_sim;

/** @brief 运行真实菜单并保存已完成传输的帧。
 * @param duration_ms 模拟时长。
 * @param sequence_p 动画名称，NULL 不导出。
 * @retval 无。
 */
static void Test_Advance(uint32_t duration_ms, const char *sequence_p)
{
	uint32_t deadline = menu_sim.now_us + duration_ms * 1000U;
	uint32_t before = 0U;
	uint32_t steps = 0U;
	uint32_t frame = 0U;
	bool busy = false;
	char name[64] = {0};
	FILE *timeline_p = NULL;

	if (sequence_p != NULL)
	{
		snprintf(name, sizeof(name), "menu_anim_%s.csv", sequence_p);
		timeline_p = fopen(name, "w");
		assert(timeline_p != NULL);
	}

	while (menu_sim.now_us < deadline)
	{
		before = display_mock.wire_us;
		busy = OLED_IsRefreshBusy();
		Menu_Update(&menu_sim.model, menu_sim.now_us / 1000U);
		/* 400 kHz 线时之外留 40 us 主循环开销，非实机性能测量。 */
		menu_sim.now_us += display_mock.wire_us - before + 40U;

		if (busy && !OLED_IsRefreshBusy())
		{
			menu_sim.completed++;

			if (timeline_p != NULL)
			{
				snprintf(name, sizeof(name), "menu_anim_%s_%03lu.bin",
					sequence_p, (unsigned long)frame);
				Test_Save(name);
				fprintf(timeline_p, "%s,%lu\n", name,
					(unsigned long)(menu_sim.now_us / 1000U));
				frame++;
			}
		}

		steps++;
		assert(steps < 100000U);
	}

	if (timeline_p != NULL)
	{
		fclose(timeline_p);
		assert(frame >= 4U);
	}
}

/** @brief 等待动画收敛，并排空已提交帧。 @retval 无。 */
static void Test_Settle(void)
{
	uint32_t before = 0U;

	Test_Advance(1800U, NULL);
	before = display_mock.wire_us;
	Test_Flush();
	menu_sim.now_us += display_mock.wire_us - before;
	assert(!Menu_GetStatus().animating);
}

/** @brief 验证上下导航、滚动、动作隔离和过渡中间帧。 */
static void Test_Menu(void)
{
	uint8_t index = 0U;
	uint32_t previous_frames = 0U;
	uint8_t frozen[OLED_CANVAS_BYTES] = {0};

	menu_sim.model.vision.sample.sample_id = 1U;
	menu_sim.model.vision.sample.dev = -0.3f;
	menu_sim.model.vision.sample.k = 0.03f;
	menu_sim.model.vision.sample.valid = 85U;
	menu_sim.model.vision.sample.conf = 90U;
	menu_sim.model.vision.sample.seq = 42U;
	menu_sim.model.vision.accepted = 1234U;
	menu_sim.model.vision.rejected = 2U;
	menu_sim.model.race.phase = RACE_READY;
	Test_Settle();
	Test_Save("menu_root.bin");
	assert(Menu_KeyEvent(CAR_MENU_KEY_UP | CAR_MENU_KEY_DOWN) ==
		CAR_MENU_ACTION_NONE);
	assert(Menu_GetStatus().selected == 0U);
	assert(Menu_KeyEvent(CAR_MENU_KEY_DOWN) == CAR_MENU_ACTION_NONE);
	assert(Menu_GetStatus().selected == 1U);
	previous_frames = menu_sim.completed;
	Test_Advance(650U, "tile_next");
	assert(menu_sim.completed - previous_frames >= 4U);
	Test_Settle();
	assert(Menu_KeyEvent(CAR_MENU_KEY_UP) == CAR_MENU_ACTION_NONE);
	Test_Advance(650U, "tile_previous");
	Test_Settle();
	assert(Menu_GetStatus().selected == 0U);

	assert(Menu_KeyEvent(CAR_MENU_KEY_CONFIRM) == CAR_MENU_ACTION_NONE);
	assert(Menu_GetStatus().page == CAR_MENU_RUN);
	Test_Advance(800U, "open_list");
	Test_Settle();
	Test_Save("menu_run.bin");
	assert(Menu_KeyEvent(CAR_MENU_KEY_DOWN) == CAR_MENU_ACTION_NONE);
	assert(Menu_GetStatus().row == 1U);
	Test_Advance(600U, "selection");
	assert(Menu_KeyEvent(CAR_MENU_KEY_CONFIRM) == CAR_MENU_ACTION_START);
	assert(Menu_KeyEvent(CAR_MENU_KEY_DOWN) == CAR_MENU_ACTION_NONE);
	assert(Menu_KeyEvent(CAR_MENU_KEY_CONFIRM) == CAR_MENU_ACTION_STOP);
	assert(Menu_KeyEvent(CAR_MENU_KEY_CONFIRM | CAR_MENU_KEY_BACK) ==
		CAR_MENU_ACTION_STOP);
	assert(Menu_GetStatus().page == CAR_MENU_ROOT);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().row == 2U);

	for (index = 0U; index < 4U; index++)
	{
		Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	}

	assert(Menu_GetStatus().row == 6U);
	assert(Menu_GetStatus().scroll_top == 3U);
	Test_Advance(900U, "scroll");
	Test_Settle();
	Test_Save("menu_run_bottom.bin");
	Menu_KeyEvent(CAR_MENU_KEY_UP);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().popup);
	Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	assert(Menu_GetStatus().row == 5U);
	Test_Advance(800U, "popup");
	Test_Settle();
	Test_Save("menu_popup.bin");
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	Test_Settle();
	assert(!Menu_GetStatus().popup);
	assert(Menu_KeyEvent(CAR_MENU_KEY_BACK) == CAR_MENU_ACTION_STOP);
	Test_Advance(800U, "return_tiles");
	Test_Settle();

	Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().page == CAR_MENU_VISION);
	Test_Settle();
	Test_Save("menu_vision.bin");
	Menu_KeyEvent(CAR_MENU_KEY_UP);
	assert(Menu_GetStatus().row == 9U);
	assert(Menu_GetStatus().scroll_top == 6U);
	Test_Advance(800U, "wrap");
	Test_Settle();
	Test_Save("menu_vision_bottom.bin");
	Menu_ShowMessage("Vision not ready");
	Test_Settle();
	Test_Save("menu_message.bin");
	Menu_KeyEvent(CAR_MENU_KEY_BACK);
	assert(!Menu_GetStatus().popup);
	assert(Menu_GetStatus().page == CAR_MENU_VISION);
	Menu_KeyEvent(CAR_MENU_KEY_BACK);
	Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().page == CAR_MENU_PITCH);
	Test_Settle();
	Test_Save("menu_pitch_off.bin");
	menu_sim.model.pitch_available = true;
	menu_sim.model.slope.ready = true;
	menu_sim.model.slope.phase = SLOPE_UPHILL;
	menu_sim.model.slope.relative_pitch = 8.2f;
	menu_sim.model.slope.baseline_pitch = -1.5f;
	Test_Settle();
	Test_Save("menu_pitch.bin");
	Menu_KeyEvent(CAR_MENU_KEY_BACK);
	Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().page == CAR_MENU_CONTROL);
	Test_Settle();
	Test_Save("menu_control.bin");
	Menu_KeyEvent(CAR_MENU_KEY_BACK);
	Test_Settle();
	Test_Save("menu_selection.bin");

	Menu_KeyEvent(CAR_MENU_KEY_DOWN);
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	assert(Menu_GetStatus().page == CAR_MENU_RAW);
	menu_sim.model.uart_ready = true;
	menu_sim.model.uart_received = 1234U;
	menu_sim.model.vision.bytes_received = 1234U;
	menu_sim.model.vision.raw_length = VISION_RAW_BYTES;

	for (index = 0U; index < VISION_RAW_BYTES; index++)
	{
		menu_sim.model.vision.raw[index] = index;
	}

	menu_sim.model.vision.raw[0] = 0x55U;
	menu_sim.model.vision.raw[1] = 0xAAU;
	menu_sim.model.vision.raw[12] = 0x0DU;
	Test_Settle();
	Test_Save("menu_raw.bin");
	memcpy(frozen, Canvas_GetBuffer(), sizeof(frozen));
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	memset(menu_sim.model.vision.raw, 0xFF,
		sizeof(menu_sim.model.vision.raw));
	menu_sim.model.vision.bytes_received++;
	Test_Settle();
	assert(memcmp(&frozen[2U * OLED_CANVAS_WIDTH],
		&Canvas_GetBuffer()[2U * OLED_CANVAS_WIDTH],
		4U * OLED_CANVAS_WIDTH) == 0);
	Test_Save("menu_raw_hold.bin");
	Menu_KeyEvent(CAR_MENU_KEY_CONFIRM);
	Test_Settle();
	assert(memcmp(&frozen[2U * OLED_CANVAS_WIDTH],
		&Canvas_GetBuffer()[2U * OLED_CANVAS_WIDTH],
		4U * OLED_CANVAS_WIDTH) != 0);
	Menu_KeyEvent(CAR_MENU_KEY_UP);
	Test_Settle();
	Test_Save("menu_raw_info.bin");
	assert(Menu_KeyEvent(CAR_MENU_KEY_BACK) == CAR_MENU_ACTION_STOP);
	assert(Menu_GetStatus().page == CAR_MENU_ROOT);
	Test_Settle();
}

/** @brief 连续传输失败仅关闭显示，不进入旧的延时总线恢复。 */
static void Test_DisplayFailure(void)
{
	uint8_t index = 0U;

	Canvas_Clear();
	Canvas_Text(0, 0, "FAIL");
	assert(OLED_Present(Canvas_GetBuffer()));
	display_mock.fail_data = true;

	for (index = 0U; index < 5U; index++)
	{
		OLED_RefreshStep();
	}

	assert(!OLED_IsReady());
	assert(!OLED_IsRefreshBusy());
	assert(!OLED_Present(Canvas_GetBuffer()));
	assert(display_mock.recoveries == 0U);
}

int main(void)
{
	Menu_Init();
	Test_CanvasAndDriver();
	Test_Menu();
	Test_DisplayFailure();
	puts("PASS: canvas, navigation, dirty transfers "
		"and display failure isolation");

	return 0;
}
