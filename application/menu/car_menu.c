#include "car_menu.h"
#include "car_menu_data.h"
#include "oled.h"
#include "oled_canvas.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MENU_ITEMS 5U
#define MENU_TICK_MS 16U
#define MENU_DATA_MS 100U
#define MENU_TILE_SPACING 42
#define MENU_ROW_HEIGHT 16
#define MENU_VISIBLE_ROWS 4U

/** 镜头、选择框和前景分别运动；各页保留选中行及滚动位置。 */
typedef struct
{
	car_menu_status_t navigation;
	uint8_t rows[MENU_ITEMS];
	uint8_t tops[MENU_ITEMS];
	car_menu_page_e popup_page;
	uint8_t popup_row;
	uint32_t last_tick_ms;
	uint32_t last_frame_ms;
	float camera_x;
	float camera_y;
	float selector_x;
	float selector_y;
	float selector_width;
	float selector_height;
	float unfold;
	float bar;
	float label_y;
	float popup_y;
	char message[21];
	bool popup;
	bool dirty;
	bool animating;
	uint8_t raw[VISION_RAW_BYTES];
	uint8_t raw_length;
	bool raw_hold;
} menu_state_t;

static menu_state_t menu_state;
static const char *const menu_titles[MENU_ITEMS] =
{
	"Run status", "Vision", "IMU pitch", "Control", "UART raw"
};

/** @brief 按毫秒趋近目标，独立于 I2C 刷新调用次数。
 * @param value_p 当前坐标。
 * @param target 目标坐标。
 * @param tau_ms 时间常数，越大越迟缓。
 * @param elapsed_ms 本次动画步长。
 * @retval true 坐标发生改变。
 */
static bool Menu_Move(float *value_p, float target, float tau_ms,
	uint32_t elapsed_ms)
{
	float previous = *value_p;

	*value_p += (target - *value_p) *
		(float)elapsed_ms / (tau_ms + (float)elapsed_ms);

	if (fabsf(target - *value_p) < 0.2f)
	{
		*value_p = target;
	}

	return previous != *value_p;
}

/** @brief 切换页面，只改变目标与展开进度，不重置选择框。
 * @param page 目标页面。
 * @retval 无。
 */
static void Menu_Open(car_menu_page_e page)
{
	menu_state.navigation.page = page;
	menu_state.unfold = 0.0f;
	menu_state.label_y = 64.0f;
	menu_state.popup = false;
	menu_state.message[0] = '\0';
	menu_state.dirty = true;
}

/** @brief 更新镜头、选择框形状与页面展开。
 * @param elapsed_ms 步长。
 * @retval 无。
 */
static void Menu_Animate(uint32_t elapsed_ms)
{
	car_menu_page_e page = menu_state.navigation.page;
	uint8_t row = page == CAR_MENU_ROOT ? 0U : menu_state.rows[page - 1U];
	float x = 0.0f;
	float y = 0.0f;
	float width = 0.0f;
	float height = 0.0f;
	float bar = 0.0f;
	bool changed = false;

	changed |= Menu_Move(&menu_state.unfold, 100.0f, 100.0f, elapsed_ms);

	if (page == CAR_MENU_ROOT)
	{
		changed |= Menu_Move(&menu_state.camera_x,
			-(float)menu_state.navigation.selected * MENU_TILE_SPACING,
			65.0f, elapsed_ms);
		x = 46.0f + menu_state.navigation.selected * MENU_TILE_SPACING +
			menu_state.camera_x;
		y = 7.0f;
		width = 36.0f;
		height = 36.0f;
		bar = (menu_state.navigation.selected + 1U) *
			128.0f / MENU_ITEMS;
		changed |= Menu_Move(&menu_state.label_y, 54.0f,
			85.0f, elapsed_ms);
	}
	else if (page == CAR_MENU_RAW)
	{
		width = 128.0f;
		height = 12.0f;
	}
	else
	{
		changed |= Menu_Move(&menu_state.camera_y,
			-(float)menu_state.tops[page - 1U] * MENU_ROW_HEIGHT,
			65.0f, elapsed_ms);
		y = row * MENU_ROW_HEIGHT + menu_state.camera_y;
		width = (float)strlen(MenuData_Label(page, row)) * 6.0f + 8.0f;
		height = 15.0f;
		bar = (row + 1U) * 64.0f / MenuData_Count(page);
	}

	changed |= Menu_Move(&menu_state.selector_x, x, 90.0f, elapsed_ms);
	changed |= Menu_Move(&menu_state.selector_y, y, 75.0f, elapsed_ms);
	changed |= Menu_Move(&menu_state.selector_width, width,
		110.0f, elapsed_ms);
	changed |= Menu_Move(&menu_state.selector_height, height,
		125.0f, elapsed_ms);
	changed |= Menu_Move(&menu_state.bar, bar, 85.0f, elapsed_ms);
	changed |= Menu_Move(&menu_state.popup_y,
		menu_state.popup ? 18.0f : -40.0f, 75.0f, elapsed_ms);
	menu_state.animating = changed;
	menu_state.dirty |= changed;
}

/** @brief 绘制自有磁贴图标。
 * @param x 左边。
 * @param y 顶部。
 * @param index 图标索引。
 * @retval 无。
 */
static void Menu_Icon(int16_t x, int16_t y, uint8_t index)
{
	int16_t slider = 0;
	int16_t knob_y = 0;

	Canvas_Box(x, y, 30, 30, true);

	if (index == 0U)
	{
		Canvas_Box(x + 9, y + 4, 12, 22, false);
		Canvas_Box(x + 10, y + 5, 10, 20, true);
		Canvas_Box(x + 11, y + 9, 8, 5, false);
		Canvas_Box(x + 11, y + 19, 8, 3, false);
		Canvas_Box(x + 6, y + 7, 3, 6, false);
		Canvas_Box(x + 21, y + 7, 3, 6, false);
		Canvas_Box(x + 6, y + 18, 3, 6, false);
		Canvas_Box(x + 21, y + 18, 3, 6, false);
	}
	else if (index == 1U)
	{
		Canvas_Line(x + 4, y + 25, x + 12, y + 5, false);
		Canvas_Line(x + 12, y + 5, x + 18, y + 5, false);
		Canvas_Line(x + 18, y + 5, x + 26, y + 25, false);
		Canvas_Line(x + 15, y + 7, x + 15, y + 10, false);
		Canvas_Line(x + 15, y + 13, x + 15, y + 17, false);
		Canvas_Line(x + 15, y + 21, x + 15, y + 25, false);
	}
	else if (index == 2U)
	{
		Canvas_Line(x + 4, y + 24, x + 13, y + 10, false);
		Canvas_Line(x + 13, y + 10, x + 18, y + 10, false);
		Canvas_Line(x + 18, y + 10, x + 26, y + 24, false);
		Canvas_Line(x + 4, y + 26, x + 26, y + 26, false);
		Canvas_Line(x + 7, y + 13, x + 15, y + 4, false);
		Canvas_Line(x + 15, y + 4, x + 15, y + 8, false);
		Canvas_Line(x + 15, y + 4, x + 11, y + 4, false);
	}
	else if (index == 3U)
	{
		for (slider = 0; slider < 3; slider++)
		{
			knob_y = slider == 1 ? 19 : 9;
			Canvas_Line(x + 7 + slider * 8, y + 4,
				x + 7 + slider * 8, y + 26, false);
			Canvas_Box(x + 5 + slider * 8, y + knob_y, 5, 5, false);
		}
	}
	else
	{
		Canvas_Line(x + 3, y + 21, x + 8, y + 21, false);
		Canvas_Line(x + 8, y + 21, x + 8, y + 8, false);
		Canvas_Line(x + 8, y + 8, x + 15, y + 8, false);
		Canvas_Line(x + 15, y + 8, x + 15, y + 21, false);
		Canvas_Line(x + 15, y + 21, x + 22, y + 21, false);
		Canvas_Line(x + 22, y + 21, x + 22, y + 8, false);
		Canvas_Line(x + 22, y + 8, x + 27, y + 8, false);
	}
}



/** @brief 绘制图标页，图标、镜头、选择器保持不同的运动速度。
 * @retval 无。
 */
static void Menu_Tiles(void)
{
	uint8_t index = 0U;
	int16_t x = 0;
	int16_t y = (int16_t)roundf(menu_state.selector_y);
	int16_t width = (int16_t)roundf(menu_state.selector_width);
	int16_t height = (int16_t)roundf(menu_state.selector_height);
	int16_t side = 0;
	const char *title_p = menu_titles[menu_state.navigation.selected];

	Canvas_Box(0, 0, (int16_t)roundf(menu_state.bar), 2, true);

	for (index = 0U; index < MENU_ITEMS; index++)
	{
		x = (int16_t)roundf(49.0f + menu_state.camera_x +
			menu_state.navigation.selected * MENU_TILE_SPACING +
			((float)index - menu_state.navigation.selected) *
			MENU_TILE_SPACING * menu_state.unfold / 100.0f);

		if (x > -30 && x < (int16_t)OLED_CANVAS_WIDTH)
		{
			Menu_Icon(x, 10, index);
		}
	}

	x = (int16_t)roundf(menu_state.selector_x);

	for (side = 0; side < 2; side++)
	{
		Canvas_Line(x, y + side * height, x + 6, y + side * height, true);
		Canvas_Line(x + width - 6, y + side * height,
			x + width, y + side * height, true);
		Canvas_Line(x + side * width, y, x + side * width, y + 6, true);
		Canvas_Line(x + side * width, y + height - 6,
			x + side * width, y + height, true);
	}

	for (x = 0; x < (int16_t)OLED_CANVAS_WIDTH; x += 5)
	{
		Canvas_Line(x, 47, x + 2, 47, true);
	}

	Canvas_Text((128 - (int16_t)strlen(title_p) * 6) / 2,
		(int16_t)roundf(menu_state.label_y), title_p);
	Canvas_Text(2, 54, "<");
	Canvas_Text(120, 54, ">");
}

/** @brief 绘制 16 像素行高的纵向列表与移动反白框。
 * @param model_p 业务快照。
 * @param now_ms 当前时间。
 * @retval 无。
 */
static void Menu_List(const car_menu_model_t *model_p, uint32_t now_ms)
{
	car_menu_page_e page = menu_state.navigation.page;
	uint8_t row = 0U;
	int16_t y = 0;
	int16_t value_x = 0;
	char text[16] = {0};

	for (row = 0U; row < MenuData_Count(page); row++)
	{
		/* 从屏外展开，同时保证相邻文字始终至少相隔 11 像素。 */
		y = (int16_t)roundf(row * MENU_ROW_HEIGHT + menu_state.camera_y -
			(16.0f + row * 5.0f) * (1.0f - menu_state.unfold / 100.0f));

		if (y > -12 && y < (int16_t)OLED_CANVAS_HEIGHT)
		{
			Canvas_Text(4, y + 4, MenuData_Label(page, row));
			MenuData_Value(page, row, model_p, now_ms, text, sizeof(text));
			value_x = 119 - (int16_t)strlen(text) * 6;
			Canvas_Text(value_x, y + 4, text);
		}
	}

	Canvas_InvertRoundBox((int16_t)roundf(menu_state.selector_x),
		(int16_t)roundf(menu_state.selector_y),
		(int16_t)roundf(menu_state.selector_width),
		(int16_t)roundf(menu_state.selector_height));
	Canvas_Line(125, 0, 125, 63, true);
	Canvas_Line(123, 0, 127, 0, true);
	Canvas_Line(123, 63, 127, 63, true);
	Canvas_Box(123, 0, 5, (int16_t)roundf(menu_state.bar), true);
}

/** @brief 显示滑入的只读数值弹窗或应用反馈。
 * @param model_p 业务快照。
 * @param now_ms 当前时间。
 * @retval 无。
 */
static void Menu_Popup(const car_menu_model_t *model_p, uint32_t now_ms)
{
	car_menu_page_e page = menu_state.popup_page;
	int16_t y = (int16_t)roundf(menu_state.popup_y);
	char text[16] = {0};
	const char *label_p = menu_state.message;

	if (y <= -30)
	{
		return;
	}

	if (menu_state.message[0] == '\0' && page != CAR_MENU_ROOT)
	{
		label_p = MenuData_Label(page, menu_state.popup_row);
		MenuData_Value(page, menu_state.popup_row,
			model_p, now_ms, text, sizeof(text));
	}

	Canvas_Dim();
	Canvas_Box(4, y, 120, 30, false);
	Canvas_RoundFrame(4, y, 120, 30);
	Canvas_Text(10, y + 5, label_p);
	Canvas_Text(10, y + 17, text);
}

/** @brief 诊断页限制显示位数，内部累计值保持不变。
 * @param value 原值。
 * @param limit 当前行容许的最大显示值。
 * @retval 用于显示的计数。
 */
static unsigned long Menu_Count(uint32_t value, uint32_t limit)
{
	return (unsigned long)(value > limit ? limit : value);
}

/** @brief 显示解析前的原始字节或硬件/协议分层计数。
 * @param model_p 当前接收快照。
 * @param now_ms 当前时间。
 * @retval 无。
 */
static void Menu_Raw(const car_menu_model_t *model_p, uint32_t now_ms)
{
	const vision_status_t *vision_p = &model_p->vision;
	char text[22] = {0};
	uint8_t row = 0U;
	uint8_t column = 0U;
	uint8_t index = 0U;

	if (menu_state.rows[CAR_MENU_RAW - 1U] == 0U)
	{
		Canvas_Text(0, 0, !model_p->uart_ready ? "UART NOT READY" :
			(menu_state.raw_hold ? "UART3 RAW HOLD" : "UART3 RAW LIVE"));
		snprintf(text, sizeof(text), "B%lu A%lu",
			Menu_Count(vision_p->bytes_received, 999999U),
			Menu_Count(vision_p->bytes_received == 0U ? 0U :
				now_ms - vision_p->last_byte_ms, 999999U));
		Canvas_Text(0, 8, text);

		for (row = 0U; row < 4U; row++)
		{
			snprintf(text, sizeof(text), "%02X:", (unsigned int)(row * 6U));

			for (column = 0U; column < 6U; column++)
			{
				index = row * 6U + column;

				if (index < menu_state.raw_length)
				{
					snprintf(&text[3U + column * 3U], 4U, "%02X ",
						(unsigned int)menu_state.raw[index]);
				}
				else
				{
					memcpy(&text[3U + column * 3U], "-- ", 4U);
				}
			}

			Canvas_Text(0, 16 + row * 8, text);
		}

		snprintf(text, sizeof(text), "OK%lu BAD%lu D%lu",
			Menu_Count(vision_p->accepted, 9999U),
			Menu_Count(vision_p->rejected, 9999U),
			Menu_Count(vision_p->duplicates, 9999U));
		Canvas_Text(0, 48, text);
		Canvas_Text(0, 56, "1/2 INFO 3:HOLD");
	}
	else
	{
		Canvas_Text(0, 0, "UART3 115200 8N1");
		snprintf(text, sizeof(text), "HW%lu APP%lu",
			Menu_Count(model_p->uart_received, 999999U),
			Menu_Count(vision_p->bytes_received, 999999U));
		Canvas_Text(0, 8, text);
		snprintf(text, sizeof(text), "DROP%lu OVR%lu",
			Menu_Count(model_p->uart_dropped, 999999U),
			Menu_Count(model_p->uart_overruns, 999999U));
		Canvas_Text(0, 16, text);
		snprintf(text, sizeof(text), "ERR%lu SYNC%lu",
			Menu_Count(model_p->uart_errors, 999999U),
			Menu_Count(vision_p->sync_hits, 999999U));
		Canvas_Text(0, 24, text);
		snprintf(text, sizeof(text), "OK%lu DUP%lu",
			Menu_Count(vision_p->accepted, 999999U),
			Menu_Count(vision_p->duplicates, 999999U));
		Canvas_Text(0, 32, text);
		snprintf(text, sizeof(text), "T%lu C%lu R%lu",
			Menu_Count(vision_p->tail_errors, 9999U),
			Menu_Count(vision_p->checksum_errors, 9999U),
			Menu_Count(vision_p->range_errors, 9999U));
		Canvas_Text(0, 40, text);
		snprintf(text, sizeof(text), "BAD%lu OLD%lu",
			Menu_Count(vision_p->rejected, 999999U),
			Menu_Count(vision_p->sequence_errors, 999999U));
		Canvas_Text(0, 48, text);
		Canvas_Text(0, 56, "1/2 HEX 3:HOLD");
	}
}

void Menu_Init(void)
{
	memset(&menu_state, 0, sizeof(menu_state));
	menu_state.selector_x = 46.0f;
	menu_state.selector_y = 7.0f;
	menu_state.selector_width = 36.0f;
	menu_state.selector_height = 36.0f;
	menu_state.label_y = 64.0f;
	menu_state.popup_y = -40.0f;
	menu_state.dirty = true;
	Canvas_Clear();
	OLED_Init();
}

car_menu_action_e Menu_KeyEvent(uint32_t keys)
{
	car_menu_page_e page = menu_state.navigation.page;
	uint8_t row = page == CAR_MENU_ROOT ? 0U : menu_state.rows[page - 1U];
	uint8_t count = page == CAR_MENU_ROOT ? MENU_ITEMS :
		(page == CAR_MENU_RAW ? 2U : MenuData_Count(page));
	bool up = (keys & CAR_MENU_KEY_UP) != 0U;
	bool down = (keys & CAR_MENU_KEY_DOWN) != 0U;

	if ((keys & CAR_MENU_KEY_BACK) != 0U)
	{
		if (menu_state.popup)
		{
			menu_state.popup = false;
		}
		else if (page != CAR_MENU_ROOT)
		{
			Menu_Open(CAR_MENU_ROOT);
		}

		menu_state.dirty = true;

		return CAR_MENU_ACTION_STOP;
	}

	if ((keys & CAR_MENU_KEY_CONFIRM) != 0U)
	{
		menu_state.dirty = true;

		if (menu_state.popup)
		{
			menu_state.popup = false;
		}
		else if (page == CAR_MENU_RAW)
		{
			menu_state.raw_hold = !menu_state.raw_hold;
		}
		else if (page == CAR_MENU_ROOT)
		{
			Menu_Open((car_menu_page_e)(menu_state.navigation.selected + 1U));
		}
		else if (row == 0U)
		{
			Menu_Open(CAR_MENU_ROOT);
		}
		else if (page == CAR_MENU_RUN && row <= 2U)
		{
			return row == 1U ? CAR_MENU_ACTION_START : CAR_MENU_ACTION_STOP;
		}
		else
		{
			menu_state.message[0] = '\0';
			menu_state.popup_page = page;
			menu_state.popup_row = row;
			menu_state.popup = true;
		}

		return CAR_MENU_ACTION_NONE;
	}

	if (!menu_state.popup && up != down)
	{
		row = page == CAR_MENU_ROOT ? menu_state.navigation.selected : row;
		row = up ? (uint8_t)((row + count - 1U) % count) :
			(uint8_t)((row + 1U) % count);

		if (page == CAR_MENU_ROOT)
		{
			menu_state.navigation.selected = row;
			menu_state.label_y = 64.0f;
		}
		else if (page == CAR_MENU_RAW)
		{
			menu_state.rows[page - 1U] = row;
		}
		else
		{
			menu_state.rows[page - 1U] = row;

			if (row < menu_state.tops[page - 1U])
			{
				menu_state.tops[page - 1U] = row;
			}
			else if (row >= menu_state.tops[page - 1U] + MENU_VISIBLE_ROWS)
			{
				menu_state.tops[page - 1U] = row - MENU_VISIBLE_ROWS + 1U;
			}
		}

		menu_state.dirty = true;
	}

	return CAR_MENU_ACTION_NONE;
}

void Menu_ShowMessage(const char *message_p)
{
	snprintf(menu_state.message, sizeof(menu_state.message), "%s", message_p);
	menu_state.popup = true;
	menu_state.dirty = true;
}

void Menu_Update(const car_menu_model_t *model_p, uint32_t now_ms)
{
	uint32_t elapsed_ms = now_ms - menu_state.last_tick_ms;

	if (!OLED_IsReady())
	{
		return;
	}

	if (!menu_state.raw_hold)
	{
		memcpy(menu_state.raw, model_p->vision.raw, sizeof(menu_state.raw));
		menu_state.raw_length = model_p->vision.raw_length;
	}

	if (elapsed_ms >= MENU_TICK_MS)
	{
		menu_state.last_tick_ms = now_ms;
		Menu_Animate(elapsed_ms > 32U ? 32U : elapsed_ms);
	}

	if (OLED_IsRefreshBusy())
	{
		OLED_RefreshStep();

		return;
	}

	if (!menu_state.dirty && (menu_state.navigation.page == CAR_MENU_ROOT ||
		(uint32_t)(now_ms - menu_state.last_frame_ms) < MENU_DATA_MS))
	{
		return;
	}

	menu_state.last_frame_ms = now_ms;
	menu_state.dirty = false;
	Canvas_Clear();

	if (menu_state.navigation.page == CAR_MENU_ROOT)
	{
		Menu_Tiles();
	}
	else if (menu_state.navigation.page == CAR_MENU_RAW)
	{
		Menu_Raw(model_p, now_ms);
	}
	else
	{
		Menu_List(model_p, now_ms);
	}

	Menu_Popup(model_p, now_ms);
	OLED_Present(Canvas_GetBuffer());
}

car_menu_status_t Menu_GetStatus(void)
{
	car_menu_status_t status = menu_state.navigation;

	if (status.page != CAR_MENU_ROOT)
	{
		status.row = menu_state.rows[status.page - 1U];
		status.scroll_top = menu_state.tops[status.page - 1U];
	}

	status.popup = menu_state.popup;
	status.animating = menu_state.animating;

	return status;
}
