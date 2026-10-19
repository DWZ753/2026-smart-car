#include "car_menu_data.h"
#include "car_config.h"
#include "tuning/car_tuning.h"
#include <math.h>
#include <stdio.h>

#define MENU_PAGE_ROW_CAPACITY 12U
#define MENU_TUNE_ROWS (1U + (unsigned)CAR_TUNE_COUNT + 1U)

/** 每页第零项为返回入口；仅运行页的 Start/Stop 产生应用命令。 */
typedef struct
{
	uint8_t count;
	const char *labels[MENU_PAGE_ROW_CAPACITY];
} menu_page_t;

static const menu_page_t menu_pages[] =
{
	{7U, {"- Run status", "Start", "Stop", "State", "PWM", "Turn", "Link"}},
	{10U, {"- Vision", "Dev", "K", "Valid", "Conf", "Lost", "Seq",
		"Age ms", "Rx", "Bad"}},
	{5U, {"- IMU pitch", "Pitch", "Base", "Phase", "Ready"}},
	{6U, {"- Control", "KP", "KH", "PWM", "Limit", "Stage"}},
	{MENU_TUNE_ROWS, {"- Debug tune", "Drive PWM", "Turn PWM",
		"Degraded PWM", "KP", "KH", "Limit", "Valid min",
		"Turn thresh", "Slope scale", "Restore defaults"}}
};

/* 列表页必须从 RUN 连续排列到 RAW 之前，RAW 为独立整屏页面。 */
_Static_assert(sizeof(menu_pages) / sizeof(menu_pages[0]) ==
	(size_t)(CAR_MENU_RAW - 1U), "menu_pages must match page enum");
_Static_assert(MENU_TUNE_ROWS <= MENU_PAGE_ROW_CAPACITY,
	"tune rows exceed label storage");
static const char *const race_titles[] =
{
	"READY", "DEPART", "RUN", "STOPPING", "DONE", "FAULT"
};
static const char *const slope_titles[] =
{
	"FLAT", "UPHILL", "CREST", "DOWNHILL"
};

/** @brief 定点格式化，避免引入 printf 浮点支持。
 * @param text_p 输出缓冲。
 * @param capacity 输出容量。
 * @param value 数值。
 * @param scale 10、100 或 1000。
 * @retval 无。
 */
static void MenuData_Float(char *text_p, size_t capacity, float value,
	int32_t scale)
{
	int32_t raw = (int32_t)roundf(fabsf(value) * (float)scale);
	int precision = scale == 1000 ? 3 : (scale == 100 ? 2 : 1);

	snprintf(text_p, capacity, "%c%ld.%0*ld", value < 0.0f ? '-' : '+',
		(long)(raw / scale), precision, (long)(raw % scale));
}

uint8_t MenuData_Count(car_menu_page_e page)
{
	return menu_pages[page - 1U].count;
}

const char *MenuData_Label(car_menu_page_e page, uint8_t row)
{
	return menu_pages[page - 1U].labels[row];
}

void MenuData_Value(car_menu_page_e page, uint8_t row,
	const car_menu_model_t *model_p, uint32_t now_ms,
	char *text_p, size_t capacity)
{
	uint32_t age = now_ms - model_p->vision.sample.received_ms;
	uint32_t value = 0U;
	float number = 0.0f;

	text_p[0] = '\0';

	if (row == 0U)
	{
		return;
	}

	if (page == CAR_MENU_RUN)
	{
		switch (row)
		{
			case 1U:
			case 2U:
				snprintf(text_p, capacity, ">");
				break;
			case 3U:
				snprintf(text_p, capacity, "%s",
					race_titles[model_p->race.phase]);
				break;
			case 4U:
				snprintf(text_p, capacity, "%ld",
					(long)model_p->race.drive_pwm);
				break;
			case 5U:
				MenuData_Float(text_p, capacity,
					model_p->race.turn_right, 10);
				break;
			default:
				snprintf(text_p, capacity, "%s",
					model_p->vision.sample.sample_id == 0U ? "NONE" :
					(age < CAR_VISION_STOP_MS ? "LIVE" : "STALE"));
				break;
		}
	}
	else if (page == CAR_MENU_VISION)
	{
		if (row == 1U || row == 2U)
		{
			MenuData_Float(text_p, capacity, row == 1U ?
				model_p->vision.sample.dev : model_p->vision.sample.k,
				row == 1U ? 1000 : 100);

			return;
		}

		switch (row)
		{
			case 3U:
				value = model_p->vision.sample.valid;
				break;
			case 4U:
				value = model_p->vision.sample.conf;
				break;
			case 5U:
				value = model_p->vision.sample.lost;
				break;
			case 6U:
				value = model_p->vision.sample.seq;
				break;
			case 7U:
				value = age;
				break;
			case 8U:
				value = model_p->vision.accepted;
				break;
			default:
				value = model_p->vision.rejected;
				break;
		}

		snprintf(text_p, capacity, "%lu", (unsigned long)value);
	}
	else if (page == CAR_MENU_PITCH)
	{
		if (!model_p->pitch_available)
		{
			snprintf(text_p, capacity, "OFF / S4");
		}
		else if (row == 1U || row == 2U)
		{
			MenuData_Float(text_p, capacity, row == 1U ?
				model_p->slope.relative_pitch :
				model_p->slope.baseline_pitch, 10);
		}
		else
		{
			snprintf(text_p, capacity, "%s", row == 3U ?
				slope_titles[model_p->slope.phase] :
				(model_p->slope.ready ? "YES" : "WAIT"));
		}
	}
	else if (page == CAR_MENU_CONTROL)
	{
		if (row == 1U || row == 2U)
		{
			number = row == 1U ?
				CarTuning_Get(CAR_TUNE_POSITION_GAIN) :
				CarTuning_Get(CAR_TUNE_HEADING_GAIN);
			MenuData_Float(text_p, capacity, number, 10);
		}
		else
		{
			value = row == 3U ?
				(uint32_t)CarTuning_Get(CAR_TUNE_DRIVE_PWM) :
				(row == 4U ?
				(uint32_t)CarTuning_Get(CAR_TUNE_STEERING_LIMIT) :
				CAR_STAGE);
			snprintf(text_p, capacity, "%lu", (unsigned long)value);
		}
	}
	else
	{
		/* CAR_MENU_TUNE：最后一行为恢复默认值，不显示数值。 */
		if (row >= 1U && row <= (uint8_t)CAR_TUNE_COUNT)
		{
			car_tune_id_e id = (car_tune_id_e)(row - 1U);

			if (id == CAR_TUNE_SLOPE_SCALE)
			{
				MenuData_Float(text_p, capacity,
					CarTuning_Get(id), 100);
			}
			else
			{
				snprintf(text_p, capacity, "%ld",
					(long)roundf(CarTuning_Get(id)));
			}
		}
	}
}
