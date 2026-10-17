#include "oled_canvas.h"
#include "oled.h"
#include <string.h>

/** 像素按 SSD1306 页布局存储，避免动态内存和运行时格式转换。 */
typedef struct
{
	uint8_t bytes[OLED_CANVAS_BYTES];
} canvas_state_t;

static canvas_state_t canvas_state;

void Canvas_Clear(void)
{
	memset(&canvas_state, 0, sizeof(canvas_state));
}

void Canvas_Dim(void)
{
	uint16_t column = 0U;
	uint8_t page = 0U;

	for (page = 0U; page < OLED_CANVAS_PAGES; page++)
	{
		for (column = 0U; column < OLED_CANVAS_WIDTH; column++)
		{
			canvas_state.bytes[page * OLED_CANVAS_WIDTH + column] &=
				column % 2U == 0U ? 0x55U : 0xAAU;
		}
	}
}

void Canvas_Pixel(int16_t x, int16_t y, bool on)
{
	uint16_t offset = 0U;
	uint8_t mask = 0U;

	if (x < 0 || y < 0 || x >= (int16_t)OLED_CANVAS_WIDTH ||
		y >= (int16_t)OLED_CANVAS_HEIGHT)
	{
		return;
	}

	offset = (uint16_t)(y / 8) * OLED_CANVAS_WIDTH + (uint16_t)x;
	mask = (uint8_t)(1U << (y % 8));

	if (on)
	{
		canvas_state.bytes[offset] |= mask;
	}
	else
	{
		canvas_state.bytes[offset] &= (uint8_t)~mask;
	}
}

void Canvas_Text(int16_t x, int16_t y, const char *text_p)
{
	const uint8_t *glyph_p = NULL;
	uint8_t column = 0U;
	uint8_t row = 0U;

	while (*text_p != '\0' && x < (int16_t)OLED_CANVAS_WIDTH)
	{
		glyph_p = OLED_GetGlyph((uint8_t)*text_p);

		for (column = 0U; column < 6U; column++)
		{
			for (row = 0U; row < 8U; row++)
			{
				if ((glyph_p[column] & (1U << row)) != 0U)
				{
					Canvas_Pixel(x + column, y + row, true);
				}
			}
		}

		x += 6;
		text_p++;
	}
}

void Canvas_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
	bool on)
{
	int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
	int16_t dy = y1 > y0 ? y0 - y1 : y1 - y0;
	int16_t step_x = x0 < x1 ? 1 : -1;
	int16_t step_y = y0 < y1 ? 1 : -1;
	int16_t error = dx + dy;
	int16_t doubled = 0;

	while (true)
	{
		Canvas_Pixel(x0, y0, on);

		if (x0 == x1 && y0 == y1)
		{
			return;
		}

		doubled = 2 * error;

		if (doubled >= dy)
		{
			error += dy;
			x0 += step_x;
		}

		if (doubled <= dx)
		{
			error += dx;
			y0 += step_y;
		}
	}
}

void Canvas_Box(int16_t x, int16_t y, int16_t width, int16_t height,
	bool on)
{
	int16_t row = 0;

	for (row = 0; row < height; row++)
	{
		Canvas_Line(x, y + row, x + width - 1, y + row, on);
	}
}

void Canvas_InvertRoundBox(int16_t x, int16_t y, int16_t width,
	int16_t height)
{
	int16_t row = 0;
	int16_t column = 0;
	int16_t inset = 0;
	int16_t pixel_x = 0;
	int16_t pixel_y = 0;
	uint16_t offset = 0U;

	for (row = 0; row < height; row++)
	{
		inset = row == 0 || row == height - 1 ? 2 :
			(row == 1 || row == height - 2 ? 1 : 0);

		for (column = inset; column < width - inset; column++)
		{
			pixel_x = x + column;
			pixel_y = y + row;

			if (pixel_x >= 0 && pixel_y >= 0 &&
				pixel_x < (int16_t)OLED_CANVAS_WIDTH &&
				pixel_y < (int16_t)OLED_CANVAS_HEIGHT)
			{
				offset = (uint16_t)(pixel_y / 8) * OLED_CANVAS_WIDTH +
					(uint16_t)pixel_x;
				canvas_state.bytes[offset] ^=
					(uint8_t)(1U << (pixel_y % 8));
			}
		}
	}
}

void Canvas_RoundFrame(int16_t x, int16_t y, int16_t width,
	int16_t height)
{
	int16_t offset = 0;

	if (width < 5 || height < 5)
	{
		return;
	}

	for (offset = 2; offset < width - 2; offset++)
	{
		Canvas_Pixel(x + offset, y, true);
		Canvas_Pixel(x + offset, y + height - 1, true);
	}

	for (offset = 2; offset < height - 2; offset++)
	{
		Canvas_Pixel(x, y + offset, true);
		Canvas_Pixel(x + width - 1, y + offset, true);
	}

	Canvas_Pixel(x + 1, y + 1, true);
	Canvas_Pixel(x + width - 2, y + 1, true);
	Canvas_Pixel(x + 1, y + height - 2, true);
	Canvas_Pixel(x + width - 2, y + height - 2, true);
}

const uint8_t *Canvas_GetBuffer(void)
{
	return canvas_state.bytes;
}
