#include "oled.h"
#include "oled_canvas.h"
#include "oledfont.h"
#include "ti_msp_dl_config.h"
#include "i2c.h"
#include "delay.h"
#include <string.h>

/** 增量刷新只读画布，缓存记录已确认写入屏幕的数据。 */
typedef struct
{
	const uint8_t *frame_p;
	uint8_t sent[OLED_CANVAS_BYTES];
	uint16_t cursor;
	uint16_t next_cursor;
	uint8_t failures;
	bool position_valid;
	bool busy;
	bool ready;
} oled_refresh_t;

static oled_refresh_t oled_refresh;
static I2C_Handle *g_oled_i2c;

void OLED_SDAUnlock(void)
{
    I2C_SDAUnlock(g_oled_i2c);
}

void OLED_WriteByte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];
    buf[0] = mode ? 0x40 : 0x00;
    buf[1] = dat;
    I2C_Write(g_oled_i2c, 0x3C, buf, 2);
}

void OLED_ColorTurn(uint8_t mode)
{
    if (mode == 0)
    {
        OLED_WriteByte(0xA6, OLED_CMD);
    }
    if (mode == 1)
    {
        OLED_WriteByte(0xA7, OLED_CMD);
    }
}

void OLED_DisplayTurn(uint8_t mode)
{
    if (mode == 0)
    {
        OLED_WriteByte(0xC8, OLED_CMD);
        OLED_WriteByte(0xA1, OLED_CMD);
    }
    if (mode == 1)
    {
        OLED_WriteByte(0xC0, OLED_CMD);
        OLED_WriteByte(0xA0, OLED_CMD);
    }
}

void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteByte(0xb0 + y, OLED_CMD);
    OLED_WriteByte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WriteByte((x & 0x0f), OLED_CMD);
}

void OLED_DisplayOn(void)
{
    OLED_WriteByte(0x8D, OLED_CMD);
    OLED_WriteByte(0x14, OLED_CMD);
    OLED_WriteByte(0xAF, OLED_CMD);
}

void OLED_DisplayOff(void)
{
    OLED_WriteByte(0x8D, OLED_CMD);
    OLED_WriteByte(0x10, OLED_CMD);
    OLED_WriteByte(0xAE, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WriteByte(0xb0 + i, OLED_CMD);
        OLED_WriteByte(0x00, OLED_CMD);
        OLED_WriteByte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            OLED_WriteByte(0, OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c = 0, sizex = sizey / 2;
    uint16_t i = 0, size1;
    if (sizey == 8)
        size1 = 6;
    else
        size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    c = chr - ' ';
    OLED_SetPos(x, y);
    for (i = 0; i < size1; i++)
    {
        if (i % sizex == 0 && sizey != 8)
            OLED_SetPos(x, y++);
        if (sizey == 8)
            OLED_WriteByte(asc2_0806[c][i], OLED_DATA);
        else if (sizey == 16)
            OLED_WriteByte(asc2_1608[c][i], OLED_DATA);
        else
            return;
    }
}

/**
 * @brief 计算 m 的 n 次幂
 *
 * 用于 OLED_ShowNum 中提取数字的每一位（十位、百位等）。
 *
 * @param m 底数
 * @param n 指数
 * @return m 的 n 次幂结果
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    if (sizey == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *str, uint8_t sizey)
{
    uint8_t j = 0;
    while (str[j] != '\0')
    {
        OLED_ShowChar(x, y, str[j++], sizey);
        if (sizey == 8)
            x += 6;
        else
            x += sizey / 2;
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint16_t i, size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    for (i = 0; i < size1; i++)
    {
        if (i % sizey == 0)
            OLED_SetPos(x, y++);
        if (sizey == 16)
            OLED_WriteByte(Hzk[no][i], OLED_DATA);
        else
            return;
    }
}

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])
{
    uint16_t j = 0;
    uint8_t i, m;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (i = 0; i < sizey; i++)
    {
        OLED_SetPos(x, i + y);
        for (m = 0; m < sizex; m++)
        {
            OLED_WriteByte(BMP[j++], OLED_DATA);
        }
    }
}

void OLED_Init(void)
{
    I2C_Config i2c_cfg = {
        .i2c          = I2C_OLED_INST,
        .sclPort      = GPIO_I2C_OLED_SCL_PORT,
        .sclPin       = GPIO_I2C_OLED_SCL_PIN,
        .sclIomux     = GPIO_I2C_OLED_IOMUX_SCL,
        .sclIomuxFunc = GPIO_I2C_OLED_IOMUX_SCL_FUNC,
        .sdaPort      = GPIO_I2C_OLED_SDA_PORT,
        .sdaPin       = GPIO_I2C_OLED_SDA_PIN,
        .sdaIomux     = GPIO_I2C_OLED_IOMUX_SDA,
        .sdaIomuxFunc = GPIO_I2C_OLED_IOMUX_SDA_FUNC,
        .syscfgInit   = SYSCFG_DL_I2C_OLED_init,
    };
    g_oled_i2c = I2C_Init(&i2c_cfg);

    if (g_oled_i2c == NULL)
    {
        return;
    }

    if (DL_I2C_getSDAStatus(g_oled_i2c->i2c) == DL_I2C_CONTROLLER_SDA_LOW)
        OLED_SDAUnlock();

    delay_ms(200);

    OLED_WriteByte(0xAE, OLED_CMD);
    OLED_WriteByte(0x00, OLED_CMD);
    OLED_WriteByte(0x10, OLED_CMD);
    OLED_WriteByte(0x40, OLED_CMD);
    OLED_WriteByte(0x81, OLED_CMD);
    OLED_WriteByte(0xCF, OLED_CMD);
    OLED_WriteByte(0xA1, OLED_CMD);
    OLED_WriteByte(0xC8, OLED_CMD);
    OLED_WriteByte(0xA6, OLED_CMD);
    OLED_WriteByte(0xA8, OLED_CMD);
    OLED_WriteByte(0x3F, OLED_CMD);
    OLED_WriteByte(0xD3, OLED_CMD);
    OLED_WriteByte(0x00, OLED_CMD);
    OLED_WriteByte(0xD5, OLED_CMD);
    OLED_WriteByte(0x80, OLED_CMD);
    OLED_WriteByte(0xD9, OLED_CMD);
    OLED_WriteByte(0xF1, OLED_CMD);
    OLED_WriteByte(0xDA, OLED_CMD);
    OLED_WriteByte(0x12, OLED_CMD);
    OLED_WriteByte(0xDB, OLED_CMD);
    OLED_WriteByte(0x40, OLED_CMD);
    OLED_WriteByte(0x20, OLED_CMD);
    OLED_WriteByte(0x02, OLED_CMD);
    OLED_WriteByte(0x8D, OLED_CMD);
    OLED_WriteByte(0x14, OLED_CMD);
    OLED_WriteByte(0xA4, OLED_CMD);
    OLED_WriteByte(0xA6, OLED_CMD);
    OLED_Clear();
    OLED_WriteByte(0xAF, OLED_CMD);
    memset(&oled_refresh, 0, sizeof(oled_refresh));
    oled_refresh.ready = true;
}

const uint8_t *OLED_GetGlyph(uint8_t character)
{
	if (character < ' ' || character > '~')
	{
		character = '?';
	}

	return asc2_0806[character - ' '];
}

bool OLED_Present(const uint8_t *frame_p)
{
	if (!oled_refresh.ready || oled_refresh.busy || frame_p == NULL)
	{
		return false;
	}

	oled_refresh.frame_p = frame_p;
	oled_refresh.cursor = 0U;
	oled_refresh.position_valid = false;
	oled_refresh.busy = true;

	return true;
}

void OLED_RefreshStep(void)
{
	uint16_t cursor = oled_refresh.cursor;
	uint8_t column = 0U;
	uint8_t length = 0U;
	uint8_t commands[4] = {0U};
	uint8_t data[8] = {0x40U};
	bool success = false;

	if (!oled_refresh.ready || !oled_refresh.busy)
	{
		return;
	}

	while (cursor < sizeof(oled_refresh.sent) &&
		oled_refresh.sent[cursor] == oled_refresh.frame_p[cursor])
	{
		cursor++;
	}

	if (cursor == sizeof(oled_refresh.sent))
	{
		oled_refresh.busy = false;

		return;
	}

	column = (uint8_t)(cursor % OLED_CANVAS_WIDTH);
	length = (uint8_t)(OLED_CANVAS_WIDTH - column);

	if (length > sizeof(data) - 1U)
	{
		length = sizeof(data) - 1U;
	}

	commands[1] = (uint8_t)(0xB0U + cursor / OLED_CANVAS_WIDTH);
	commands[2] = column & 0x0FU;
	commands[3] = 0x10U | (column >> 4);
	memcpy(&data[1], &oled_refresh.frame_p[cursor], length);
	/* 同页连续数据使用屏幕自动增列，跳过重复定位事务。 */
	success = true;

	if (!oled_refresh.position_valid || cursor != oled_refresh.next_cursor)
	{
		success = I2C_TryWrite(g_oled_i2c, 0x3CU,
			commands, sizeof(commands));
	}

	if (success)
	{
		success = I2C_TryWrite(g_oled_i2c, 0x3CU, data, length + 1U);
	}

	if (success)
	{
		memcpy(&oled_refresh.sent[cursor], &data[1], length);
		oled_refresh.cursor = cursor + length;
		oled_refresh.next_cursor = cursor + length;
		oled_refresh.position_valid = column + length < OLED_CANVAS_WIDTH;
		oled_refresh.failures = 0U;
	}
	else
	{
		oled_refresh.position_valid = false;
		oled_refresh.failures++;

		if (oled_refresh.failures >= 5U)
		{
			oled_refresh.ready = false;
			oled_refresh.busy = false;
		}
	}
}

bool OLED_IsRefreshBusy(void)
{
	return oled_refresh.busy;
}

bool OLED_IsReady(void)
{
	return oled_refresh.ready;
}
