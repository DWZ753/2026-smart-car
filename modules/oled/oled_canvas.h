#ifndef OLED_CANVAS_H
#define OLED_CANVAS_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_CANVAS_WIDTH 128U
#define OLED_CANVAS_HEIGHT 64U
#define OLED_CANVAS_PAGES (OLED_CANVAS_HEIGHT / 8U)
#define OLED_CANVAS_BYTES (OLED_CANVAS_WIDTH * OLED_CANVAS_PAGES)

/** @brief 清空内存画布，不发送 I2C。 @retval 无。 */
void Canvas_Clear(void);

/** @brief 保留棋盘状像素，压暗弹窗背后的画面。 @retval 无。 */
void Canvas_Dim(void);

/** @brief 绘制裁剪后的单个像素。
 * @param x 横坐标，可为负。
 * @param y 纵坐标，可为负。
 * @param on true 点亮，false 清除。
 * @retval 无。
 */
void Canvas_Pixel(int16_t x, int16_t y, bool on);

/** @brief 绘制 6x8 ASCII 字符串，超屏部分裁剪。
 * @param x 横坐标。
 * @param y 像素纵坐标，不是 SSD1306 页号。
 * @param text_p 只读字符串。
 * @retval 无。
 */
void Canvas_Text(int16_t x, int16_t y, const char *text_p);

/** @brief 绘制裁剪后的直线。
 * @param x0 起点横坐标。
 * @param y0 起点纵坐标。
 * @param x1 终点横坐标。
 * @param y1 终点纵坐标。
 * @param on true 点亮，false 清除。
 * @retval 无。
 */
void Canvas_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
	bool on);

/** @brief 填充裁剪后的矩形。
 * @param x 左上横坐标。
 * @param y 左上纵坐标。
 * @param width 宽度。
 * @param height 高度。
 * @param on true 点亮，false 清除。
 * @retval 无。
 */
void Canvas_Box(int16_t x, int16_t y, int16_t width, int16_t height,
	bool on);

/** @brief 反转圆角矩形内像素，保留文字形成反白效果。
 * @param x 左上横坐标。
 * @param y 左上纵坐标。
 * @param width 宽度。
 * @param height 高度。
 * @retval 无。
 */
void Canvas_InvertRoundBox(int16_t x, int16_t y, int16_t width,
	int16_t height);

/** @brief 绘制两像素圆角的选择框。
 * @param x 左上横坐标。
 * @param y 左上纵坐标。
 * @param width 宽度。
 * @param height 高度。
 * @retval 无。
 */
void Canvas_RoundFrame(int16_t x, int16_t y, int16_t width,
	int16_t height);

/** @brief 获取页布局画布只读指针。
 * @retval OLED_CANVAS_BYTES 字节；刷新进行中不得修改画布。
 */
const uint8_t *Canvas_GetBuffer(void);

#endif
