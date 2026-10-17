#ifndef CAR_MENU_H
#define CAR_MENU_H

#include "race_control.h"
#include "slope.h"
#include "vision.h"

#define CAR_MENU_KEY_UP (1U << 0)
#define CAR_MENU_KEY_DOWN (1U << 1)
#define CAR_MENU_KEY_CONFIRM (1U << 2)
#define CAR_MENU_KEY_BACK (1U << 3)

/** UI 只发布命令，由 car_app 检查启动门控并控制底盘。 */
typedef enum
{
	CAR_MENU_ACTION_NONE,
	CAR_MENU_ACTION_START,
	CAR_MENU_ACTION_STOP
} car_menu_action_e;

/** 菜单只读数据，由应用收集，页面不直接操纵执行器。 */
typedef struct
{
	vision_status_t vision;
	race_output_t race;
	slope_status_t slope;
	bool pitch_available;
	uint32_t uart_received;
	uint32_t uart_dropped;
	uint32_t uart_overruns;
	uint32_t uart_errors;
	bool uart_ready;
} car_menu_model_t;

/** 页面编号，主菜单四项对应四个详情页。 */
typedef enum
{
	CAR_MENU_ROOT,
	CAR_MENU_RUN,
	CAR_MENU_VISION,
	CAR_MENU_PITCH,
	CAR_MENU_CONTROL,
	CAR_MENU_RAW
} car_menu_page_e;

/** 导航状态副本，用于调试和主机验证。 */
typedef struct
{
	car_menu_page_e page;
	uint8_t selected;
	uint8_t row;
	uint8_t scroll_top;
	bool popup;
	bool animating;
} car_menu_status_t;

/** @brief 初始化 OLED 与菜单，必须在视觉接收前调用。 @retval 无。 */
void Menu_Init(void);

/** @brief 处理导航事件。
 * @param keys KEY1/2/3/4 分别为上、下、确认、返回。
 * @retval 应用命令；返回键同时发布停止，优先于确认。
 */
car_menu_action_e Menu_KeyEvent(uint32_t keys);

/** @brief 显示应用反馈，例如视觉或 IMU 尚未就绪。
 * @param message_p 消息，会复制到菜单内部缓冲。
 * @retval 无。
 */
void Menu_ShowMessage(const char *message_p);

/** @brief 主循环末尾处理一个刷新步骤或生成新画面。
 * @param model_p 业务只读数据。
 * @param now_ms 当前毫秒时间。
 * @retval 无。
 */
void Menu_Update(const car_menu_model_t *model_p, uint32_t now_ms);

/** @brief 获取导航状态。 @retval 当前页面和选中项。 */
car_menu_status_t Menu_GetStatus(void);

#endif
