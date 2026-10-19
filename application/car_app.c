#include "car_app.h"
#include "car_config.h"
#include "board.h"
#include "chassis_control.h"
#include "race_control.h"
#include "uart.h"
#include "vision.h"
#include "tuning/car_tuning.h"
#include <stddef.h>

#if (CAR_ENABLE_MENU == 1U)
#include "car_menu.h"
#endif

#if (CAR_STAGE >= CAR_STAGE_STEERING) || (CAR_ENABLE_MENU == 1U)
#include "key.h"
#endif

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
#include "imu.h"
#include "slope.h"
#endif

/** ISR 只发布按键位，其他应用状态全部由主循环拥有。 */
typedef struct
{
	UART_Handle *uart_p;
	volatile uint32_t key_events;
	uint32_t last_control_ms;
	uint32_t late_updates;
#if (CAR_ENABLE_MENU == 1U)
	car_menu_model_t menu_model;
#endif
} car_state_t;

static car_state_t car_state;

#if (CAR_STAGE >= CAR_STAGE_STEERING) || (CAR_ENABLE_MENU == 1U)
/** @brief 记录已有按键模块的释放事件，不在 ISR 控制执行器。
 * @param key_index 按键索引，依次为上、下、确认、返回。
 * @retval 无。
 */
static void Car_KeyEvent(uint8_t key_index)
{
	car_state.key_events |= (uint32_t)1U << key_index;
}

/** @brief 原子取走按键事件。 @retval 本次事件位图。 */
static uint32_t Car_TakeKeys(void)
{
	uint32_t primask = __get_PRIMASK();
	uint32_t events = 0U;

	__disable_irq();
	events = car_state.key_events;
	car_state.key_events = 0U;
	__set_PRIMASK(primask);

	return events;
}
#endif

#if (CAR_ENABLE_MENU == 1U)
/** @brief 在主循环低优先级位置提供只读数据并刷新一小块。
 * @param vision_p 当前视觉快照。
 * @retval 无。
 */
static void Car_MenuService(const vision_status_t *vision_p)
{
	car_menu_model_t *model_p = &car_state.menu_model;
	uart_rx_status_t uart_status = {0};

	model_p->vision = *vision_p;
	model_p->race = Race_GetOutput();
	model_p->uart_ready = car_state.uart_p != NULL;

	if (model_p->uart_ready)
	{
		uart_status = UART_GetRxStatus(car_state.uart_p);
		model_p->uart_received = uart_status.received;
		model_p->uart_dropped = uart_status.dropped;
		model_p->uart_overruns = uart_status.overruns;
		model_p->uart_errors = uart_status.errors;
	}

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	model_p->slope = Slope_GetStatus();
	model_p->pitch_available = true;
#endif

	Menu_Update(model_p, Board_GetTickMs());
}
#endif

void Car_Init(void)
{
	UART_Config uart_config =
	{
		.uart = UART_PI_INST,
		.irqNum = UART_PI_INST_INT_IRQN,
		.dmaTxChanId = 0U,
		.dmaTxTrigger = 0U
	};

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	slope_config_t slope_config =
	{
		.enter_deg = CAR_PITCH_ENTER_DEG,
		.exit_deg = CAR_PITCH_EXIT_DEG,
		.filter_gain = CAR_PITCH_FILTER_GAIN,
		.direction = CAR_PITCH_DIRECTION,
		.confirm_ms = CAR_PITCH_CONFIRM_MS,
		.warmup_samples = CAR_PITCH_WARMUP_SAMPLES,
		.baseline_samples = CAR_PITCH_BASELINE_SAMPLES
	};
#endif

	Race_Init();
	Chassis_Init();

#if (CAR_ENABLE_MENU == 1U)
	Menu_Init();
#endif

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	IMU_Init();
	Slope_Init(&slope_config);
#endif

#if (CAR_STAGE >= CAR_STAGE_STEERING) || (CAR_ENABLE_MENU == 1U)
	Key_Init();
	Key_RegisterCallback(Car_KeyEvent);
#endif

	/* 阻塞硬件初始化结束后再接入视觉，避免启动积压的旧数据。 */
	Vision_Init();
	car_state.uart_p = UART_Init(&uart_config);

	if (car_state.uart_p != NULL)
	{
		UART_StartReceiveRaw(car_state.uart_p);
	}

	car_state.last_control_ms = Board_GetTickMs();
}

void Car_Update(void)
{
	uint32_t now_ms = Board_GetTickMs();
	uint16_t remaining = 0U;
	uint8_t byte = 0U;
	vision_status_t vision = {0};
	race_output_t output = {0};
	float drive_scale = 1.0f;

#if (CAR_STAGE >= CAR_STAGE_STEERING) || (CAR_ENABLE_MENU == 1U)
	uint32_t keys = 0U;

#if (CAR_ENABLE_MENU == 1U)
	car_menu_action_e action = CAR_MENU_ACTION_NONE;
#endif
#endif

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	slope_status_t slope = {0};
#endif

	if (car_state.uart_p != NULL)
	{
		remaining = UART_RawRxAvailable(car_state.uart_p);
	}

	while (remaining > 0U)
	{
		byte = UART_ReadRawByte(car_state.uart_p);
		Vision_FeedByte(byte, now_ms);
		remaining--;
	}

	vision = Vision_GetStatus();

#if (CAR_STAGE >= CAR_STAGE_STEERING) || (CAR_ENABLE_MENU == 1U)
	keys = Car_TakeKeys();

#if (CAR_ENABLE_MENU == 1U)
	action = Menu_KeyEvent(keys);
#endif

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	/* 菜单只发布意图，启动条件和输出仍由应用检查。 */
#if (CAR_ENABLE_MENU == 1U)
	if (action == CAR_MENU_ACTION_STOP)
#else
	if ((keys & 2U) != 0U)
#endif
	{
		Race_Stop(false);
		output = Race_GetOutput();
		Chassis_Apply(&output);
	}
#if (CAR_ENABLE_MENU == 1U)
	else if (action == CAR_MENU_ACTION_START)
#else
	else if ((keys & 1U) != 0U)
#endif
	{
#if (CAR_STAGE >= CAR_STAGE_SLOPE)
		slope = Slope_GetStatus();

		if (slope.ready)
#endif
		{
#if (CAR_ENABLE_MENU == 1U)
			output = Race_GetOutput();

			if (output.enabled)
			{
				Menu_ShowMessage("Already running");
			}
			else if (!Race_Start(&vision.sample, now_ms))
			{
				Menu_ShowMessage("Vision not ready");
			}
#else
			Race_Start(&vision.sample, now_ms);
#endif
		}
#if (CAR_STAGE >= CAR_STAGE_SLOPE) && (CAR_ENABLE_MENU == 1U)
		else
		{
			Menu_ShowMessage("IMU not ready");
		}
#endif
	}
#elif (CAR_ENABLE_MENU == 1U)
	if (action == CAR_MENU_ACTION_START)
	{
		Menu_ShowMessage("Stage 1: RX only");
	}
#endif
#endif

	if ((uint32_t)(now_ms - car_state.last_control_ms) <
		CAR_CONTROL_PERIOD_MS)
	{
#if (CAR_ENABLE_MENU == 1U)
		Car_MenuService(&vision);
#endif

		return;
	}

	if ((uint32_t)(now_ms - car_state.last_control_ms) >
		2U * CAR_CONTROL_PERIOD_MS)
	{
		car_state.late_updates++;
	}

	car_state.last_control_ms = now_ms;

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	IMU_Update();
	IMU_GetEuler(&roll, &pitch, &yaw);
	Slope_Update(pitch, now_ms);
	slope = Slope_GetStatus();
	drive_scale = slope.ready ? 1.0f : 0.0f;

	if (slope.phase != SLOPE_FLAT)
	{
		drive_scale *= CarTuning_Get(CAR_TUNE_SLOPE_SCALE);
	}
#endif

	/* SPI 等同步调用可能耗时，输出前重新检查真实观测年龄。 */
	now_ms = Board_GetTickMs();

	Race_Update(&vision.sample, now_ms, drive_scale);
	output = Race_GetOutput();
	Chassis_Apply(&output);

#if (CAR_ENABLE_MENU == 1U)
	Car_MenuService(&vision);
#endif
}
