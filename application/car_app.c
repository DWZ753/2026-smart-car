#include "car_app.h"
#include "car_config.h"
#include "board.h"
#include "chassis_control.h"
#include "race_control.h"
#include "uart.h"
#include "vision.h"
#include <stddef.h>

#if (CAR_STAGE >= CAR_STAGE_STEERING)
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
} car_state_t;

static car_state_t car_state;

#if (CAR_STAGE >= CAR_STAGE_STEERING)
/** @brief 记录已有按键模块的释放事件，不在 ISR 控制执行器。
 * @param key_index 按键索引，0 启动、1 停止。
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

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	IMU_Init();
	Slope_Init(&slope_config);
#endif

#if (CAR_STAGE >= CAR_STAGE_STEERING)
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

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	uint32_t keys = 0U;
#endif

#if (CAR_STAGE >= CAR_STAGE_SLOPE)
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	slope_status_t slope = {0};
#endif

	if (car_state.uart_p == NULL)
	{
		return;
	}

	remaining = UART_RawRxAvailable(car_state.uart_p);

	while (remaining > 0U)
	{
		byte = UART_ReadRawByte(car_state.uart_p);
		Vision_FeedByte(byte, now_ms);
		remaining--;
	}

	vision = Vision_GetStatus();

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	keys = Car_TakeKeys();

	/* 同时收到启停事件时停止优先；只靠视觉帧不会重新启动。 */
	if ((keys & 2U) != 0U)
	{
		Race_Stop(false);
		output = Race_GetOutput();
		Chassis_Apply(&output);
	}
	else if ((keys & 1U) != 0U)
	{
#if (CAR_STAGE >= CAR_STAGE_SLOPE)
		slope = Slope_GetStatus();

		if (slope.ready)
#endif
		{
			Race_Start(&vision.sample, now_ms);
		}
	}
#endif

	if ((uint32_t)(now_ms - car_state.last_control_ms) <
		CAR_CONTROL_PERIOD_MS)
	{
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
		drive_scale *= CAR_SLOPE_DRIVE_SCALE;
	}
#endif

	/* SPI 等同步调用可能耗时，输出前重新检查真实观测年龄。 */
	now_ms = Board_GetTickMs();

	Race_Update(&vision.sample, now_ms, drive_scale);
	output = Race_GetOutput();
	Chassis_Apply(&output);
}
