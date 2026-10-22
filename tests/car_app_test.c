#include "car_app.h"
#include "car_config.h"
#include "car_menu.h"
#include "board.h"
#include "chassis_control.h"
#include "imu.h"
#include "key.h"
#include "uart.h"
#include "tuning/car_tuning.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TestHardware test_hardware;

/** 硬件边界替身，比赛、视觉、坡道和应用调度均使用真实实现。 */
typedef struct
{
	UART_Handle uart;
	Key_Callback_t key_callback;
	car_menu_action_e action;
	uint32_t now_ms;
	uint32_t messages;
	char last_message[21];
	uint8_t frame[13];
	uint8_t position;
	uint8_t seq;
	float pitch;
} app_mock_t;

static app_mock_t app_mock;

uint32_t Board_GetTickMs(void)
{
	return app_mock.now_ms;
}

void Key_Init(void)
{
}

void Key_RegisterCallback(Key_Callback_t callback)
{
	app_mock.key_callback = callback;
}
void Chassis_Init(void)
{
}

void Chassis_Apply(const race_output_t *output_p)
{
#if (CAR_STAGE == CAR_STAGE_RECEIVE)
	assert(!output_p->enabled);
#else
	(void)output_p;
#endif
}
void IMU_Init(void)
{
}

void IMU_Update(void)
{
}

void IMU_GetEuler(float *roll_p, float *pitch_p, float *yaw_p)
{
	*roll_p = 0.0f;
	*pitch_p = app_mock.pitch;
	*yaw_p = 0.0f;
}
UART_Handle *UART_Init(const UART_Config *config_p)
{
	(void)config_p;
	return &app_mock.uart;
}
void UART_StartReceiveRaw(UART_Handle *handle_p)
{
	(void)handle_p;
}

uint16_t UART_RawRxAvailable(UART_Handle *handle_p)
{
	(void)handle_p;
	return (uint16_t)(sizeof(app_mock.frame) - app_mock.position);
}
uint8_t UART_ReadRawByte(UART_Handle *handle_p)
{
	(void)handle_p;
	return app_mock.frame[app_mock.position++];
}

uart_rx_status_t UART_GetRxStatus(const UART_Handle *handle_p)
{
	(void)handle_p;
	return (uart_rx_status_t){0};
}
void Menu_Init(void)
{
}

void Menu_Update(const car_menu_model_t *model_p, uint32_t now_ms)
{
	(void)model_p;
	(void)now_ms;
}
car_menu_action_e Menu_KeyEvent(uint32_t keys)
{
	car_menu_action_e action = app_mock.action;
	(void)keys;
	app_mock.action = CAR_MENU_ACTION_NONE;
	return action;
}
void Menu_ShowMessage(const char *message_p)
{
	assert(message_p[0] != '\0');
	snprintf(app_mock.last_message, sizeof(app_mock.last_message),
		"%s", message_p);
	app_mock.messages++;
}

/** @brief 准备合法的新观测，验证不会依赖视觉帧自动启动。 */
static void Test_Frame(void)
{
	uint8_t offset = 0U;
	uint8_t sum = 0U;

	memset(app_mock.frame, 0, sizeof(app_mock.frame));
	app_mock.frame[0] = 0x55U;
	app_mock.frame[1] = 0xAAU;
	app_mock.frame[2] = app_mock.seq++;
	app_mock.frame[8] = 100U;
	app_mock.frame[9] = 100U;
	app_mock.frame[12] = 0x0DU;

	for (offset = 2U; offset <= 10U; offset++)
	{
		sum += app_mock.frame[offset];
	}

	app_mock.frame[11] = sum;
	app_mock.position = 0U;
}

#if (CAR_STAGE == CAR_STAGE_SLOPE) && (CAR_ENABLE_MENU == 1U)
/** @brief 用真实坡道判定和应用调度推进一段连续观测。 */
static void Test_Pitch(float pitch, uint16_t frames)
{
	uint16_t index = 0U;

	app_mock.pitch = pitch;

	for (index = 0U; index < frames; index++)
	{
		Test_Frame();
		app_mock.now_ms += CAR_CONTROL_PERIOD_MS;
		Car_Update();
	}
}

/** @brief 验证倒装 IMU 的上坡加速、坡顶减速和下坡独立比例。 */
static void Test_SlopeSpeeds(void)
{
	uint16_t index = 0U;
	float uphill_speed = 0.0f;

	assert(Slope_GetStatus().ready);
	Test_Pitch(0.0f, 1U);
	app_mock.action = CAR_MENU_ACTION_START;
	app_mock.key_callback(2U);
	Car_Update();
	assert(Race_GetOutput().enabled);
	Test_Pitch(0.0f, 300U);
	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		CAR_DRIVE_SPEED_MM_S) < 0.01f);
	Test_Pitch(-10.0f, 100U);
	assert(Slope_GetStatus().phase == SLOPE_UPHILL);
	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		CAR_DRIVE_SPEED_MM_S * CAR_UPHILL_SPEED_SCALE) < 0.01f);
	CarTuning_Step(CAR_TUNE_UPHILL_SCALE, true);
	Test_Pitch(-10.0f, 40U);
	uphill_speed = Race_GetOutput().drive_speed_mm_s;
	assert(fabsf(uphill_speed - CAR_DRIVE_SPEED_MM_S *
		CarTuning_Get(CAR_TUNE_UPHILL_SCALE)) < 0.01f);

	for (index = 0U; index < 100U; index++)
	{
		Test_Pitch(0.0f, 1U);

		if (Slope_GetStatus().phase == SLOPE_CREST)
		{
			break;
		}
	}

	assert(index < 100U);
	assert(Race_GetOutput().drive_speed_mm_s < uphill_speed);
	Test_Pitch(0.0f, 9U);
	assert(Slope_GetStatus().phase == SLOPE_CREST);
	/* 坡顶窗口较短，验证按减速率跟随目标，不要求瞬间降到目标。 */
	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		fmaxf(CAR_DRIVE_SPEED_MM_S * CAR_CREST_SPEED_SCALE,
			uphill_speed - CAR_DECEL_MM_S2 * 0.1f)) < 0.01f);
	Test_Pitch(10.0f, 100U);
	assert(Slope_GetStatus().phase == SLOPE_DOWNHILL);
	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		CAR_DRIVE_SPEED_MM_S * CAR_DOWNHILL_SPEED_SCALE) < 0.01f);
	Test_Pitch(0.0f, 200U);
	assert(Slope_GetStatus().phase == SLOPE_FLAT);
	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		CAR_DRIVE_SPEED_MM_S) < 0.01f);
	Race_Stop(false);
	CarTuning_Reset();
}
#endif

int main(void)
{
	uint16_t index = 0U;

	app_mock.position = sizeof(app_mock.frame);
	Car_Init();
	assert(app_mock.key_callback != NULL);
	Test_Frame();
	app_mock.now_ms = 10U;
	Car_Update();
	assert(!Race_GetOutput().enabled);

#if (CAR_ENABLE_MENU == 1U)
	/* 上下导航既不能启动，也不能停止已启动的比赛。 */
	app_mock.key_callback(0U);
	Car_Update();
	assert(!Race_GetOutput().enabled);
	app_mock.action = CAR_MENU_ACTION_START;
	app_mock.key_callback(2U);
	Car_Update();

#if (CAR_STAGE == CAR_STAGE_RECEIVE) || (CAR_STAGE == CAR_STAGE_SLOPE)
	assert(!Race_GetOutput().enabled);
	assert(app_mock.messages == 1U);
#else
	assert(Race_GetOutput().enabled);
#endif

	for (index = 0U; index < 210U; index++)
	{
		Test_Frame();
		app_mock.now_ms += CAR_CONTROL_PERIOD_MS;
		Car_Update();
	}

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	app_mock.action = CAR_MENU_ACTION_STOP;
	app_mock.key_callback(3U);
	Car_Update();
	assert(!Race_GetOutput().enabled);
	app_mock.action = CAR_MENU_ACTION_START;
	app_mock.key_callback(2U);
	Car_Update();
	assert(Race_GetOutput().enabled);
	app_mock.action = CAR_MENU_ACTION_START;
	app_mock.key_callback(2U);
	Car_Update();
	assert(Race_GetOutput().enabled);
	assert(strcmp(app_mock.last_message, "Already running") == 0);
	app_mock.key_callback(1U);
	Car_Update();
	assert(Race_GetOutput().enabled);
	app_mock.action = CAR_MENU_ACTION_STOP;
	app_mock.key_callback(3U);
	Car_Update();
	assert(!Race_GetOutput().enabled);
	app_mock.now_ms += 60U;
	app_mock.action = CAR_MENU_ACTION_START;
	app_mock.key_callback(2U);
	Car_Update();
	assert(!Race_GetOutput().enabled);
	assert(app_mock.messages > 0U);
	assert(strcmp(app_mock.last_message, "Vision not ready") == 0);
#else
	assert(!Race_GetOutput().enabled);
#endif
#else
	(void)index;
	app_mock.key_callback(0U);
	Car_Update();
	assert(Race_GetOutput().enabled);
	app_mock.key_callback(1U);
	Car_Update();
	assert(!Race_GetOutput().enabled);
#endif

#if (CAR_STAGE == CAR_STAGE_SLOPE) && (CAR_ENABLE_MENU == 1U)
	Test_SlopeSpeeds();
#endif

	printf("PASS: application input gates, stage %u, menu %u\n",
		CAR_STAGE, CAR_ENABLE_MENU);
	return 0;
}
