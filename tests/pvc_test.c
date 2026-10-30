#include "car_config.h"
#include "chassis_control.h"
#include "race_control.h"
#include "slope.h"
#include "vision.h"
#include "servo.h"
#include "motor.h"
#include "tuning/car_tuning.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** 主机执行器记录，验证真实 Chassis_Apply 的编译门控。 */
typedef struct
{
	uint32_t servo_calls;
	uint32_t motor_calls;
	uint32_t brakes;
	int32_t servo;
	float left_speed;
	float right_speed;
} actuator_mock_t;

static actuator_mock_t actuator;

static void Test_TuneValue(car_tune_id_e id, float value);

void Servo_Init(void)
{
	actuator.servo_calls++;
}
void Servo_SetValue(int32_t value)
{
	actuator.servo_calls++;
	actuator.servo = value;
}

void Motor_Init(void)
{
	actuator.motor_calls++;
}

void Motor_Brake(void)
{
	actuator.brakes += 2U;
}

void Motor_SetSpeedLR(float left_mm_s, float right_mm_s)
{
	actuator.motor_calls++;
	actuator.left_speed = left_mm_s;
	actuator.right_speed = right_mm_s;
}

/** @brief 重算人为构造帧的校验。 @param frame_p 完整帧。 @retval 无。 */
static void Test_Checksum(uint8_t *frame_p)
{
	uint8_t index = 0U;

	frame_p[11] = 0U;

	for (index = 2U; index <= 10U; index++)
	{
		frame_p[11] = (uint8_t)(frame_p[11] + frame_p[index]);
	}
}

/** @brief 连续喂入一帧，分包不改变字节流语义。
 * @param frame_p 完整字节。
 * @param now_ms 接收时刻。
 * @retval 无。
 */
static void Test_Feed(const uint8_t *frame_p, uint32_t now_ms)
{
	uint32_t index = 0U;

	for (index = 0U; index < VISION_FRAME_SIZE; index++)
	{
		Vision_FeedByte(frame_p[index], now_ms);
	}
}

/** @brief 验证有符号解码、内嵌同步字、坏帧恢复与序号新鲜度。 */
static void Test_Vision(void)
{
	uint8_t frame[] =
	{
		0x55, 0xAA, 1, 0, 0, 0xFE, 0xD4,
		0xF4, 90, 80, 0, 0x71, 0x0D
	};
	vision_status_t status = {0};
	uint32_t index = 0U;

	Vision_Init();
	Vision_FeedByte(0x99U, 0U);
	Vision_FeedByte(0x55U, 0U);

	for (index = 0U; index < sizeof(frame) - 1U; index++)
	{
		Vision_FeedByte(frame[index], 10U);
	}

	assert(Vision_GetStatus().accepted == 0U);
	Vision_FeedByte(frame[sizeof(frame) - 1U], 11U);
	status = Vision_GetStatus();
	assert(status.accepted == 1U);
	assert(fabsf(status.sample.dev + 0.3f) < 0.00001f);
	assert(fabsf(status.sample.k + 0.12f) < 0.00001f);
	Test_Feed(frame, 30U);
	assert(Vision_GetStatus().duplicates == 1U);
	assert(Vision_GetStatus().sample.received_ms == 11U);

	frame[2] = 2U;
	frame[5] = 3U;
	frame[6] = 0x55U;
	frame[7] = 0xAAU;
	Test_Checksum(frame);
	Test_Feed(frame, 40U);
	status = Vision_GetStatus();
	assert(status.accepted == 2U);
	assert(fabsf(status.sample.dev - 0.853f) < 0.00001f);
	assert(fabsf(status.sample.k + 0.86f) < 0.00001f);

	frame[2] = 3U;
	Test_Checksum(frame);
	frame[11] ^= 1U;
	Test_Feed(frame, 50U);
	assert(Vision_GetStatus().accepted == 2U);
	frame[5] = 0U;
	frame[6] = 0x0DU;
	Test_Checksum(frame);
	Test_Feed(frame, 51U);
	assert(Vision_GetStatus().sample.seq == 3U);

	frame[2] = 4U;
	frame[9] = 101U;
	Test_Checksum(frame);
	Test_Feed(frame, 52U);
	assert(Vision_GetStatus().sample.seq == 3U);
	frame[9] = 80U;
	frame[2] = 2U;
	Test_Checksum(frame);
	Test_Feed(frame, 53U);
	assert(Vision_GetStatus().sample.seq == 3U);

	Vision_Init();
	frame[2] = 255U;
	Test_Checksum(frame);
	Test_Feed(frame, UINT32_MAX - 20U);
	frame[2] = 0U;
	Test_Checksum(frame);
	Test_Feed(frame, 0U);
	assert(Vision_GetStatus().discontinuities == 0U);
	assert(Vision_GetStatus().accepted == 2U);
	frame[2] = 200U;
	Test_Checksum(frame);
	Test_Feed(frame, 1U);
	assert(Vision_GetStatus().accepted == 2U);
	Test_Feed(frame, 101U);
	assert(Vision_GetStatus().sample.seq == 200U);
}

/** @brief 生成一个新的可靠普通道路观测。 @retval 观测副本。 */
static vision_sample_t Test_Sample(void)
{
	vision_sample_t sample = {0};

	sample.sample_id = 1U;
	sample.valid = 80U;
	sample.conf = 90U;

	return sample;
}

/** @brief 区分字节到达、合法重复、尾字节/校验/范围/序号错误。 */
static void Test_VisionDiagnostics(void)
{
	uint8_t frame[VISION_FRAME_SIZE] =
	{
		0x55U, 0xAAU, 1U, 0U, 0U, 0U, 0U,
		0U, 90U, 80U, 0U, 0U, 0x0DU
	};
	vision_status_t status = {0};
	uint32_t index = 0U;

	Vision_Init();
	Vision_FeedByte(0x35U, 1U);
	Test_Checksum(frame);
	Test_Feed(frame, 10U);
	Test_Feed(frame, 20U);
	status = Vision_GetStatus();
	assert(status.bytes_received == 1U + 2U * sizeof(frame));
	assert(status.raw_length == VISION_RAW_BYTES);
	assert(status.accepted == 1U && status.duplicates == 1U);
	assert(status.checked_frames == 2U && status.sync_hits == 2U);
	assert(status.last_byte_ms == 20U && status.sample.received_ms == 10U);
	frame[2] = 2U;
	Test_Checksum(frame);
	frame[11] ^= 1U;
	Test_Feed(frame, 30U);
	assert(Vision_GetStatus().checksum_errors == 1U);
	Test_Checksum(frame);
	Test_Feed(frame, 40U);
	assert(Vision_GetStatus().accepted == 2U);
	frame[2] = 3U;
	Test_Checksum(frame);
	frame[12] = 0U;
	Test_Feed(frame, 50U);
	assert(Vision_GetStatus().tail_errors == 1U);
	frame[12] = 0x0DU;
	Test_Feed(frame, 60U);
	frame[2] = 4U;
	frame[8] = 101U;
	Test_Checksum(frame);
	Test_Feed(frame, 70U);
	assert(Vision_GetStatus().range_errors == 1U);
	frame[8] = 90U;
	Test_Checksum(frame);
	Test_Feed(frame, 80U);
	frame[2] = 2U;
	Test_Checksum(frame);
	Test_Feed(frame, 81U);
	status = Vision_GetStatus();
	assert(status.sequence_errors == 1U && status.sample.seq == 4U);
	assert(memcmp(&status.raw[VISION_RAW_BYTES - sizeof(frame)],
		frame, sizeof(frame)) == 0);

	/* 连续流跨越序号回绕，带错位噪声和坏帧仍可持续找回帧头。 */
	Vision_Init();

	for (index = 0U; index < 300U; index++)
	{
		frame[2] = (uint8_t)index;
		Test_Checksum(frame);
		Vision_FeedByte(0x55U, index * 17U);
		Vision_FeedByte(0x99U, index * 17U);
		Test_Feed(frame, index * 17U);
	}

	assert(Vision_GetStatus().accepted == 300U);
	assert(Vision_GetStatus().sample.received_ms == 299U * 17U);
}

/** @brief 验证启动条件、短暂丢线、超时与停止后不自动重启。 */
static void Test_RaceFault(void)
{
	vision_sample_t sample = Test_Sample();
	race_output_t output = {0};
	float turn = 0.0f;

	Race_Init();
	assert(!Race_Start(&sample, CAR_VISION_SLOW_MS));
	assert(Race_Start(&sample, 0U));
	sample.dev = 0.2f;
	sample.k = 0.03f;
	sample.conf = 0U;
	Race_Update(&sample, 0U, 1.0f);
	assert(Race_GetOutput().drive_speed_mm_s == 0.0f);
	Race_Update(&sample, 10U, 1.0f);
	output = Race_GetOutput();
	assert(output.enabled);
	turn = CAR_POSITION_GAIN * 0.2f + CAR_HEADING_GAIN * 0.03f;
	assert(fabsf(output.turn_target - turn) < 0.00001f);
	assert(fabsf(output.turn_right -
		fminf(turn, CAR_STEERING_SLEW_PER_S * 0.01f)) < 0.00001f);
	assert(output.drive_speed_mm_s > 0.0f);

	sample.sample_id++;
	sample.received_ms = 20U;
	sample.lost = true;
	Race_Update(&sample, 20U, 1.0f);
	assert(Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 300U;
	Race_Update(&sample, 300U, 1.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(!Race_GetOutput().enabled);
	sample.lost = false;
	sample.sample_id++;
	sample.received_ms = 310U;
	Race_Update(&sample, 310U, 1.0f);
	assert(!Race_GetOutput().enabled);
	assert(Race_Start(&sample, 310U));
	Race_Stop(false);
	sample.received_ms = 320U;
	sample.sample_id++;
	Race_Update(&sample, 320U, 1.0f);
	assert(!Race_GetOutput().enabled);

	sample = Test_Sample();
	sample.received_ms = UINT32_MAX - 50U;
	assert(Race_Start(&sample, sample.received_ms));
	Race_Update(&sample, UINT32_MAX - 40U, 1.0f);
	assert(Race_GetOutput().enabled);
	Race_Update(&sample, 249U, 1.0f);
	assert(!Race_GetOutput().enabled);

	sample = Test_Sample();
	Race_Init();
	assert(Race_Start(&sample, 0U));
	Race_Update(&sample, 10U, 0.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
}

/** @brief 验证初始斑马线屏蔽、只按新帧计数和停车锁定。 */
static void Test_Finish(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	uint32_t deadline_ms = 0U;

	Race_Init();
	sample.elem = 5U;
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 500U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
		assert(Race_GetOutput().phase == RACE_DEPART);
	}

	sample.elem = 0U;

	for (; now_ms <= CAR_START_GUARD_MS; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().phase == RACE_RUN);
	sample.elem = 5U;
	sample.sample_id++;
	sample.received_ms = now_ms;
	Race_Update(&sample, now_ms, 1.0f);
	Race_Update(&sample, now_ms + 10U, 1.0f);
	Race_Update(&sample, now_ms + 20U, 1.0f);
	assert(Race_GetOutput().phase == RACE_RUN);

	/* 留出新帧确认和当前输出减至零的时间，适配不同 PWM 默认值。 */
	deadline_ms = now_ms + 30U +
		CAR_FINISH_CONFIRM_FRAMES * CAR_CONTROL_PERIOD_MS +
		(uint32_t)ceilf(Race_GetOutput().drive_speed_mm_s * 1000.0f /
			CAR_DECEL_MM_S2) + CAR_CONTROL_PERIOD_MS;

	for (now_ms += 30U; now_ms <= deadline_ms;
		now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().phase == RACE_DONE);
	assert(Race_GetOutput().drive_speed_mm_s == 0.0f);
	assert(Race_GetOutput().turn_right == 0.0f);
	assert(Race_GetOutput().turn_target == 0.0f);
	assert(!Race_GetOutput().enabled);
}

/** @brief 验证安装基准、时间滞回、短坡跳阶段和无效姿态。 */
static void Test_Slope(void)
{
	slope_config_t config = {6.0f, 3.0f, 1.0f, 1.0f, 30U, 0U, 2U};

	Slope_Init(&config);
	Slope_Update(10.0f, 0U);
	assert(!Slope_GetStatus().ready);
	Slope_Update(10.0f, 10U);
	assert(Slope_GetStatus().ready);
	assert(Slope_GetStatus().baseline_pitch == 10.0f);
	Slope_Update(20.0f, 20U);
	Slope_Update(10.0f, 30U);
	assert(Slope_GetStatus().phase == SLOPE_FLAT);
	Slope_Update(20.0f, 40U);
	Slope_Update(20.0f, 70U);
	assert(Slope_GetStatus().phase == SLOPE_UPHILL);
	Slope_Update(10.0f, 80U);
	Slope_Update(10.0f, 110U);
	assert(Slope_GetStatus().phase == SLOPE_CREST);
	Slope_Update(0.0f, 120U);
	Slope_Update(0.0f, 150U);
	assert(Slope_GetStatus().phase == SLOPE_DOWNHILL);
	Slope_Update(10.0f, 160U);
	Slope_Update(10.0f, 190U);
	assert(Slope_GetStatus().phase == SLOPE_FLAT);
	Slope_Update(NAN, 200U);
	assert(!Slope_GetStatus().ready);
}

/** @brief 验证四级输出裁剪和统一停止门控。 */
static void Test_Chassis(void)
{
	race_output_t output =
	{
		.phase = RACE_RUN,
		.turn_right = 20.0f,
		.drive_speed_mm_s = 500.0f,
		.enabled = true,
		.turn_target = 60.0f
	};
	uint32_t previous_calls = 0U;

	/* 本用例只验证输出门控，关闭差速以免受默认比例影响。 */
	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_DIFF_MAX_RATIO, 0.0f);
	memset(&actuator, 0, sizeof(actuator));
	Chassis_Init();
	Chassis_Apply(&output);

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	/* 底盘使用平滑输出，不能误用未完成跟随的目标。 */
	assert(actuator.servo == CAR_SERVO_CENTER + 20 * CAR_SERVO_DIRECTION);
#endif

	previous_calls = actuator.motor_calls;
	output.enabled = false;
	Chassis_Apply(&output);

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	assert(actuator.servo_calls > 0U);
	assert(actuator.servo == CAR_SERVO_CENTER);
#else
	assert(actuator.servo_calls == 0U);
#endif

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
	assert(previous_calls > 0U);
	assert(actuator.motor_calls == previous_calls);
	assert(actuator.left_speed == 500.0f);
	assert(actuator.right_speed == 500.0f);
	assert(actuator.brakes == 2U);
#else
	assert(previous_calls == 0U);
	assert(actuator.brakes == 0U);
#endif

	CarTuning_Reset();
}

/** @brief 调低直道目标不能在转弯时加速，短时丢线也遵守新转向限幅。 */
static void Test_TuningOutputLimits(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t index = 0U;
	uint32_t now_ms = 0U;
	float previous_speed = 0.0f;

	CarTuning_Reset();
	sample.dev = 1.0f;
	Race_Init();
	assert(Race_Start(&sample, now_ms));

	for (now_ms = 10U; now_ms <= 2000U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().turn_right > 0.0f);
	previous_speed = Race_GetOutput().drive_speed_mm_s;

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_DRIVE_SPEED, false);
		CarTuning_Step(CAR_TUNE_STEERING_LIMIT, false);
	}

	/* 观测暂时不可用时仍立即应用新的转向限幅。 */
	sample.sample_id++;
	sample.received_ms = now_ms;
	sample.lost = true;
	Race_Update(&sample, now_ms, 1.0f);
	assert(Race_GetOutput().turn_right == 0.0f);

	/* 恢复转向输出并令阈值为零，检验 Drive=0 也不能被转弯档覆盖。 */
	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_STEERING_LIMIT, true);
		CarTuning_Step(CAR_TUNE_TURN_THRESHOLD, false);
	}

	sample.lost = false;

	for (; now_ms <= 2500U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
		assert(Race_GetOutput().drive_speed_mm_s <= previous_speed);
		previous_speed = Race_GetOutput().drive_speed_mm_s;
	}

	assert(Race_GetOutput().drive_speed_mm_s == 0.0f);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 在现有步长内设置测试参数。 */
static void Test_TuneValue(car_tune_id_e id, float value)
{
	uint32_t index = 0U;

	for (index = 0U; index < 200U &&
		fabsf(CarTuning_Get(id) - value) > 0.00001f; index++)
	{
		CarTuning_Step(id, CarTuning_Get(id) < value);
	}

	assert(fabsf(CarTuning_Get(id) - value) < 0.00001f);
}

/** @brief 用新观测推进真实控制器。 */
static void Test_ControlStep(vision_sample_t *sample_p, uint32_t now_ms)
{
	sample_p->sample_id++;
	sample_p->received_ms = now_ms;
	Race_Update(sample_p, now_ms, 1.0f);
}

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
/** @brief 差速跟随平滑转向，连续介入且不突破电机速度及停止门控。 */
static void Test_Differential(void)
{
	race_output_t output =
	{
		.phase = RACE_RUN,
		.enabled = true,
		.drive_speed_mm_s = 800.0f,
		.turn_right = 20.0f,
		.turn_target = -60.0f
	};
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	uint32_t previous_calls = 0U;
	float previous_diff = 0.0f;

	/* 固定本用例假定的差速工作点，避免受默认参数变化影响。 */
	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_DIFF_THRESHOLD, 25.0f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 60.0f);
	Test_TuneValue(CAR_TUNE_DIFF_MAX_RATIO, 0.20f);
	Chassis_Apply(&output);
	assert(actuator.left_speed == 800.0f);
	assert(actuator.right_speed == 800.0f);
	output.turn_right = CarTuning_Get(CAR_TUNE_DIFF_THRESHOLD);
	Chassis_Apply(&output);
	assert(actuator.left_speed == actuator.right_speed);
	output.turn_right += 0.01f;
	Chassis_Apply(&output);
	assert(actuator.left_speed > actuator.right_speed);
	assert(actuator.left_speed - actuator.right_speed < 1.0f);
	output.turn_right = 42.5f;
	Chassis_Apply(&output);
	assert(fabsf(actuator.left_speed - 880.0f) < 0.01f);
	assert(fabsf(actuator.right_speed - 720.0f) < 0.01f);
	output.turn_right = 60.0f;
	Chassis_Apply(&output);
	assert(fabsf(actuator.left_speed - 960.0f) < 0.01f);
	assert(fabsf(actuator.right_speed - 640.0f) < 0.01f);
	output.turn_right = -60.0f;
	Chassis_Apply(&output);
	assert(fabsf(actuator.left_speed - 640.0f) < 0.01f);
	assert(fabsf(actuator.right_speed - 960.0f) < 0.01f);
	Test_TuneValue(CAR_TUNE_DIFF_MAX_RATIO, 0.30f);
	Chassis_Apply(&output);
	assert(fabsf(actuator.left_speed - 560.0f) < 0.01f);
	assert(fabsf(actuator.right_speed - 1040.0f) < 0.01f);
	output.drive_speed_mm_s = MOTOR_MAX_SPEED_MM_S;
	Chassis_Apply(&output);
	assert(actuator.right_speed <= MOTOR_MAX_SPEED_MM_S);
	assert(fabsf(actuator.left_speed / actuator.right_speed -
		0.70f / 1.30f) < 0.00001f);
	output.drive_speed_mm_s = 10.0f;
	Chassis_Apply(&output);
	assert(actuator.left_speed > 0.0f && actuator.right_speed > 0.0f);
	Test_TuneValue(CAR_TUNE_DIFF_MAX_RATIO, 0.0f);
	Chassis_Apply(&output);
	assert(actuator.left_speed == actuator.right_speed);
	Test_TuneValue(CAR_TUNE_DIFF_MAX_RATIO, 0.50f);
	Test_TuneValue(CAR_TUNE_DIFF_THRESHOLD, 60.0f);
	Chassis_Apply(&output);
	assert(actuator.left_speed == actuator.right_speed);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 0.0f);
	Test_TuneValue(CAR_TUNE_DIFF_THRESHOLD, 0.0f);
	Chassis_Apply(&output);
	assert(actuator.left_speed == actuator.right_speed);
	previous_calls = actuator.motor_calls;
	output.drive_speed_mm_s = 0.0f;
	Chassis_Apply(&output);
	output.drive_speed_mm_s = NAN;
	Chassis_Apply(&output);
	output.drive_speed_mm_s = INFINITY;
	Chassis_Apply(&output);
	output.enabled = false;
	output.drive_speed_mm_s = 800.0f;
	Chassis_Apply(&output);
	assert(actuator.motor_calls == previous_calls);

	/* 急弯反号时跟随输出方向，丢线后保持比例并随降速减小差速量。 */
	CarTuning_Reset();
	Race_Init();
	sample.dev = 1.0f;
	sample.k = 0.10f;
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 2000U; now_ms += 10U)
	{
		Test_ControlStep(&sample, now_ms);
	}

	sample.dev = -1.0f;
	sample.k = -0.10f;
	Test_ControlStep(&sample, now_ms);
	output = Race_GetOutput();
	assert(output.turn_target < 0.0f && output.turn_right > 0.0f);
	Chassis_Apply(&output);
	assert(actuator.left_speed > actuator.right_speed);
	previous_diff = actuator.left_speed - actuator.right_speed;
	sample.lost = true;
	now_ms += 10U;
	Test_ControlStep(&sample, now_ms);
	output = Race_GetOutput();
	Chassis_Apply(&output);
	assert(actuator.left_speed > actuator.right_speed);
	assert(actuator.left_speed - actuator.right_speed < previous_diff);
	previous_calls = actuator.motor_calls;
	Test_ControlStep(&sample, now_ms + CAR_VISION_STOP_MS - 10U);
	output = Race_GetOutput();
	assert(!output.enabled);
	Chassis_Apply(&output);
	assert(actuator.motor_calls == previous_calls);
	CarTuning_Reset();
}
#endif

/** @brief 验证双向阶跃、真实 dt、丢线、限幅及停止重启。 */
static void Test_SteeringSlew(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	float previous = 0.0f;

	CarTuning_Reset();
	assert(CarTuning_Get(CAR_TUNE_STEERING_SLEW) ==
		CAR_STEERING_SLEW_PER_S);
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 800.0f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 60.0f);
	sample.dev = 1.0f;
	sample.k = 0.1f;
	Race_Init();
	assert(Race_Start(&sample, 0U));
	Race_Update(&sample, 0U, 1.0f);
	assert(Race_GetOutput().turn_target == 60.0f);
	assert(Race_GetOutput().turn_right == 0.0f);

	for (now_ms = 10U; now_ms <= 80U; now_ms += 10U)
	{
		Test_ControlStep(&sample, now_ms);
		assert(Race_GetOutput().turn_right <= previous + 8.0f);
		assert(Race_GetOutput().turn_right <= 60.0f);
		previous = Race_GetOutput().turn_right;
	}

	assert(previous == 60.0f);
	sample.dev = -1.0f;
	sample.k = -0.1f;

	for (; now_ms <= 230U; now_ms += 10U)
	{
		Test_ControlStep(&sample, now_ms);
		assert(Race_GetOutput().turn_target == -60.0f);
		assert(Race_GetOutput().turn_right >= previous - 8.0f);
		previous = Race_GetOutput().turn_right;
	}

	assert(previous == -60.0f);

	/* 手动停止清零；新启动不能继承上一次 -60 的状态。 */
	Race_Stop(false);
	assert(Race_GetOutput().turn_right == 0.0f);
	assert(Race_GetOutput().turn_target == 0.0f);
	sample.received_ms = 240U;
	assert(Race_Start(&sample, 240U));
	sample.dev = 1.0f;
	sample.k = 0.1f;
	Test_ControlStep(&sample, 250U);
	assert(Race_GetOutput().turn_right == 8.0f);
	Test_ControlStep(&sample, 255U);
	assert(Race_GetOutput().turn_right == 12.0f);
	Test_ControlStep(&sample, 255U);
	assert(Race_GetOutput().turn_right == 12.0f);

	/* 调大变化率立即生效；偶发 30 ms 卡顿最多按 20 ms 推进。 */
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 1000.0f);
	Test_ControlStep(&sample, 265U);
	assert(Race_GetOutput().turn_right == 22.0f);
	Test_ControlStep(&sample, 295U);
	assert(Race_GetOutput().turn_right == 42.0f);

	/* 明确丢线时保持并线性衰减，300 ms 后才故障。 */
	sample.lost = true;
	Test_ControlStep(&sample, 305U);
	assert(fabsf(Race_GetOutput().turn_target -
		42.0f * (1.0f - 10.0f / (float)CAR_LOST_DECAY_MS)) < 0.01f);
	assert(fabsf(Race_GetOutput().turn_right -
		Race_GetOutput().turn_target) < 0.01f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 20.0f);
	Race_Update(&sample, 315U, 1.0f);
	assert(Race_GetOutput().turn_right == 20.0f);
	assert(Race_GetOutput().turn_target == 20.0f);
	sample.lost = false;
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 60.0f);
	Test_ControlStep(&sample, 325U);
	assert(Race_GetOutput().turn_right == 30.0f);

	/* SLOW=80：50 ms 仍在跟踪；80 ms 后冻结当前舵量，300 ms 故障。 */
	Race_Update(&sample, 375U, 1.0f);
	assert(Race_GetOutput().turn_right == 50.0f);
	Race_Update(&sample, 405U, 1.0f);
	assert(Race_GetOutput().turn_right == 50.0f);
	assert(Race_GetOutput().turn_target == 50.0f);
	Race_Update(&sample, 625U, 1.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(Race_GetOutput().turn_right == 0.0f);
	assert(Race_GetOutput().turn_target == 0.0f);

	/* 设为 0 恢复直通；上下界与恢复默认均可通过调参入口验证。 */
	sample.received_ms = 635U;
	assert(Race_Start(&sample, 635U));
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 0.0f);
	Test_ControlStep(&sample, 645U);
	assert(Race_GetOutput().turn_right == 60.0f);
	CarTuning_Step(CAR_TUNE_STEERING_SLEW, false);
	assert(CarTuning_Get(CAR_TUNE_STEERING_SLEW) == 0.0f);
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 3000.0f);
	CarTuning_Step(CAR_TUNE_STEERING_SLEW, true);
	assert(CarTuning_Get(CAR_TUNE_STEERING_SLEW) == 3000.0f);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 目标提前触发降速，回正期间也不能过早恢复直道速度。 */
static void Test_SteeringSpeed(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	float previous_speed = 0.0f;

	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 800.0f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 60.0f);
	Test_TuneValue(CAR_TUNE_TURN_THRESHOLD, 30.0f);
	Test_TuneValue(CAR_TUNE_DRIVE_SPEED, 600.0f);
	Test_TuneValue(CAR_TUNE_TURN_SPEED, 400.0f);
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 1600U; now_ms += 10U)
	{
		Test_ControlStep(&sample, now_ms);
	}

	previous_speed = Race_GetOutput().drive_speed_mm_s;
	sample.dev = 1.0f;
	sample.k = 0.1f;
	Test_ControlStep(&sample, now_ms);
	assert(Race_GetOutput().turn_right == 8.0f);
	assert(Race_GetOutput().turn_target == 60.0f);
	assert(Race_GetOutput().drive_speed_mm_s < previous_speed);

	for (now_ms += 10U; now_ms <= 1800U; now_ms += 10U)
	{
		Test_ControlStep(&sample, now_ms);
	}

	previous_speed = Race_GetOutput().drive_speed_mm_s;
	sample.dev = 0.0f;
	sample.k = 0.0f;
	Test_ControlStep(&sample, now_ms);
	assert(Race_GetOutput().turn_target == 0.0f);
	assert(Race_GetOutput().turn_right == 52.0f);
	assert(Race_GetOutput().drive_speed_mm_s <= previous_speed);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 上坡比例可放大目标，但不能突破速度及视觉降级上限。 */
static void Test_SlopeDriveLimits(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;

	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_DRIVE_SPEED, 1000.0f);
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 4000U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.25f);
	}

	assert(fabsf(Race_GetOutput().drive_speed_mm_s - 1250.0f) < 0.01f);
	Test_TuneValue(CAR_TUNE_DRIVE_SPEED, MOTOR_MAX_SPEED_MM_S);

	for (; now_ms <= 7000U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 2.0f);
		assert(Race_GetOutput().drive_speed_mm_s <= MOTOR_MAX_SPEED_MM_S);
	}

	assert(fabsf(Race_GetOutput().drive_speed_mm_s -
		MOTOR_MAX_SPEED_MM_S) < 0.01f);
	Race_Stop(false);
	Test_TuneValue(CAR_TUNE_DEGRADED_SPEED, 20.0f);
	sample = Test_Sample();
	assert(Race_Start(&sample, 0U));
	sample.lost = true;

	for (now_ms = 10U; now_ms < CAR_VISION_STOP_MS; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 2.0f);
		assert(Race_GetOutput().drive_speed_mm_s <= 20.0f);
	}

	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 验证调参默认值、步进钳位以及运行时立即生效。 */
static void Test_Tuning(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t index = 0U;
	float turn = 0.0f;

	CarTuning_Reset();
	assert(fabsf(CarTuning_Get(CAR_TUNE_DRIVE_SPEED) - CAR_DRIVE_SPEED_MM_S) <
		0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_TURN_SPEED) - CAR_TURN_SPEED_MM_S) <
		0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_DEGRADED_SPEED) -
		CAR_DEGRADED_SPEED_MM_S) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_POSITION_GAIN) -
		CAR_POSITION_GAIN) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_HEADING_GAIN) -
		CAR_HEADING_GAIN) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_STEERING_LIMIT) -
		CAR_STEERING_LIMIT) < 0.00001f);
	assert((uint8_t)CarTuning_Get(CAR_TUNE_VALID_MIN) == CAR_VALID_MIN);
	assert(fabsf(CarTuning_Get(CAR_TUNE_TURN_THRESHOLD) -
		CAR_TURN_SLOW_THRESHOLD) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_UPHILL_SCALE) -
		CAR_UPHILL_SPEED_SCALE) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_CREST_SCALE) -
		CAR_CREST_SPEED_SCALE) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_DOWNHILL_SCALE) -
		CAR_DOWNHILL_SPEED_SCALE) < 0.00001f);
	assert(CarTuning_Get(CAR_TUNE_DIFF_THRESHOLD) == CAR_DIFF_THRESHOLD);
	assert(fabsf(CarTuning_Get(CAR_TUNE_DIFF_MAX_RATIO) -
		CAR_DIFF_MAX_RATIO) < 0.00001f);

	/* 连续步进越过上下限时钳位。 */
	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_DRIVE_SPEED, true);
	}

	assert(fabsf(CarTuning_Get(CAR_TUNE_DRIVE_SPEED) -
		MOTOR_MAX_SPEED_MM_S) < 0.01f);

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_DRIVE_SPEED, false);
	}

	assert(CarTuning_Get(CAR_TUNE_DRIVE_SPEED) == 0.0f);

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_UPHILL_SCALE, false);
		CarTuning_Step(CAR_TUNE_CREST_SCALE, false);
		CarTuning_Step(CAR_TUNE_DOWNHILL_SCALE, false);
	}

	assert(fabsf(CarTuning_Get(CAR_TUNE_UPHILL_SCALE) - 0.10f) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_CREST_SCALE) - 0.10f) < 0.00001f);
	assert(fabsf(CarTuning_Get(CAR_TUNE_DOWNHILL_SCALE) - 0.10f) < 0.00001f);

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_UPHILL_SCALE, true);
		CarTuning_Step(CAR_TUNE_CREST_SCALE, true);
		CarTuning_Step(CAR_TUNE_DOWNHILL_SCALE, true);
	}

	assert(CarTuning_Get(CAR_TUNE_UPHILL_SCALE) == 2.0f);
	assert(CarTuning_Get(CAR_TUNE_CREST_SCALE) == 1.0f);
	assert(CarTuning_Get(CAR_TUNE_DOWNHILL_SCALE) == 1.0f);

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_DIFF_THRESHOLD, false);
		CarTuning_Step(CAR_TUNE_DIFF_MAX_RATIO, false);
	}

	assert(CarTuning_Get(CAR_TUNE_DIFF_THRESHOLD) == 0.0f);
	assert(CarTuning_Get(CAR_TUNE_DIFF_MAX_RATIO) == 0.0f);

	for (index = 0U; index < 512U; index++)
	{
		CarTuning_Step(CAR_TUNE_DIFF_THRESHOLD, true);
		CarTuning_Step(CAR_TUNE_DIFF_MAX_RATIO, true);
	}

	assert(CarTuning_Get(CAR_TUNE_DIFF_THRESHOLD) == 100.0f);
	assert(CarTuning_Get(CAR_TUNE_DIFF_MAX_RATIO) == 0.50f);
	CarTuning_Reset();

	/* 运行中修改 KP 后下一次更新即采用新增益。 */
	sample.dev = 0.2f;
	sample.k = 0.03f;
	Race_Init();
	assert(Race_Start(&sample, 0U));
	Race_Update(&sample, 10U, 1.0f);
	turn = Race_GetOutput().turn_target;
	assert(fabsf(turn - (CAR_POSITION_GAIN * 0.2f +
		CAR_HEADING_GAIN * 0.03f)) < 0.00001f);
	CarTuning_Step(CAR_TUNE_POSITION_GAIN, true);
	Race_Update(&sample, 20U, 1.0f);
	assert(Race_GetOutput().turn_target > turn);
	assert(fabsf(Race_GetOutput().turn_target -
		(CarTuning_Get(CAR_TUNE_POSITION_GAIN) * 0.2f +
		CarTuning_Get(CAR_TUNE_HEADING_GAIN) * 0.03f)) < 0.00001f);
	Race_Stop(false);

	CarTuning_Reset();
}

/** @brief 明确丢线保持并线性衰减，150 ms 内不故障。 */
static void Test_LostHoldDecay(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	float held_turn = 0.0f;

	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 800.0f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 80.0f);
	sample.dev = 1.0f;
	sample.k = 0.0f;
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 500U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	held_turn = Race_GetOutput().turn_right;
	assert(held_turn > 60.0f);

	/* lost 从最后一个可用帧 500 ms 起算。 */
	now_ms = 510U;
	sample.sample_id++;
	sample.received_ms = now_ms;
	sample.lost = true;
	Race_Update(&sample, now_ms, 1.0f);
	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().turn_right < held_turn);
	assert(Race_GetOutput().turn_right > 0.0f);

	for (now_ms = 520U; now_ms <= 650U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().phase != RACE_FAULT);
	assert(Race_GetOutput().turn_right < held_turn * 0.6f);

	now_ms = 800U;
	sample.sample_id++;
	sample.received_ms = now_ms;
	Race_Update(&sample, now_ms, 1.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(!Race_GetOutput().enabled);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 故障后连续 3 个新的可用帧自动恢复，重复/丢线帧不计数。 */
static void Test_AutoRecovery(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;

	CarTuning_Reset();
	Race_Init();
	assert(Race_Start(&sample, 0U));
	sample.lost = true;

	for (now_ms = 10U; now_ms < CAR_VISION_STOP_MS; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	now_ms = CAR_VISION_STOP_MS;
	sample.sample_id++;
	sample.received_ms = now_ms;
	Race_Update(&sample, now_ms, 1.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(!Race_GetOutput().enabled);

	/* IMU 无效帧不累计恢复计数。 */
	sample.lost = false;
	sample.sample_id++;
	sample.received_ms = 310U;
	Race_Update(&sample, 310U, 0.0f);
	assert(!Race_GetOutput().enabled);

	sample.sample_id++;
	sample.received_ms = 320U;
	Race_Update(&sample, 320U, 1.0f);
	assert(!Race_GetOutput().enabled);

	/* 重复 seq 不计数。 */
	sample.received_ms = 330U;
	Race_Update(&sample, 330U, 1.0f);
	assert(!Race_GetOutput().enabled);

	/* 新的丢线帧清零恢复计数。 */
	sample.lost = true;
	sample.sample_id++;
	sample.received_ms = 340U;
	Race_Update(&sample, 340U, 1.0f);
	assert(!Race_GetOutput().enabled);

	sample.lost = false;
	sample.sample_id++;
	sample.received_ms = 350U;
	Race_Update(&sample, 350U, 1.0f);
	assert(!Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 360U;
	Race_Update(&sample, 360U, 1.0f);
	assert(!Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 370U;
	Race_Update(&sample, 370U, 1.0f);
	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().phase == RACE_RUN);

	/* 人工停止后即使视觉恢复也不能自动重新起步。 */
	Race_Stop(false);
	for (now_ms = 380U; now_ms <= 400U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
		assert(!Race_GetOutput().enabled);
	}

	CarTuning_Reset();
}

/** @brief 故障帧被消费，同一 sample_id 不能重复累计恢复计数。 */
static void Test_RecoveryFrameConsumption(void)
{
	vision_sample_t sample = Test_Sample();

	CarTuning_Reset();
	Race_Init();
	assert(Race_Start(&sample, 0U));

	/* IMU 无效触发故障，当前帧可用且是 fresh。 */
	Race_Update(&sample, 10U, 0.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);

	/* 同一帧在后续周期不能反复计入恢复。 */
	Race_Update(&sample, 20U, 1.0f);
	Race_Update(&sample, 30U, 1.0f);
	Race_Update(&sample, 40U, 1.0f);
	assert(!Race_GetOutput().enabled);

	/* 三个新的可用帧后才恢复。 */
	sample.sample_id++;
	sample.received_ms = 50U;
	Race_Update(&sample, 50U, 1.0f);
	assert(!Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 60U;
	Race_Update(&sample, 60U, 1.0f);
	assert(!Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 70U;
	Race_Update(&sample, 70U, 1.0f);
	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().phase == RACE_RUN);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief valid 只缩放转向权限，低 valid 不阻止启动或触发故障。 */
static void Test_ValidAuthority(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	float full = 0.0f;

	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 0.0f);
	Test_TuneValue(CAR_TUNE_STEERING_LIMIT, 80.0f);
	sample.dev = 0.5f;
	sample.k = 0.0f;
	sample.valid = 10U;
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 200U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
		assert(Race_GetOutput().enabled);
		assert(Race_GetOutput().phase != RACE_FAULT);
	}

	full = CAR_POSITION_GAIN * 0.5f;
	assert(fabsf(Race_GetOutput().turn_target - full * 0.625f) < 0.01f);

	sample.valid = 0U;
	sample.sample_id++;
	sample.received_ms = 210U;
	Race_Update(&sample, 210U, 1.0f);
	assert(fabsf(Race_GetOutput().turn_target - full * 0.5f) < 0.01f);

	sample.valid = CAR_VALID_MIN;
	sample.sample_id++;
	sample.received_ms = 220U;
	Race_Update(&sample, 220U, 1.0f);
	assert(fabsf(Race_GetOutput().turn_target - full) < 0.01f);
	Race_Stop(false);

	/* elem=6 与 lost=1 一样保持并衰减，不立即故障。 */
	CarTuning_Reset();
	Test_TuneValue(CAR_TUNE_STEERING_SLEW, 0.0f);
	sample = Test_Sample();
	sample.dev = 1.0f;
	sample.k = 0.0f;
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 100U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().turn_target > 60.0f);
	sample.elem = 6U;
	sample.sample_id++;
	sample.received_ms = 110U;
	Race_Update(&sample, 110U, 1.0f);
	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().phase != RACE_FAULT);
	assert(Race_GetOutput().turn_target < 70.0f);
	assert(Race_GetOutput().turn_target > 0.0f);
	Race_Stop(false);
	CarTuning_Reset();
}

/** @brief 终点停车过程中故障，恢复后仍执行停车而不是重新起步。 */
static void Test_StoppingRecovery(void)
{
	vision_sample_t sample = Test_Sample();
	uint32_t now_ms = 0U;
	uint8_t index = 0U;

	CarTuning_Reset();
	sample.elem = 0U;
	Race_Init();
	assert(Race_Start(&sample, 0U));

	for (now_ms = 10U; now_ms <= 2100U; now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().phase == RACE_RUN);

	sample.elem = 5U;
	sample.conf = 90U;

	for (index = 0U; index < CAR_FINISH_CONFIRM_FRAMES; index++)
	{
		now_ms += 10U;
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().phase == RACE_STOPPING);

	/* 停车过程中 IMU 失效，记录故障前阶段。 */
	now_ms += 10U;
	sample.sample_id++;
	sample.received_ms = now_ms;
	Race_Update(&sample, now_ms, 0.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(!Race_GetOutput().enabled);

	sample.elem = 0U;

	for (index = 0U; index < CAR_RECOVER_FRAMES; index++)
	{
		now_ms += 10U;
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().enabled);
	assert(Race_GetOutput().phase == RACE_STOPPING);

	now_ms += 10U;
	sample.sample_id++;
	sample.received_ms = now_ms;
	Race_Update(&sample, now_ms, 1.0f);
	assert(!Race_GetOutput().enabled);
	assert(Race_GetOutput().phase == RACE_DONE);
	CarTuning_Reset();
}

int main(void)
{
	Test_Vision();
	Test_VisionDiagnostics();
	Test_SteeringSlew();
	Test_SteeringSpeed();
	Test_SlopeDriveLimits();
	Test_RaceFault();
	Test_LostHoldDecay();
	Test_AutoRecovery();
	Test_RecoveryFrameConsumption();
	Test_ValidAuthority();
	Test_StoppingRecovery();
	Test_TuningOutputLimits();
	Test_Finish();
	Test_Slope();
	Test_Chassis();
#if (CAR_STAGE >= CAR_STAGE_DRIVE)
	Test_Differential();
#endif
	Test_Tuning();
	printf("PASS: PVC behavior and output gates, stage %u\n", CAR_STAGE);

	return 0;
}
