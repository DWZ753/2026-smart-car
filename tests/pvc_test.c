#include "car_config.h"
#include "chassis_control.h"
#include "race_control.h"
#include "slope.h"
#include "vision.h"
#include "servo.h"
#include "tb6612.h"
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
	int32_t left_pwm;
	int32_t right_pwm;
} actuator_mock_t;

static actuator_mock_t actuator;

void Servo_Init(void)
{
	actuator.servo_calls++;
}
void Servo_SetValue(int32_t value)
{
	actuator.servo_calls++;
	actuator.servo = value;
}

void TB6612_Init(void)
{
	actuator.motor_calls++;
}

void TB6612_A_Stop(void)
{
	actuator.motor_calls++;
}

void TB6612_B_Stop(void)
{
	actuator.motor_calls++;
}

void TB6612_A_Brake(void)
{
	actuator.brakes++;
}

void TB6612_B_Brake(void)
{
	actuator.brakes++;
}
void TB6612_A_Forward(uint32_t duty)
{
	actuator.motor_calls++;
	actuator.left_pwm = (int32_t)duty;
}

void TB6612_A_Backward(uint32_t duty)
{
	actuator.motor_calls++;
	actuator.left_pwm = -(int32_t)duty;
}

void TB6612_B_Forward(uint32_t duty)
{
	actuator.motor_calls++;
	actuator.right_pwm = (int32_t)duty;
}

void TB6612_B_Backward(uint32_t duty)
{
	actuator.motor_calls++;
	actuator.right_pwm = -(int32_t)duty;
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

	Race_Init();
	assert(!Race_Start(&sample, CAR_VISION_SLOW_MS));
	assert(Race_Start(&sample, 0U));
	sample.dev = 0.2f;
	sample.k = 0.03f;
	sample.conf = 0U;
	Race_Update(&sample, 0U, 1.0f);
	assert(Race_GetOutput().drive_pwm == 0.0f);
	Race_Update(&sample, 10U, 1.0f);
	output = Race_GetOutput();
	assert(output.enabled);
	assert(fabsf(output.turn_right - 13.0f) < 0.00001f);
	assert(output.drive_pwm > 0.0f);

	sample.sample_id++;
	sample.received_ms = 20U;
	sample.lost = true;
	Race_Update(&sample, 20U, 1.0f);
	assert(Race_GetOutput().enabled);
	sample.sample_id++;
	sample.received_ms = 100U;
	Race_Update(&sample, 100U, 1.0f);
	assert(Race_GetOutput().phase == RACE_FAULT);
	assert(!Race_GetOutput().enabled);
	sample.lost = false;
	sample.sample_id++;
	sample.received_ms = 110U;
	Race_Update(&sample, 110U, 1.0f);
	assert(!Race_GetOutput().enabled);
	assert(Race_Start(&sample, 110U));
	Race_Stop(false);
	sample.received_ms = 120U;
	sample.sample_id++;
	Race_Update(&sample, 120U, 1.0f);
	assert(!Race_GetOutput().enabled);

	sample = Test_Sample();
	sample.received_ms = UINT32_MAX - 50U;
	assert(Race_Start(&sample, sample.received_ms));
	Race_Update(&sample, UINT32_MAX - 40U, 1.0f);
	assert(Race_GetOutput().enabled);
	Race_Update(&sample, 49U, 1.0f);
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

	for (now_ms += 30U; now_ms <= CAR_START_GUARD_MS + 250U;
		now_ms += 10U)
	{
		sample.sample_id++;
		sample.received_ms = now_ms;
		Race_Update(&sample, now_ms, 1.0f);
	}

	assert(Race_GetOutput().phase == RACE_DONE);
	assert(Race_GetOutput().drive_pwm == 0.0f);
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
	race_output_t output = {RACE_RUN, 20.0f, 500.0f, true};
	uint32_t previous_calls = 0U;

	memset(&actuator, 0, sizeof(actuator));
	Chassis_Init();
	Chassis_Apply(&output);
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
	assert(actuator.left_pwm == 500 * CAR_LEFT_DIRECTION);
	assert(actuator.right_pwm == 500 * CAR_RIGHT_DIRECTION);
	assert(actuator.brakes == 2U);
#else
	assert(previous_calls == 0U);
	assert(actuator.brakes == 0U);
#endif
}

int main(void)
{
	Test_Vision();
	Test_VisionDiagnostics();
	Test_RaceFault();
	Test_Finish();
	Test_Slope();
	Test_Chassis();
	printf("PASS: PVC behavior and output gates, stage %u\n", CAR_STAGE);

	return 0;
}
