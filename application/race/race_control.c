#include "race_control.h"
#include "car_config.h"
#include "tuning/car_tuning.h"
#include "motor.h"
#include <math.h>
#include <string.h>

/** 计数只由新观测推进，所有时间差均使用无符号回绕运算。 */
typedef struct
{
	race_output_t output;
	uint32_t started_ms;
	uint32_t last_update_ms;
	uint32_t last_good_ms;
	uint32_t last_sample_id;
	uint8_t clear_frames;
	uint8_t finish_frames;
	bool finish_armed;
} race_state_t;

static race_state_t race_state;

/** @brief 判断几何可用于跟踪。
 * @param sample_p 当前观测。
 * @param now_ms 当前时刻。
 * @retval true 有观测、未过期且道路几何可靠。
 */
static bool Race_Usable(const vision_sample_t *sample_p, uint32_t now_ms)
{
	return sample_p->sample_id != 0U && !sample_p->lost &&
		sample_p->elem != 6U &&
		sample_p->valid >= (uint8_t)CarTuning_Get(CAR_TUNE_VALID_MIN) &&
		(uint32_t)(now_ms - sample_p->received_ms) < CAR_VISION_STOP_MS;
}

void Race_Init(void)
{
	memset(&race_state, 0, sizeof(race_state));
	race_state.output.phase = RACE_READY;
}

void Race_Stop(bool fault)
{
	race_state.output.enabled = false;
	race_state.output.drive_speed_mm_s = 0.0f;
	race_state.output.turn_right = 0.0f;
	race_state.output.turn_target = 0.0f;
	race_state.output.phase = fault ? RACE_FAULT : RACE_READY;
}

bool Race_Start(const vision_sample_t *sample_p, uint32_t now_ms)
{
	if (race_state.output.enabled || !Race_Usable(sample_p, now_ms) ||
		(uint32_t)(now_ms - sample_p->received_ms) >= CAR_VISION_SLOW_MS)
	{
		return false;
	}

	Race_Init();
	race_state.output.enabled = true;
	race_state.output.phase = RACE_DEPART;
	race_state.started_ms = now_ms;
	race_state.last_update_ms = now_ms;
	race_state.last_good_ms = sample_p->received_ms;

	return true;
}

/** @brief 根据独立新帧确认起点离开和返程标记。
 * @param sample_p 当前可靠新观测。
 * @param now_ms 当前时刻。
 * @retval 无。
 */
static void Race_UpdateMarker(const vision_sample_t *sample_p,
	uint32_t now_ms)
{
	bool zebra = sample_p->elem == 5U &&
		sample_p->conf >= CAR_ELEMENT_CONF_MIN;

	if (!race_state.finish_armed)
	{
		if (sample_p->elem == 5U)
		{
			race_state.clear_frames = 0U;
		}
		else if (race_state.clear_frames < CAR_MARK_CONFIRM_FRAMES)
		{
			race_state.clear_frames++;
		}

		if (race_state.clear_frames >= CAR_MARK_CONFIRM_FRAMES &&
			(uint32_t)(now_ms - race_state.started_ms) >=
			CAR_START_GUARD_MS)
		{
			race_state.finish_armed = true;
			race_state.output.phase = RACE_RUN;
		}

		return;
	}

	if (!zebra)
	{
		race_state.finish_frames = 0U;
	}
	else if (race_state.finish_frames < CAR_FINISH_CONFIRM_FRAMES)
	{
		race_state.finish_frames++;
	}

	if (race_state.finish_frames >= CAR_FINISH_CONFIRM_FRAMES)
	{
		race_state.output.phase = RACE_STOPPING;
	}
}

/** @brief 跟随可靠目标，按时间限制输出变化，丢线时保持当前输出。
 * @param sample_p 最新观测。
 * @param tracking true 当前几何可靠且未进入超时降速区间。
 * @param elapsed_ms 距离上次控制的毫秒数。
 * @retval 无。
 */
static void Race_UpdateSteering(const vision_sample_t *sample_p,
	bool tracking, uint32_t elapsed_ms)
{
	race_output_t *output_p = &race_state.output;
	float position_gain = CarTuning_Get(CAR_TUNE_POSITION_GAIN);
	float heading_gain = CarTuning_Get(CAR_TUNE_HEADING_GAIN);
	float limit = CarTuning_Get(CAR_TUNE_STEERING_LIMIT);
	float slew = CarTuning_Get(CAR_TUNE_STEERING_SLEW);
	float step = 0.0f;
	float delta = 0.0f;
	uint32_t steering_ms = elapsed_ms;

	output_p->turn_target = tracking ?
		position_gain * sample_p->dev + heading_gain * sample_p->k :
		output_p->turn_right;
	output_p->turn_target = fmaxf(-limit,
		fminf(limit, output_p->turn_target));

	if (slew == 0.0f)
	{
		output_p->turn_right = output_p->turn_target;
	}
	else
	{
		/* 卡顿后最多补两个控制周期，避免一次下发大幅跳变。 */
		if (steering_ms > 2U * CAR_CONTROL_PERIOD_MS)
		{
			steering_ms = 2U * CAR_CONTROL_PERIOD_MS;
		}

		step = slew * (float)steering_ms / 1000.0f;
		delta = output_p->turn_target - output_p->turn_right;
		output_p->turn_right += fmaxf(-step, fminf(step, delta));
	}

	/* 新限幅立即生效，优先于变化率限制。 */
	output_p->turn_right = fmaxf(-limit,
		fminf(limit, output_p->turn_right));
}

void Race_Update(const vision_sample_t *sample_p, uint32_t now_ms,
	float drive_scale)
{
	race_output_t *output_p = &race_state.output;
	uint32_t elapsed_ms = now_ms - race_state.last_update_ms;
	bool fresh = sample_p->sample_id != race_state.last_sample_id;
	bool usable = Race_Usable(sample_p, now_ms);
	float target_speed = CarTuning_Get(CAR_TUNE_DRIVE_SPEED);
	float turn_threshold = CarTuning_Get(CAR_TUNE_TURN_THRESHOLD);
	float turn_speed = CarTuning_Get(CAR_TUNE_TURN_SPEED);
	float degraded_speed = CarTuning_Get(CAR_TUNE_DEGRADED_SPEED);
	float step = 0.0f;

	race_state.last_update_ms = now_ms;

	if (!output_p->enabled)
	{
		return;
	}

	if (usable && fresh)
	{
		race_state.last_good_ms = sample_p->received_ms;
	}

	if (elapsed_ms >= CAR_VISION_STOP_MS || drive_scale <= 0.0f ||
		!isfinite(drive_scale) ||
		(uint32_t)(now_ms - race_state.last_good_ms) >=
		CAR_VISION_STOP_MS)
	{
		Race_Stop(true);

		return;
	}

	if (fresh)
	{
		race_state.last_sample_id = sample_p->sample_id;

		if (usable)
		{
			Race_UpdateMarker(sample_p, now_ms);
		}
		else
		{
			race_state.clear_frames = 0U;
			race_state.finish_frames = 0U;
		}
	}

	Race_UpdateSteering(sample_p, usable &&
		(uint32_t)(now_ms - race_state.last_good_ms) <
		CAR_VISION_SLOW_MS, elapsed_ms);

	/* 目标提前触发降速；输出尚未回正时继续保持转向档。 */
	if (fmaxf(fabsf(output_p->turn_target),
		fabsf(output_p->turn_right)) >= turn_threshold)
	{
		/* 转向档只能降低直道目标，不能因独立调参而加速。 */
		target_speed = fminf(target_speed, turn_speed);
	}

	target_speed = fminf(MOTOR_MAX_SPEED_MM_S,
		target_speed * drive_scale);

	if (!usable || (uint32_t)(now_ms - race_state.last_good_ms) >=
		CAR_VISION_SLOW_MS)
	{
		/* 上坡补偿不放大视觉降级上限，下坡仍按比例降速。 */
		target_speed = fminf(target_speed,
			degraded_speed * fminf(1.0f, drive_scale));
	}

	if (output_p->phase == RACE_STOPPING)
	{
		target_speed = 0.0f;
	}

	step = (target_speed > output_p->drive_speed_mm_s ? CAR_ACCEL_MM_S2 :
		CAR_DECEL_MM_S2) * (float)elapsed_ms / 1000.0f;

	if (target_speed > output_p->drive_speed_mm_s)
	{
		output_p->drive_speed_mm_s = fminf(target_speed, output_p->drive_speed_mm_s + step);
	}
	else
	{
		output_p->drive_speed_mm_s = fmaxf(target_speed, output_p->drive_speed_mm_s - step);
	}

	if (output_p->phase == RACE_STOPPING && output_p->drive_speed_mm_s == 0.0f)
	{
		output_p->enabled = false;
		output_p->phase = RACE_DONE;
		output_p->turn_right = 0.0f;
		output_p->turn_target = 0.0f;
	}
}

race_output_t Race_GetOutput(void)
{
	return race_state.output;
}
