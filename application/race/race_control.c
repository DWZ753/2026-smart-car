#include "race_control.h"
#include "car_config.h"
#include "tuning/car_tuning.h"
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
	race_state.output.drive_pwm = 0.0f;
	race_state.output.turn_right = 0.0f;
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

void Race_Update(const vision_sample_t *sample_p, uint32_t now_ms,
	float drive_scale)
{
	race_output_t *output_p = &race_state.output;
	uint32_t elapsed_ms = now_ms - race_state.last_update_ms;
	bool fresh = sample_p->sample_id != race_state.last_sample_id;
	bool usable = Race_Usable(sample_p, now_ms);
	float target_pwm = CarTuning_Get(CAR_TUNE_DRIVE_PWM);
	float position_gain = CarTuning_Get(CAR_TUNE_POSITION_GAIN);
	float heading_gain = CarTuning_Get(CAR_TUNE_HEADING_GAIN);
	float steering_limit = CarTuning_Get(CAR_TUNE_STEERING_LIMIT);
	float turn_threshold = CarTuning_Get(CAR_TUNE_TURN_THRESHOLD);
	float turn_pwm = CarTuning_Get(CAR_TUNE_TURN_PWM);
	float degraded_pwm = CarTuning_Get(CAR_TUNE_DEGRADED_PWM);
	float step = 0.0f;
	float turn = 0.0f;

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

	if (usable)
	{
		turn = position_gain * sample_p->dev + heading_gain * sample_p->k;
		output_p->turn_right = fmaxf(-steering_limit,
			fminf(steering_limit, turn));
	}

	if (fabsf(output_p->turn_right) >= turn_threshold)
	{
		target_pwm = turn_pwm;
	}

	if (!usable || (uint32_t)(now_ms - race_state.last_good_ms) >=
		CAR_VISION_SLOW_MS)
	{
		target_pwm = fminf(target_pwm, degraded_pwm);
	}

	target_pwm *= fminf(1.0f, drive_scale);

	if (output_p->phase == RACE_STOPPING)
	{
		target_pwm = 0.0f;
	}

	step = (target_pwm > output_p->drive_pwm ? CAR_ACCEL_PWM_PER_S :
		CAR_DECEL_PWM_PER_S) * (float)elapsed_ms / 1000.0f;

	if (target_pwm > output_p->drive_pwm)
	{
		output_p->drive_pwm = fminf(target_pwm, output_p->drive_pwm + step);
	}
	else
	{
		output_p->drive_pwm = fmaxf(target_pwm, output_p->drive_pwm - step);
	}

	if (output_p->phase == RACE_STOPPING && output_p->drive_pwm == 0.0f)
	{
		output_p->enabled = false;
		output_p->phase = RACE_DONE;
	}
}

race_output_t Race_GetOutput(void)
{
	return race_state.output;
}
