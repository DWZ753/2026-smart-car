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
	float hold_turn;
	uint8_t clear_frames;
	uint8_t finish_frames;
	uint8_t recover_frames;
	bool finish_armed;
	race_phase_e phase_before_fault;
} race_state_t;

static race_state_t race_state;

/** @brief 判断几何可用于跟踪。
 * @param sample_p 当前观测。
 * @param now_ms 当前时刻。
 * @retval true 有观测、未过期且道路几何可靠。
 * @note valid 不再参与可用性判断，只用于缩放转向权限。
 */
static bool Race_Usable(const vision_sample_t *sample_p, uint32_t now_ms)
{
	return sample_p->sample_id != 0U && !sample_p->lost &&
		sample_p->elem != 6U &&
		(uint32_t)(now_ms - sample_p->received_ms) < CAR_VISION_STOP_MS;
}

/** @brief 最新观测是否明确表示丢线。 @retval true lost 或 elem=6。 */
static bool Race_LostHold(const vision_sample_t *sample_p)
{
	return sample_p->lost || sample_p->elem == 6U;
}

/** @brief valid 缩放转向权限，最低保留一半。
 * @param valid 有效行比例 0～100。
 * @retval 转向权限系数 0.5～1.0。
 */
static float Race_Authority(uint8_t valid)
{
	float threshold = CarTuning_Get(CAR_TUNE_VALID_MIN);

	if (!(threshold > 0.0f) || (float)valid >= threshold)
	{
		return 1.0f;
	}

	return 0.5f + 0.5f * (float)valid / threshold;
}

/** @brief 丢线后舵量线性衰减系数。
 * @param held_ms 距最后一个可用帧的毫秒数。
 * @retval 1.0 开始，300 ms 时降到 0。
 */
static float Race_LostScale(uint32_t held_ms)
{
	if (held_ms >= CAR_LOST_DECAY_MS)
	{
		return 0.0f;
	}

	return 1.0f - (float)held_ms / (float)CAR_LOST_DECAY_MS;
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
	race_state.hold_turn = 0.0f;
	race_state.recover_frames = 0U;
	race_state.clear_frames = 0U;
	race_state.finish_frames = 0U;
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

/** @brief 跟随可靠目标；明确丢线时保持并衰减，仅停发时冻结。
 * @param sample_p 最新观测。
 * @param tracking true 当前几何可靠且未超过 SLOW。
 * @param lost_hold true 最新观测明确丢线或 elem=6。
 * @param held_ms 距最后一个可用帧的毫秒数。
 * @param elapsed_ms 距离上次控制的毫秒数。
 * @retval 无。
 */
static void Race_UpdateSteering(const vision_sample_t *sample_p,
	bool tracking, bool lost_hold, uint32_t held_ms, uint32_t elapsed_ms)
{
	race_output_t *output_p = &race_state.output;
	float position_gain = CarTuning_Get(CAR_TUNE_POSITION_GAIN);
	float heading_gain = CarTuning_Get(CAR_TUNE_HEADING_GAIN);
	float limit = CarTuning_Get(CAR_TUNE_STEERING_LIMIT);
	float slew = CarTuning_Get(CAR_TUNE_STEERING_SLEW);
	float desired = 0.0f;
	float step = 0.0f;
	float delta = 0.0f;
	uint32_t steering_ms = elapsed_ms;

	if (tracking)
	{
		desired = (position_gain * sample_p->dev +
			heading_gain * sample_p->k) * Race_Authority(sample_p->valid);
	}
	else if (lost_hold)
	{
		desired = race_state.hold_turn * Race_LostScale(held_ms);
	}
	else
	{
		/* 没有新帧但未明确丢线：保持原舵量，避免在 SLOW 边界跳变。 */
		desired = output_p->turn_right;
	}

	output_p->turn_target = fmaxf(-limit, fminf(limit, desired));

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

	if (tracking)
	{
		race_state.hold_turn = output_p->turn_right;
	}
}

void Race_Update(const vision_sample_t *sample_p, uint32_t now_ms,
	float drive_scale)
{
	race_output_t *output_p = &race_state.output;
	uint32_t elapsed_ms = now_ms - race_state.last_update_ms;
	uint32_t held_ms = 0U;
	bool fresh = sample_p->sample_id != race_state.last_sample_id;
	bool usable = Race_Usable(sample_p, now_ms);
	bool lost_hold = Race_LostHold(sample_p);
	float target_speed = CarTuning_Get(CAR_TUNE_DRIVE_SPEED);
	float turn_threshold = CarTuning_Get(CAR_TUNE_TURN_THRESHOLD);
	float turn_speed = CarTuning_Get(CAR_TUNE_TURN_SPEED);
	float degraded_speed = CarTuning_Get(CAR_TUNE_DEGRADED_SPEED);
	float step = 0.0f;

	race_state.last_update_ms = now_ms;

	/* 故障后连续 3 个新的可用帧自动恢复，不要求人工重新 Start。 */
	if (output_p->phase == RACE_FAULT)
	{
		if (fresh && usable && drive_scale > 0.0f &&
			isfinite(drive_scale) &&
			(uint32_t)(now_ms - sample_p->received_ms) <
			CAR_VISION_SLOW_MS)
		{
			if (race_state.recover_frames < CAR_RECOVER_FRAMES)
			{
				race_state.recover_frames++;
			}
		}
		else if (fresh)
		{
			race_state.recover_frames = 0U;
		}

		if (race_state.recover_frames >= CAR_RECOVER_FRAMES)
		{
			output_p->enabled = true;
			output_p->phase =
				(race_state.phase_before_fault == RACE_STOPPING) ?
				RACE_STOPPING : RACE_RUN;
			output_p->drive_speed_mm_s = 0.0f;
			output_p->turn_right = 0.0f;
			output_p->turn_target = 0.0f;
			race_state.last_good_ms = sample_p->received_ms;
			race_state.last_sample_id = sample_p->sample_id;
			race_state.last_update_ms = now_ms;
			race_state.hold_turn = 0.0f;
			race_state.recover_frames = 0U;
		}

		return;
	}

	if (!output_p->enabled)
	{
		return;
	}

	if (usable && fresh)
	{
		race_state.last_good_ms = sample_p->received_ms;
	}

	held_ms = now_ms - race_state.last_good_ms;

	if (elapsed_ms >= CAR_VISION_STOP_MS || drive_scale <= 0.0f ||
		!isfinite(drive_scale) || held_ms >= CAR_VISION_STOP_MS)
	{
		race_state.phase_before_fault = output_p->phase;
		Race_Stop(true);
		/* 消费当前帧，避免同一 sample_id 在后续周期反复计入恢复。 */
		race_state.last_sample_id = sample_p->sample_id;

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

	Race_UpdateSteering(sample_p, usable && held_ms < CAR_VISION_SLOW_MS,
		lost_hold, held_ms, elapsed_ms);

	/* 目标提前触发降速；输出尚未回正时继续保持转向档。 */
	if (fmaxf(fabsf(output_p->turn_target),
		fabsf(output_p->turn_right)) >= turn_threshold)
	{
		/* 转向档只能降低直道目标，不能因独立调参而加速。 */
		target_speed = fminf(target_speed, turn_speed);
	}

	target_speed = fminf(MOTOR_MAX_SPEED_MM_S,
		target_speed * drive_scale);

	if (!usable || held_ms >= CAR_VISION_SLOW_MS)
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
