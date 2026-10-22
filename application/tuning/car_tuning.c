#include "car_tuning.h"
#include "car_config.h"
#include "motor.h"
#include <stddef.h>
#include <stdint.h>

/** 调参范围与步长；默认值保持与编译期参数一致。 */
typedef struct
{
	float default_value;
	float minimum;
	float maximum;
	float step;
} car_tune_spec_t;

static const car_tune_spec_t tuning_specs[CAR_TUNE_COUNT] =
{
	{CAR_DRIVE_SPEED_MM_S, 0.0f, MOTOR_MAX_SPEED_MM_S, 10.0f},
	{CAR_TURN_SPEED_MM_S, 0.0f, MOTOR_MAX_SPEED_MM_S, 10.0f},
	{CAR_DEGRADED_SPEED_MM_S, 0.0f, MOTOR_MAX_SPEED_MM_S, 5.0f},
	{CAR_POSITION_GAIN, 0.0f, 500.0f, 5.0f},
	{CAR_HEADING_GAIN, 0.0f, 500.0f, 5.0f},
	{CAR_STEERING_LIMIT, 0.0f, 100.0f, 5.0f},
	{(float)CAR_VALID_MIN, 0.0f, 100.0f, 5.0f},
	{CAR_TURN_SLOW_THRESHOLD, 0.0f, 100.0f, 5.0f},
	{CAR_UPHILL_SPEED_SCALE, 0.10f, 2.0f, 0.05f},
	{CAR_CREST_SPEED_SCALE, 0.10f, 1.0f, 0.05f},
	{CAR_DOWNHILL_SPEED_SCALE, 0.10f, 1.0f, 0.05f},
	{CAR_STEERING_SLEW_PER_S, 0.0f, 3000.0f, 100.0f},
	{CAR_DIFF_THRESHOLD, 0.0f, 100.0f, 5.0f},
	{CAR_DIFF_MAX_RATIO, 0.0f, 0.50f, 0.05f}
};

/** 仅保存在 RAM，上电即为默认值。 */
static float tuning_values[CAR_TUNE_COUNT] =
{
	CAR_DRIVE_SPEED_MM_S,
	CAR_TURN_SPEED_MM_S,
	CAR_DEGRADED_SPEED_MM_S,
	CAR_POSITION_GAIN,
	CAR_HEADING_GAIN,
	CAR_STEERING_LIMIT,
	(float)CAR_VALID_MIN,
	CAR_TURN_SLOW_THRESHOLD,
	CAR_UPHILL_SPEED_SCALE,
	CAR_CREST_SPEED_SCALE,
	CAR_DOWNHILL_SPEED_SCALE,
	CAR_STEERING_SLEW_PER_S,
	CAR_DIFF_THRESHOLD,
	CAR_DIFF_MAX_RATIO
};

float CarTuning_Get(car_tune_id_e id)
{
	if ((unsigned)id >= (unsigned)CAR_TUNE_COUNT)
	{
		return 0.0f;
	}

	return tuning_values[id];
}

void CarTuning_Step(car_tune_id_e id, bool increase)
{
	const car_tune_spec_t *spec_p = NULL;
	float value = 0.0f;

	if ((unsigned)id >= (unsigned)CAR_TUNE_COUNT)
	{
		return;
	}

	spec_p = &tuning_specs[id];
	value = tuning_values[id] + (increase ? spec_p->step : -spec_p->step);

	if (value < spec_p->minimum)
	{
		value = spec_p->minimum;
	}
	else if (value > spec_p->maximum)
	{
		value = spec_p->maximum;
	}

	tuning_values[id] = value;
}

void CarTuning_Reset(void)
{
	uint8_t index = 0U;

	for (index = 0U; index < (uint8_t)CAR_TUNE_COUNT; index++)
	{
		tuning_values[index] = tuning_specs[index].default_value;
	}
}
