#include "slope.h"
#include <math.h>
#include <string.h>

/** 所有基准积累、滤波和滞回计时均由主循环拥有。 */
typedef struct
{
	slope_config_t config;
	slope_status_t status;
	slope_phase_e candidate;
	uint32_t candidate_ms;
	uint16_t samples;
	float sum;
} slope_state_t;

static slope_state_t slope_state;

void Slope_Init(const slope_config_t *config_p)
{
	memset(&slope_state, 0, sizeof(slope_state));
	slope_state.config = *config_p;
}

void Slope_Update(float pitch_deg, uint32_t now_ms)
{
	slope_state_t *state_p = &slope_state;
	slope_phase_e candidate = state_p->status.phase;
	float relative = 0.0f;
	uint32_t target_samples = state_p->config.warmup_samples +
		state_p->config.baseline_samples;

	if (!isfinite(pitch_deg))
	{
		state_p->status.ready = false;
		state_p->samples = 0U;
		state_p->sum = 0.0f;

		return;
	}

	if (!state_p->status.ready)
	{
		state_p->samples++;

		if (state_p->samples > state_p->config.warmup_samples)
		{
			state_p->sum += pitch_deg;
		}

		if (state_p->samples >= target_samples)
		{
			state_p->status.baseline_pitch = state_p->sum /
				(float)state_p->config.baseline_samples;
			state_p->status.relative_pitch = 0.0f;
			state_p->status.phase = SLOPE_FLAT;
			state_p->candidate = SLOPE_FLAT;
			state_p->status.ready = true;
		}

		return;
	}

	relative = (pitch_deg - state_p->status.baseline_pitch) *
		state_p->config.direction;
	state_p->status.relative_pitch += state_p->config.filter_gain *
		(relative - state_p->status.relative_pitch);
	relative = state_p->status.relative_pitch;

	if (relative >= state_p->config.enter_deg)
	{
		candidate = SLOPE_UPHILL;
	}
	else if (relative <= -state_p->config.enter_deg)
	{
		candidate = SLOPE_DOWNHILL;
	}
	else if (fabsf(relative) <= state_p->config.exit_deg)
	{
		candidate = state_p->status.phase == SLOPE_UPHILL ?
			SLOPE_CREST : SLOPE_FLAT;
	}

	if (candidate != state_p->candidate)
	{
		state_p->candidate = candidate;
		state_p->candidate_ms = now_ms;
	}

	if ((uint32_t)(now_ms - state_p->candidate_ms) >=
		state_p->config.confirm_ms)
	{
		state_p->status.phase = candidate;
	}
}

slope_status_t Slope_GetStatus(void)
{
	return slope_state.status;
}
