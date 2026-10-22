#include "chassis_control.h"
#include "car_config.h"

#if (CAR_STAGE >= CAR_STAGE_STEERING)
#include "servo.h"
#endif

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
#include "motor.h"
#include "tuning/car_tuning.h"
#include <math.h>
#endif

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
/** @brief 根据平滑逻辑转向分配后轮速度，饱和时共同缩放。
 * @param output_p 已启用且基础速度为正的比赛输出。
 * @retval 无。
 */
static void Chassis_Drive(const race_output_t *output_p)
{
	float speed = fminf(output_p->drive_speed_mm_s, MOTOR_MAX_SPEED_MM_S);
	float threshold = CarTuning_Get(CAR_TUNE_DIFF_THRESHOLD);
	float limit = CarTuning_Get(CAR_TUNE_STEERING_LIMIT);
	float maximum_ratio = CarTuning_Get(CAR_TUNE_DIFF_MAX_RATIO);
	float ratio = 0.0f;
	float difference = 0.0f;
	float scale = 1.0f;
	float left_speed = 0.0f;
	float right_speed = 0.0f;

	/* 没有有效介入区间时保持同速，也避免 Limit=0 时除零。 */
	if (limit > threshold)
	{
		ratio = maximum_ratio * fmaxf(0.0f, fminf(1.0f,
			(fabsf(output_p->turn_right) - threshold) / (limit - threshold)));
	}

	/* 逻辑右转为正：左侧外轮加速，右侧内轮减速。 */
	difference = speed * copysignf(ratio, output_p->turn_right);
	left_speed = speed + difference;
	right_speed = speed - difference;
	scale = fminf(1.0f, MOTOR_MAX_SPEED_MM_S /
		fmaxf(left_speed, right_speed));

	Motor_SetSpeedLR(left_speed * scale, right_speed * scale);
}
#endif

void Chassis_Init(void)
{
#if (CAR_STAGE >= CAR_STAGE_DRIVE)
	Motor_Init();
#endif

#if (CAR_STAGE >= CAR_STAGE_STEERING)
	Servo_Init();
	Servo_SetValue(CAR_SERVO_CENTER);
#endif
}

void Chassis_Apply(const race_output_t *output_p)
{
#if (CAR_STAGE >= CAR_STAGE_STEERING)
	int32_t steering = CAR_SERVO_CENTER;

	if (output_p->enabled)
	{
		steering += (int32_t)(CAR_SERVO_DIRECTION * output_p->turn_right);
	}

	Servo_SetValue(steering);
#else
	(void)output_p;
#endif

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
	/* 无效或零速度制动；先拒绝 NaN/Inf，避免分配限幅掩盖无效值。 */
	if (!output_p->enabled || output_p->drive_speed_mm_s <= 0.0f ||
		!isfinite(output_p->drive_speed_mm_s))
	{
		Motor_Brake();

		return;
	}

	Chassis_Drive(output_p);
#endif
}
