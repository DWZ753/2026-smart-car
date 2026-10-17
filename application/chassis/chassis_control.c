#include "chassis_control.h"
#include "car_config.h"

#if (CAR_STAGE >= CAR_STAGE_STEERING)
#include "servo.h"
#endif

#if (CAR_STAGE >= CAR_STAGE_DRIVE)
#include "tb6612.h"
#endif

void Chassis_Init(void)
{
#if (CAR_STAGE >= CAR_STAGE_DRIVE)
	TB6612_A_Stop();
	TB6612_B_Stop();
	TB6612_Init();
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
	/* PWM 为零时制动；停止态后续帧不能绕过此门控。 */
	if (!output_p->enabled || output_p->drive_pwm <= 0.0f)
	{
		TB6612_A_Brake();
		TB6612_B_Brake();

		return;
	}

	/* 两电机镜像安装，采用现有 Motor 模块的正向约定。 */
#if (CAR_LEFT_DIRECTION == 1)
	TB6612_A_Forward((uint32_t)output_p->drive_pwm);
#else
	TB6612_A_Backward((uint32_t)output_p->drive_pwm);
#endif

#if (CAR_RIGHT_DIRECTION == 1)
	TB6612_B_Forward((uint32_t)output_p->drive_pwm);
#else
	TB6612_B_Backward((uint32_t)output_p->drive_pwm);
#endif
#endif
}
