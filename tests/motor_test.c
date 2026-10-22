#include "motor.h"
#include "tb6612.h"
#include "pit_control_tick.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** 记录真实 motor 模块下发的驱动命令和初始化过程。 */
typedef struct
{
	int32_t left_pwm;
	int32_t right_pwm;
	uint32_t initialized;
	uint32_t stopped;
	uint32_t brakes;
	PIT_Control_Callback_t callback;
} motor_mock_t;

static motor_mock_t motor_mock;

void TB6612_Init(void)
{
	assert(motor_mock.stopped == 2U);
	motor_mock.initialized++;
}

void TB6612_A_Stop(void)
{
	motor_mock.stopped++;
}

void TB6612_B_Stop(void)
{
	motor_mock.stopped++;
}

void TB6612_A_Brake(void)
{
	motor_mock.brakes++;
}

void TB6612_B_Brake(void)
{
	motor_mock.brakes++;
}

void TB6612_A_Forward(uint32_t duty)
{
	motor_mock.left_pwm = (int32_t)duty;
}

void TB6612_A_Backward(uint32_t duty)
{
	motor_mock.left_pwm = -(int32_t)duty;
}

void TB6612_B_Forward(uint32_t duty)
{
	motor_mock.right_pwm = (int32_t)duty;
}

void TB6612_B_Backward(uint32_t duty)
{
	motor_mock.right_pwm = -(int32_t)duty;
}

bool PIT_Control_Tick_RegisterCallback(PIT_Control_Callback_t callback)
{
	motor_mock.callback = callback;

	return true;
}

int main(void)
{
	int32_t expected = 0;
	uint32_t brakes = 0U;

	Motor_Init();
	assert(motor_mock.initialized == 1U);
	assert(motor_mock.callback != NULL);
	Motor_SetSpeed(640.0f);
	expected = (int32_t)(640.0f / MOTOR_MAX_SPEED_MM_S *
		(TB6612_PWM_PERIOD_COUNT - 1U));
	assert(motor_mock.left_pwm == expected * MOTOR_LEFT_DIRECTION);
	assert(motor_mock.right_pwm == expected * MOTOR_RIGHT_DIRECTION);
	Motor_SetSpeed(-640.0f);
	assert(motor_mock.left_pwm == -expected * MOTOR_LEFT_DIRECTION);
	assert(motor_mock.right_pwm == -expected * MOTOR_RIGHT_DIRECTION);

	/* 超范围速度被模块钳到满量程，不再受 40% 应用限制。 */
	Motor_SetSpeedLR(MOTOR_MAX_SPEED_MM_S * 2.0f,
		-MOTOR_MAX_SPEED_MM_S * 2.0f);
	assert(motor_mock.left_pwm ==
		9999 * MOTOR_LEFT_DIRECTION);
	assert(motor_mock.right_pwm ==
		-9999 * MOTOR_RIGHT_DIRECTION);
	brakes = motor_mock.brakes;
	Motor_SetSpeed(0.0f);
	assert(motor_mock.brakes == brakes + 2U);
	Motor_SetSpeedLR(NAN, 640.0f);
	assert(motor_mock.brakes == brakes + 4U);
	Motor_SetSpeed(INFINITY);
	assert(motor_mock.brakes == brakes + 6U);
	puts("PASS: motor init, speed conversion, polarity, limits and brake");

	return 0;
}
