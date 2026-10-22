#ifndef RACE_CONTROL_H
#define RACE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "vision.h"

/** 无里程的比赛状态；停止态只能通过本地显式启动退出。 */
typedef enum
{
	RACE_READY,
	RACE_DEPART,
	RACE_RUN,
	RACE_STOPPING,
	RACE_DONE,
	RACE_FAULT
} race_phase_e;

/** 单一底盘参考，drive_speed_mm_s 是 mm/s 速度命令，不是反馈。 */
typedef struct
{
	race_phase_e phase;
	float turn_right;
	float drive_speed_mm_s;
	bool enabled;
	/** 限幅后的目标转向；turn_right 为变化率限制后的输出命令。 */
	float turn_target;
} race_output_t;

/** @brief 初始化停止状态。 @retval 无。 */
void Race_Init(void);

/** @brief 本地显式启动。
 * @param sample_p 最新视觉观测。
 * @param now_ms 当前毫秒时刻。
 * @retval true 新鲜且可靠，已进入起步状态。
 */
bool Race_Start(const vision_sample_t *sample_p, uint32_t now_ms);

/** @brief 停止并锁定输出。
 * @param fault true 进入故障停止，false 返回待启动。
 * @retval 无。
 */
void Race_Stop(bool fault);

/** @brief 固定周期更新比赛和转向参考。
 * @param sample_p 最新观测，未到新帧时保持原副本。
 * @param now_ms 当前毫秒时刻。
 * @param drive_scale 坡道驱动比例，0 表示 IMU 尚不可用。
 * @retval 无。
 */
void Race_Update(const vision_sample_t *sample_p, uint32_t now_ms,
	float drive_scale);

/** @brief 获取参考副本。 @retval 当前底盘参考。 */
race_output_t Race_GetOutput(void);

#endif
