#ifndef SLOPE_H
#define SLOPE_H

#include <stdbool.h>
#include <stdint.h>

/** 坡道阶段；允许跨阶段变化以适应短坡和实际采样。 */
typedef enum
{
	SLOPE_FLAT,
	SLOPE_UPHILL,
	SLOPE_CREST,
	SLOPE_DOWNHILL
} slope_phase_e;

/** 姿态处理参数，由应用传入，模块不依赖应用配置头。 */
typedef struct
{
	float enter_deg;
	float exit_deg;
	float filter_gain;
	float direction;
	uint32_t confirm_ms;
	uint16_t warmup_samples;
	uint16_t baseline_samples;
} slope_config_t;

/** pitch 诊断副本；ready 之前必须保持车辆静止且不允许起步。 */
typedef struct
{
	float relative_pitch;
	float baseline_pitch;
	slope_phase_e phase;
	bool ready;
} slope_status_t;

/** @brief 初始化处理器。
 * @param config_p 合法的阈值、方向及采样配置。
 * @retval 无。
 */
void Slope_Init(const slope_config_t *config_p);

/** @brief 每次新的 IMU 解算后更新。
 * @param pitch_deg IMU pitch，单位度。
 * @param now_ms 当前毫秒时刻。
 * @retval 无。
 */
void Slope_Update(float pitch_deg, uint32_t now_ms);

/** @brief 获取相对 pitch 和阶段。 @retval 当前处理状态副本。 */
slope_status_t Slope_GetStatus(void);

#endif
