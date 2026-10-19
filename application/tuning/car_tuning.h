#ifndef CAR_TUNING_H
#define CAR_TUNING_H

#include <stdbool.h>

/** 运行时可调参数，枚举顺序即 Debug tune 页的行顺序。 */
typedef enum
{
	CAR_TUNE_DRIVE_PWM = 0,
	CAR_TUNE_TURN_PWM,
	CAR_TUNE_DEGRADED_PWM,
	CAR_TUNE_POSITION_GAIN,
	CAR_TUNE_HEADING_GAIN,
	CAR_TUNE_STEERING_LIMIT,
	CAR_TUNE_VALID_MIN,
	CAR_TUNE_TURN_THRESHOLD,
	CAR_TUNE_SLOPE_SCALE,
	CAR_TUNE_COUNT
} car_tune_id_e;

/** @brief 读取当前参数值。 @param id 参数编号。 @retval 参数值，越界返回 0。 */
float CarTuning_Get(car_tune_id_e id);

/** @brief 按内置步长增减参数并钳位。
 * @param id 参数编号。
 * @param increase true 增加，false 减少。
 * @retval 无。
 */
void CarTuning_Step(car_tune_id_e id, bool increase);

/** @brief 恢复 car_config.h 中的默认值。 @retval 无。 */
void CarTuning_Reset(void);

#endif
