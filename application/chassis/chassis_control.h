#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "race_control.h"

/** @brief 初始化当前编译级别允许的执行器并保持停止。 @retval 无。 */
void Chassis_Init(void);

/** @brief 唯一执行器输出入口，按平滑逻辑转向分配左右轮速度。
 * @param output_p 比赛层提供的转向和开环驱动参考。
 * @retval 无。
 */
void Chassis_Apply(const race_output_t *output_p);

#endif
