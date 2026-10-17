#ifndef CAR_APP_H
#define CAR_APP_H

/** @brief 注册各层模块，默认不允许运动。 @retval 无。 */
void Car_Init(void);

/** @brief 主循环调用，优先收帧再处理到期任务。 @retval 无。 */
void Car_Update(void);

#endif
