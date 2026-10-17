#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* 最小屏幕测试优先于 CAR_STAGE；排查完成后设为 0 恢复应用。 */
#ifndef CAR_OLED_TEST
#define CAR_OLED_TEST 0U
#endif

#if (CAR_OLED_TEST != 0U) && (CAR_OLED_TEST != 1U)
#error "CAR_OLED_TEST must be 0 or 1"
#endif

/* 1 收帧；2 舵机；3 低速行驶及停车；4 再加入 IMU 坡道。 */
/* OLED 菜单可独立于运动级别关闭，默认一级也能查看视觉数据。 */
#ifndef CAR_ENABLE_MENU
#define CAR_ENABLE_MENU 1U
#endif

#if (CAR_ENABLE_MENU != 0U) && (CAR_ENABLE_MENU != 1U)
#error "CAR_ENABLE_MENU must be 0 or 1"
#endif

#define CAR_STAGE_RECEIVE 1U
#define CAR_STAGE_STEERING 2U
#define CAR_STAGE_DRIVE 3U
#define CAR_STAGE_SLOPE 4U

#ifndef CAR_STAGE
#define CAR_STAGE CAR_STAGE_SLOPE
#endif

#if (CAR_STAGE < CAR_STAGE_RECEIVE) || (CAR_STAGE > CAR_STAGE_SLOPE)
#error "CAR_STAGE must be 1..4"
#endif

/* 以下是待实车标定的低速起始值，不是已经验证的比赛参数。 */
#define CAR_CONTROL_PERIOD_MS 10U
#define CAR_VISION_SLOW_MS 50U
#define CAR_VISION_STOP_MS 100U
#define CAR_VALID_MIN 40U
#define CAR_ELEMENT_CONF_MIN 70U
#define CAR_START_GUARD_MS 2000U
#define CAR_MARK_CONFIRM_FRAMES 5U
#define CAR_FINISH_CONFIRM_FRAMES 3U

#define CAR_POSITION_GAIN 50.0f
#define CAR_HEADING_GAIN 100.0f
#define CAR_STEERING_LIMIT 60.0f
#define CAR_SERVO_CENTER 0
#define CAR_SERVO_DIRECTION (-1)

/* PWM 计数，不是实际 mm/s；TB6612 周期为 10000。 */
#define CAR_DRIVE_PWM 1000.0f
#define CAR_TURN_PWM 700.0f
#define CAR_DEGRADED_PWM 300.0f
#define CAR_TURN_SLOW_THRESHOLD 30.0f
#define CAR_ACCEL_PWM_PER_S 2000.0f
#define CAR_DECEL_PWM_PER_S 10000.0f
#define CAR_LEFT_DIRECTION 1
#define CAR_RIGHT_DIRECTION (-1)

/* 相对 pitch 的正值定义为抬头，安装相反时改为 -1。 */
#define CAR_PITCH_DIRECTION (-1.0f)
#define CAR_PITCH_WARMUP_SAMPLES 100U
#define CAR_PITCH_BASELINE_SAMPLES 100U
#define CAR_PITCH_ENTER_DEG 6.0f
#define CAR_PITCH_EXIT_DEG 3.0f
#define CAR_PITCH_CONFIRM_MS 100U
#define CAR_PITCH_FILTER_GAIN 0.2f
#define CAR_SLOPE_DRIVE_SCALE 0.65f

#if (CAR_SERVO_DIRECTION != 1) && (CAR_SERVO_DIRECTION != -1)
#error "CAR_SERVO_DIRECTION must be 1 or -1"
#endif

#if ((CAR_LEFT_DIRECTION != 1) && (CAR_LEFT_DIRECTION != -1)) || \
	((CAR_RIGHT_DIRECTION != 1) && (CAR_RIGHT_DIRECTION != -1))
#error "Motor directions must be 1 or -1"
#endif

#endif
