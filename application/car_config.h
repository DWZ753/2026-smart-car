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
#define CAR_VISION_SLOW_MS 80U
#define CAR_VISION_STOP_MS 300U
/* Valid min 现在是转向权限满值门限，不再作为几何可用门限。 */
#define CAR_VALID_MIN 40U
#define CAR_LOST_DECAY_MS 300U
#define CAR_RECOVER_FRAMES 3U
#define CAR_ELEMENT_CONF_MIN 70U
#define CAR_START_GUARD_MS 2000U
#define CAR_MARK_CONFIRM_FRAMES 5U
#define CAR_FINISH_CONFIRM_FRAMES 3U

#define CAR_POSITION_GAIN 70.0f
#define CAR_HEADING_GAIN 130.0f
#define CAR_STEERING_LIMIT 80.0f
/* 转向控制单位/秒，0 关闭变化率限制。 */
#define CAR_STEERING_SLEW_PER_S 700.0f
#define CAR_SERVO_CENTER 0
#define CAR_SERVO_DIRECTION 1

/* 开环速度命令，单位 mm/s，由 motor 模块负责换算。 */
#define CAR_DRIVE_SPEED_MM_S 1200.0f
#define CAR_TURN_SPEED_MM_S 700.0f
#define CAR_DEGRADED_SPEED_MM_S 300.0f
#define CAR_TURN_SLOW_THRESHOLD 55.0f
/* 差速跟随逻辑转向，在 Limit 处达到最大比例。 */
#define CAR_DIFF_THRESHOLD 5.0f
#define CAR_DIFF_MAX_RATIO 0.40f
#define CAR_ACCEL_MM_S2 427.0f
#define CAR_DECEL_MM_S2 2136.0f

/* 相对 pitch 的正值定义为抬头，安装相反时改为 -1。 */
#define CAR_PITCH_DIRECTION (-1.0f)
#define CAR_PITCH_WARMUP_SAMPLES 100U
#define CAR_PITCH_BASELINE_SAMPLES 100U
#define CAR_PITCH_ENTER_DEG 6.0f
#define CAR_PITCH_EXIT_DEG 3.0f
#define CAR_PITCH_CONFIRM_MS 100U
#define CAR_PITCH_FILTER_GAIN 0.2f
/* 分别缩放上坡、坡顶和下坡目标；上坡允许大于 1 以增加驱动。 */
#define CAR_UPHILL_SPEED_SCALE 1.25f
#define CAR_CREST_SPEED_SCALE 0.80f
#define CAR_DOWNHILL_SPEED_SCALE 0.65f

#if (CAR_SERVO_DIRECTION != 1) && (CAR_SERVO_DIRECTION != -1)
#error "CAR_SERVO_DIRECTION must be 1 or -1"
#endif

#endif
