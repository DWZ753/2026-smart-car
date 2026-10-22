# 智能车首版应用与分级联调

**当前状态**: 已编写首版，主机测试及四级固件编译链接通过，尚未上板验证。
**默认级别**: CAR_STAGE_RECEIVE，仅接收，不初始化电机和舵机应用。

## 1. 分级配置

集中配置在 [car_config.h](car_config.h)，参考 VANGUARD 的统一配置头和
VISION_STAGE/SHOOT_STAGE 分级写法；这里用一个 CAR_STAGE 避免单车开关组合过多。
改默认宏后在 CCS 重新编译，也可通过命令行 -DCAR_STAGE=n 选择。

| CAR_STAGE | 功能 | 实车验证 |
|---|---|---|
| 1 / CAR_STAGE_RECEIVE | 仅 UART3 原始接收、解码、诊断缓存 | dev/k 正负与尺度、seq、valid、帧间隔 |
| 2 / CAR_STAGE_STEERING | 加入按键和舵机，电机输出不编入底盘执行路径 | 架空或断开驱动，核对中位、方向、左右极限和响应 |
| 3 / CAR_STAGE_DRIVE | 加入低速开环驱动、起点屏蔽、斑马线停车、失效停机 | 先直道和弯道，再十字、环岛，最后测试停车 |
| 4 / CAR_STAGE_SLOPE | 加入 10 ms IMU 和相对 pitch 坡道判断 | 静止平地建基准后低速测试坡道 |

级别是条件编译，不是上电后自动切换的菜单。
一级默认不启用执行器；二级及以上也不会因为收到视觉帧就自行启动。
现有独立 BSP/设备源文件仍按 CCS 配置编译，未使用函数可由链接器裁剪；
分级负责实际初始化、调用和输出行为，不承诺把每个旧设备文件从编译列表移除。

## 2. 各层职责

```text
main.c
  Board_Init()：唯一 SYSCFG_DL_init 入口和毫秒 tick
  Car_Init()/Car_Update()：注册与裸机调度

modules/vision：纯字节协议、序号和观测缓存，无 UART 或执行器依赖
modules/slope：纯 pitch 基准、滤波、滞回和持续时间判断

application/race：dev/k 跟踪、驱动目标、比赛状态和停止锁定
application/chassis：分配左右轮速度，唯一舵机与 motor 调用入口，按级别裁剪

bsp/uart、modules/servo、modules/motor、modules/imu、modules/key
  继续复用已有设备接口
```

没有引入 RTOS、发布订阅、里程或轮速闭环，不实现路障识别/绕行。
十字和环岛不设电控出口选择逻辑，持续跟踪视觉参考误差。

## 3. 按键与启动条件

- KEY1 / PB18：上一项；KEY2 / PB23：下一项。
- KEY3 / PB19：确认；KEY4 / PB10：返回并发布停止。
- Run status → Start → KEY3，申请启动；Stop 项确认或任意页 KEY4 停止。
- Debug tune 顶部也提供 Start/Stop，同样按 KEY3 申请启停并保持在调参页。
- Debug tune 编辑弹窗内 KEY4 只退出编辑，不发布停止；页面级 KEY4 仍停止。
- 纯导航不产生启停，返回与确认同时出现时停止优先。
- 一级可完整导航，但启动仅显示 Stage 1: RX only，不启用执行器。
- 按键沿用低有效、20 ms 扫描和释放事件。
- CAR_ENABLE_MENU=0 时保留原 KEY1 启动、KEY2 停止的无屏联调方式。

启动要求存在可靠且未超过 50 ms 的新观测，valid 至少 40，未丢线。
四级还要求 IMU 基准就绪；IMU_Init 完成后，平地静止约 2 秒进行预热和均值建基准。
未满足条件时启动申请被拒绝，保持停止；之后需在 Start 项重新确认。
故障恢复后也需要显式按键启动，不由视觉帧解除锁定。

初次验证应先核对舵机现有 4 ms PWM 周期是否适合实物型号。
CAR_SERVO_DIRECTION 默认 +1 只是初始约定，必须在二级确认实际正值为右转。
电机方向沿现有镜像安装约定：A 正向、B 反向；方向不符时在配置头修改。

## 4. 算法与初值

```text
turn_target = limit(KP * dev + KH * k, -Limit, Limit)
turn_right += limit(turn_target - turn_right, -step, step)
step = Steer_slew * dt
```

dev 为 [-1,1]，k 单位 px/row；两个正值都要求右转。
位置增益、方向增益和开环速度命令的上电初值见 car_config.h。
这些量在启动时从 [car_config.h](car_config.h) 复制到 RAM 调参表，
可从 Debug tune 页实时修改，改动在下一个 10 ms 控制周期生效。
参数不写 Flash，掉电或复位后恢复配置头默认值。
不启用积分，方向反馈不再求时间微分，也不套用旧灰度 PID 参数。
大转向时降低速度命令；坡道阶段按上坡/坡顶/下坡分别乘 1.25/0.80/0.65。
上坡比例允许调到 2.00，坡顶/下坡最高 1.00，速度目标不超过 motor 模块上限。
上坡补偿不放大视觉降级速度上限；坡顶和下坡仍按各自比例降低降级目标。
转向档目标取 Drive speed 与 Turn speed 的较小值，调低直道目标不会转弯加速。
短时丢线保持的旧转向也会立即按当前 Limit 重新限幅。
转向变化率默认 800 单位/秒，10 ms 最多变化 8；实时项为 Steer slew。
设为 0 直通目标；转向 dt 最多取两个控制周期，防止卡顿后大幅跳变。
降速判断采用目标与输出的最大绝对值，避免输出尚在跟随时错误恢复直道速度。
保留当前直道/转弯/降级默认值 1200/800/300 mm/s；上坡可靠直行目标为 1500 mm/s。

chassis 根据平滑后的逻辑 turn_right 连续介入差速，不另行解释 dev/k 或元素类型。
默认转向幅度超过 25 开始介入，到当前 Limit=60 时，外轮加 20%、内轮减 20%。
左右轮通过 Motor_SetSpeedLR 下发；外轮超速时两轮共同缩放，保持命令比例。
Diff ratio=0 或 Diff start>=Limit 时关闭辅助；零速及未启用仍直接制动。
Speed mm/s 显示的是差速分配前基础速度，最终两轮命令可能不同。
经验差速没有增加轮速闭环或车辆几何模型；参数详见实时调参说明。

速度命令上升按 427 mm/s²，下降按 2136 mm/s²。
到达终点后按斜坡降到零，再保持短路制动；本地停止和故障直接制动。
驱动量降到零表示命令停止，不是测得实际零速度。

起点离开需至少 5 个独立可靠非斑马线观测，且启动屏蔽时间达到 2 秒。
之后至少 3 个独立高置信度斑马线帧触发停车。
这些是低速初值，不证明完整一圈，也不保证精确停车位置。

50 ms 没有可靠新测量时降驱动，100 ms 时故障停止。
新的丢线帧不延长最后可靠几何的有效期，重复 seq 也不刷新有效期。
短时丢线仅有限保持最后转向，不进行主动搜索或无限盲走。
明确丢线、质量不足或有效观测年龄达到 50 ms 时冻结当前输出，
不再朝旧目标继续推进；100 ms 故障制动，目标及输出转向同时清零。
conf 只评价元素；几何可靠性主要看 valid/lost 和年龄。

坡道相对 pitch 使用平地均值，预热 100 次、基准均值 100 次，周期 10 ms。
进入阈值 6°，退出阈值 3°，候选持续 100 ms；允许上坡直接转下坡。
若安装方向相反，修改 CAR_PITCH_DIRECTION。加减速与振动下的误判需实测。

## 5. 观测与测试

调试器可查看模块内部静态状态，业务代码通过 getter 获取副本：

- Vision_GetStatus()：最新 dev/k、seq、sample_id、接收时刻及各类计数。
- Race_GetOutput()：比赛状态、右转参考、速度命令（mm/s）、输出使能。
- Slope_GetStatus()：基准 pitch、相对 pitch、就绪标志和坡道阶段。
- car_state.late_updates：主循环控制迟到次数，供调试器观察。

回调/ISR 不运行赛道决策或输出控制，不启动 OLED 刷新和串口日志洪泛。
当前没有新增调试串口或遥控控制通道。

```powershell
python tools/run_pvc_tests.py
python tools/run_software_tests.py
python tools/build_firmware.py --all-stages
```

build_firmware.py 使用本机已有 SDK 2.11、SysConfig 1.26.2 和 tiarmclang 4.0.4，
与当前 CCS 配置一致；更换安装目录时调整脚本顶部工具路径。
四级产物在 Debug/firmware_build/stage_1～stage_4/2026_smart_car.out。
直接 CCS Build 仍采用配置头默认级别和 CCS 原有 Debug 输出路径。

主机验证真实 vision、slope、race 和 chassis 源码，执行器使用 mock。
覆盖字节同步与符号、坏帧/范围拒绝、重复/倒序/回绕、停车门控、
初始斑马线屏蔽、新帧确认、零 dt 斜坡、计时回绕、无效 IMU 和坡道滞回。
主机测试不包含真实 UART ISR、按键波形、SPI、舵机响应或电机实测。
四级编译有两个原有 PIT switch 枚举未穷尽警告，没有新增应用警告。

## 6. OLED 菜单

当前默认启用 OLED，磁贴、列表、选择器与弹窗说明见 [菜单说明](menu/car_menu.md)。
主循环优先处理串口和控制，菜单在低优先级位置按脏块刷新。
CAR_ENABLE_MENU=0 可关闭菜单；此时一级不再注册按键。
Debug tune 页可调 Drive/Turn/Degraded speed（mm/s）、KP、KH、转向限幅、
valid 门限、转向降速阈值、上坡/坡顶/下坡比例、转向变化率及差速参数；确认参数行进入编辑，
上下键增减，Restore defaults 恢复默认。
调参表位于 [application/tuning/car_tuning.c](tuning/car_tuning.c)，仅驻留 RAM。
Control 页列出相同顺序的十四项当前值和 Stage，Run status 的 Speed mm/s 则是差速分配前基础速度命令，不是实测速度。
Turn target 显示目标，Turn out 显示变化率限制后的实际转向命令。
参数范围、边界和本次检查结果见 [实时调参说明](tuning/car_tuning.md)。

## 视觉接收诊断

首页 UART raw 可查看解析前最近 24 字节和硬件/协议计数。
见 [视觉串口排查](vision_rx_debug.md)；确认只冻结显示，不暂停接收。
原始 UART 接收已增加 FIFO、有界排空、硬件错误与软件队列丢弃计数。
