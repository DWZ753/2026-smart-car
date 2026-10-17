# 2026_smart_car

**来源**: ti_cup 的 main e09a1dd 公共驱动基线。
**引脚依据**: 2026h 23d6e82，移除 ZDT UART0 及其 DMA 配置。
**工具链**: 保留本工程 MSPM0-SDK 2.11.0.07、SysConfig 1.26.2、
tiarmclang 4.0.4.LTS、-O2 和 XDS110。

## 已复制的文件

- bsp、modules：main 的公共驱动、PID 和模块文档。
- application/comm：旧通信应用文件，当前排除编译。
- main.c、board.c/h：唯一板级初始化、PIT 毫秒计数与分级应用入口。
- tests、tools：主机软件回归测试及串口辅助工具。
- .gitignore 与 CCS 所需头文件搜索路径、源排除规则。

保留本工程原来的 .project、.ccsproject、targetConfigs 和原有设置。
empty.c 是原始模板，保留文件但排除编译，实际入口为 main.c。
入口已接入分级智能车应用，默认 CAR_STAGE=1，只收 UART3 视觉帧。
联调级别和参数在 [car_config.h](application/car_config.h)；
使用步骤见 [首版应用说明](application/car_app.md)。
二级及以上在 Run status 菜单选择 Start 并确认启动；KEY4 返回兼停止。

## 引脚表

| 功能 | 实例与引脚 |
|---|---|
| Pi UART | UART3，TX=PA26，RX=PA25，115200 8N1 |
| 蓝牙 UART | UART1，TX=PB6，RX=PB7，115200 8N1 |
| ZDT UART0 | 未配置；PA10/PA11 和原 DMA channel 0 已释放 |
| 电机 PWM | TIMA0，C0=PB14，C1=PA22 |
| 电机方向 | M1=PB0/PB21，M2=PB8/PB24 |
| 编码器 | M1 A=PA15/B=PA24，M2 A=PA7/B=PA17 |
| 舵机 PWM | TIMA1 C0=PB2 |
| BMI088 SPI | SPI0，SCLK=PA12，MOSI=PA14，MISO=PA13 |
| BMI088 CS | ACC=PB13，GYRO=PB15 |
| OLED I2C | I2C0，SDA=PA28，SCL=PA31，400 kHz，地址 0x3c |
| 按键 | PB18/PB23/PB19/PB10 |
| 灰度 1～8 | PB12/PB4/PB1/PB17/PB9/PB16/PB20/PB11 |
| 蜂鸣器 / LED | PB5 / PB22 |
| 快速 / 控制 tick | TIMG12 1 ms / TIMG0 20 ms |
| SWD | PA20 SWCLK / PA19 SWDIO |

引脚和计时配置采用 2026h；舵机配置仍为 5 MHz、20000 计数，
即 4 ms 周期，使用前需确认舵机规格。
本次没有因复制引脚而擅自修改 PWM 周期或比赛控制参数。

## 未配置模块与最小适配

tests、ZDT、Ultrasonic、TM1637、旧 application/comm 不参加固件编译。
这些文件仍保留，以供以后重新配置对应外设时使用。
Motor 的 GPIO IRQ 宏适配为 GPIO_MOTORs_INT_IRQN，
并移除当前不存在的超声波 ISR 分发。
UART 头文件移除无对应外设的旧 PRINT 映射；使用时通过 UART_Config
选择 UART_PI_INST 或 UART_BT_INST，并显式注册句柄。
当前两路 UART 未配置 TX DMA，不能直接使用 DMA 发送接口。

PID 文件保持 main 原样。问题说明见
[PID 参数与使用问题](pid_parameter_review.md)。

## 验证与构建

在 CCS 中打开本工程，执行 Build，输出 Debug/2026_smart_car.out。
SysConfig 生成文件写入 Debug，修改引脚应改 empty.syscfg。

主机软件回归测试：

```powershell
python tools\run_software_tests.py
```

迁移包已用 SDK 2.11 编译链接 24 个源文件，并通过主机软件回归。
两个 PIT switch 的已有编译警告保留。
尚未验证实物接线、外设波形和控制效果。
目标被覆盖文件的原件保存在 Debug 下的 migration_backup 目录。

## 分析与当前开发方案

[分析资料索引](analysis_index.md)汇总当前 PVC 方案、PID 问题及历史分析证据。
当前按低速开环、视觉 dev/k 跟踪和 IMU pitch 坡道判断推进。

## OLED 菜单

默认 CAR_ENABLE_MENU=1，一至四级均可查看菜单；
KEY1/KEY2/KEY3/KEY4 分别为上、下、确认、返回；返回同时发布停止。
首页为磁贴，二级为可滚动列表；确认只读数据可查看弹窗，控制配置只读。
动画预览：Debug/oled_tests/menu_animation.html。
移植范围、刷新策略与验证步骤见 [菜单说明](application/menu/car_menu.md)。
