# OLED 最小上板测试

**当前默认关闭**: 最小测试已由用户上板确认正常，现恢复菜单。
需要复测时，将 application/car_config.h 中 CAR_OLED_TEST 改为 1。
该开关优先于 CAR_STAGE 和 CAR_ENABLE_MENU，启动后只执行 OLED 测试，
不调用 Car_Init，不接入菜单、视觉、按键、IMU 或电机/舵机应用。

重新编译并下载本次固件，预期现象：

1. 初始化后强制全屏点亮 2 秒，使用 SSD1306 A5 命令，不依赖字体或画布。
2. 恢复显存显示并清屏，全黑 1 秒。
3. 显示 OLED TEST、I2C0 ADDR 0x3C、COUNT，计数每秒递增。

固定接线仍为 SDA=PA28、SCL=PA31，7 位地址 0x3C。
此测试直接走原有逐字节 OLED 接口，绕开新增增量刷新路径。
菜单等源码仍可参与编译，但本模式不会初始化或调用它们。

若全亮阶段也看不到，先检查供电、共地、SDA/SCL 接线、屏幕接口模式及地址。
若全亮和文字正常，再回到菜单刷新路径排查。
若文字停在固定计数，检查主循环和毫秒 tick。
未连接或异常总线时，原驱动的超时与解锁可能让初始化明显变慢。

测试结束后将 CAR_OLED_TEST 改为 0，恢复原来的分级应用和菜单。
默认测试固件产物在 Debug/firmware_build/default/2026_smart_car.out。
CCS Build 的产物仍为其原有 Debug 路径。
