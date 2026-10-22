# OLED 菜单与动画联调

**参考版本**: AstraThreshold/oled-ui-astra，当前 main 提交 88716a6。
**实现状态**: 显示/输入/控制/驱动测试及四级编译通过，313 帧预览已核对。
实机按键、显示时序与流畅度待下载检查。

## 1. 参考内容

- [原版仓库](https://github.com/AstraThreshold/oled-ui-astra)
- [Wiki 介绍与动画](https://github.com/AstraThreshold/oled-ui-astra/wiki/astra-UI介绍-‐-astra-UI-Introduction)
- 源码重点：item.h 的逐步趋近、Camera 的滚动边界、Selector 的位置/宽高，
  Tile/List 的展开、Launcher 的进入/返回和弹窗。

原版菜单、视角和选择器分别运动；列表高亮是反白填充，单行高 16 像素。
本工程用 C 实现这些运动关系，沿用现有字库和自绘图标。
当前没有引入 C++/u8g2，也没有复制原版字体、Logo 或源文件。

## 2. 四键操作

| 按键 | 功能 |
|---|---|
| KEY1 / PB18 | 上一项；磁贴页向左 |
| KEY2 / PB23 | 下一项；磁贴页向右 |
| KEY3 / PB19 | 进入；在运行页或调参页确认 Start/Stop；查看只读数据弹窗 |
| KEY4 / PB10 | 关闭弹窗或返回上级，同时发布停止命令 |

任意页返回都可停止。返回与确认同时出现时停止优先，上下同时出现时不移动。
唯一的例外是 Debug tune 的参数编辑弹窗：此时 KEY4 只退出编辑，不发布停止，
避免行驶中调参误停车；退出编辑后再按 KEY4 才返回首页并停止。
列表第一项是页面名称，确认它也可回到首页。
操作沿用按下后释放事件。默认一级可导航，Start 会提示仅接收，不启动执行器。
二级以上 Start 仍由应用检查视觉新鲜度、质量，以及四级的 IMU 就绪条件。
失败时显示原因；确认可关闭提示。Control 页只读，Debug tune 页可实时编辑。

## 3. 页面与动画

首页六项为 Run status、Vision、IMU pitch、Control、Debug tune、UART raw。
二级列表保留各自选中行和视角，超过四行后滚动显示，侧边条表示当前位置。

- 运行列表：Start、Stop、State、Speed mm/s、Turn out、Turn target、Link。
- 视觉列表：Dev、K、Valid、Conf、Lost、Seq、Age ms、Rx、Bad。
- 坡道列表：相对 Pitch、平地 Base、Phase、Ready。
- 控制列表：十四项实时参数，顺序与 Debug tune 相同，最后一项为 Stage。
- 调参列表：Start、Stop、Drive speed、Turn speed、Degr speed、KP、KH、Limit、
  Valid min、Turn thresh、Up scale、Crest scale、Down scale、Steer slew、
  Diff start、Diff ratio、Restore defaults。

调参页在参数行按确认打开编辑弹窗，上下键按固定步长增减，确认或返回退出编辑。
顶部 Start/Stop 按确认直接发布启停请求，页面和参数保持不变，调完无需切回运行页。
Start 复用原应用的视觉、阶段与 IMU 就绪检查，失败时仍显示原有提示。
修改直接写入 `application/tuning/car_tuning.c` 的 RAM 参数表，下一个 10 ms
控制周期即生效，可在行驶中调整；掉电或复位后恢复 `car_config.h` 默认值。
Restore defaults 一次恢复全部默认值。步长与上下限见该文件的规格表。
Control 与 Debug tune 共用数值读取和格式化，均反映当前 RAM 参数。
Up scale 默认 1.25，范围 0.10～2.00；Crest scale 默认 0.80，
Down scale 默认 0.65，两者范围 0.10～1.00；三项步长均为 0.05。
Diff start 默认 25，范围 0～100、步长 5；Diff ratio 默认 0.20，范围 0～0.50、步长 0.05。
差速在 Diff start 到当前 Limit 间连续介入；比例为 0 或阈值不低于 Limit 时关闭。
Drive speed 是直道基础目标；Run status 的 Speed mm/s 是降速、坡道及斜坡处理后的基础速度命令，
随后 chassis 再分配左右轮速度，外轮饱和时共同缩放；该值不是实测速度或单轮速度。
Turn target 是限幅后的目标，Turn out 是按变化率跟随后的舵机命令。
Steer slew 默认 800 单位/秒，范围 0～3000，步长 100；0 关闭变化率限制。
转弯档依据目标和输出中较大的绝对值触发，覆盖提前入弯和逐步回正。
操作后不要复位来检查同步，复位会恢复编译时默认值。

镜头、选择框 x/y/宽/高、展开量、标题和侧边条各有时间常数。
选择框跨页面保持当前形状，目标在磁贴角标与列表反白框之间变化。
列表入场从屏外展开，保留至少 11 像素行距，避免中途文字互相覆盖。
只读弹窗滑入/滑出，并对背景进行棋盘状压暗。
动画每 16 ms 推进，已提交画布在发送完成前保持不变；静止列表每 100 ms 更新数据。

## 4. 硬件与刷新

SSD1306 128x64，I2C0，SDA=PA28、SCL=PA31，400 kHz，7 位地址 0x3C。
CAR_OLED_TEST=0、CAR_ENABLE_MENU=1；最小逐字节测试入口仍可单独启用。

运行期每主循环最多发送一个 7 字节数据块，保留短写的有限等待和故障隔离。
同页连续数据利用屏幕自动增列；跳过无变化区域、换页或写失败后重新定位。
这减少重复定位事务，不改变硬件 FIFO 大小或引脚，也不阻塞整屏发送。
连续五次失败停用显示；初始化仍使用既有逐字节路径。

## 5. 验证与预览

```powershell
python tools/run_oled_tests.py
python tools/run_car_app_tests.py
python tools/run_software_tests.py
python tools/run_pvc_tests.py
python tools/build_firmware.py --all-stages
python tools/build_firmware.py
```

OLED 测试按每笔 I2C 的地址、数据和 ACK 位估算 400 kHz 线时，
并加入每次主循环 40 us 的模拟开销，记录完成传输的实际画布帧。
全屏变化的模型线时小于 35 ms；这不是实机帧率或最坏执行时间测量。
应用测试使用真实 car_app/race/vision/slope，确认导航不误启停、
启动门控、返回停止、一级禁用执行器及关闭菜单时的原按键行为。
菜单测试额外覆盖调参步进、编辑态返回不停止和 Restore defaults；
PVC 测试覆盖默认值与实时生效。

打开 Debug/oled_tests/menu_animation.html 可选择并查看 GIF 动画：
磁贴前后切换、进入列表、选择、滚动、弹窗、返回与循环。
原始二进制画布、PNG 与逐帧时间记录均保存在同一 Debug 目录。

默认下载产物为 Debug/firmware_build/default/2026_smart_car.out。
CCS 自行 Build 的产物仍在 CCS 对应 Debug 路径。
前置提交 eca2b97 保留，当前 OLED 改动尚未提交或推送。

## 6. UART raw 诊断页

首页新增第五项 UART raw，提供 HEX/INFO 两个固定布局的视图。
上下切换视图，确认冻结/恢复最近 24 字节；后台接收和解析保持运行。
逐项说明及合法测试帧见 [视觉串口排查](../vision_rx_debug.md)。
HEX 页的 A 为字节年龄，Vision 列表的 Age 为独立合法观测年龄。
