# 视觉串口通信协议

**适用版本**：当前 `main` 分支，`modules/vision/vision.c` 与 `application/race/race_control.c`。
**物理层**：UART3，115200 8N1，无流控；视觉端 TX → MCU PA25，MCU PA26 为 TX（当前不向视觉端发送）。
**链路方向**：单向下行，MCU 不回复 ACK，也没有重传或流控。
**帧类型**：固定 13 字节原始二进制帧，不使用 COBS、行文本或旧协议。

## 1. 帧格式

| 偏移 | 长度 | 字段 | 类型 | 编码与范围 | 含义 |
|---:|---:|---|---|---|---|
| 0 | 2 | header | uint8[2] | `0x55 0xAA` | 帧头 |
| 2 | 1 | seq | uint8 | 0～255，回绕 | 新测量序号 |
| 3 | 1 | elem | uint8 | 0～6 | 元素类型，见下表 |
| 4 | 1 | phase | uint8 | 0～3 | 环岛阶段，见下表 |
| 5 | 2 | dev | int16 大端 | raw −1000～1000，除以 1000 | 横向偏差，−1.000～+1.000 |
| 7 | 1 | k | int8 | raw −128～127，除以 100 | 航向斜率，约 −1.28～+1.27 px/row |
| 8 | 1 | conf | uint8 | 0～100 | 元素识别置信度 |
| 9 | 1 | valid | uint8 | 0～100 | 有效行比例（几何质量） |
| 10 | 1 | lost | uint8 | 0 或 1 | 0 正常，1 丢线 |
| 11 | 1 | sum | uint8 | `sum(bytes[2..10]) & 0xFF` | 累加校验 |
| 12 | 1 | tail | uint8 | `0x0D` | 帧尾 |

多字节字段统一**大端**（高字节在前）。`dev` 为二进制补码有符号数；
`k` 为有符号 int8，例如 `0xF4` 表示 −12，即 −0.12 px/row。

### 1.1 elem

| 值 | 含义 | 当前控制层用法 |
|---:|---|---|
| 0 | 普通道路 | 正常循迹 |
| 1 | 急弯 | 不单独处理，仍按 dev/k 跟踪 |
| 2 | 十字 | 不单独处理 |
| 3 | 左环岛 | 不单独处理，不选择出口 |
| 4 | 右环岛 | 不单独处理，不选择出口 |
| 5 | 斑马线 | 起点离开/终点停车确认 |
| 6 | 丢线 | 视为几何不可用，等同 `lost=1` |

### 1.2 phase

| 值 | 含义 |
|---:|---|
| 0 | 非环岛 |
| 1 | 进入环岛 |
| 2 | 环内 |
| 3 | 离开环岛 |

当前控制层**不使用 phase**，环岛出口完全依赖 dev/k 的连续跟踪。
如果后续需要按元素选择路径，必须同时修改 `race_control.c` 和比赛状态机。

### 1.3 dev / k 的符号约定

```text
turn_target = clamp(KP × dev + KH × k, −Limit, +Limit)
```

- `dev > 0` 和 `k > 0` 都表示**需要向右修正**，输出 `turn_right > 0`。
- `dev` 的归一化方式由视觉端决定；固件只要求它落在 −1～+1，
  且 0 表示车辆正对道路中心。
- `k` 是道路中心线在图像中的斜率，单位为 px/row；正值同样要求右转。
- `dev` 和 `k` 只在几何可用（`lost=0`、`elem≠6`、帧足够新）时参与转向；
  `valid` 不决定可用性，只缩放转向权限。

## 2. 校验与解析规则

### 2.1 帧同步与校验顺序

1. 在字节流中搜索 `55 AA`，累积 13 字节。
2. 检查第 12 字节是否为 `0x0D`。不是则记 `tail_errors`，窗口前移 1 字节重新搜索。
3. 检查第 11 字节是否等于第 2～10 字节累加和对 256 取模。不是则记 `checksum_errors`，前移 1 字节。
4. 检查字段范围：
   - `elem ≤ 6`、`phase ≤ 3`、`conf ≤ 100`、`valid ≤ 100`、`lost ≤ 1`
   - `dev_raw ∈ [−1000, 1000]`
   超范围记 `range_errors`，前移 1 字节。
5. 通过以上检查后进入序号判定；通过则发布新观测并令 `sample_id++`，同时累加 `accepted`。

逐字节前移可以处理 `55 55 AA`、载荷中出现的 `55 AA`/`0D`、以及分多次到达的帧。

### 2.2 序号规则

| 情况 | 处理 | 计数 |
|---|---|---|
| `seq` 与上一条已接受帧相同 | 丢弃，不刷新观测 | `duplicates` |
| `seq` 回退且距上一条已接受帧 < 100 ms | 判为倒序，丢弃 | `sequence_errors`、`rejected` |
| 间距/跳号（`delta ≠ 1`，非回绕） | 接受，但记一次不连续 | `discontinuities` |
| 距上一条帧 ≥ 100 ms 后出现任意序号 | 接受并重新同步 | — |

`sample_id` 只在接受**新的独立测量**时递增。重复序号不会刷新
`received_ms`，也不会延长视觉有效性；视觉端必须为每个新感知结果递增 `seq`。

### 2.3 帧率与新鲜度

协议本身不规定固定周期，但固件按以下时限判断：

| 常量 | 当前值 | 含义 |
|---|---:|---|
| `CAR_VISION_SLOW_MS` | 80 ms | 没有新帧但未明确丢线：冻结舵量并限制到 Degr speed |
| `CAR_VISION_STOP_MS` | 300 ms | 距最后可用帧达到此年龄：故障停车 |
| `CAR_LOST_DECAY_MS` | 300 ms | 明确 `lost=1`/`elem=6` 时舵量线性衰减到 0 的时间 |
| `CAR_CONTROL_PERIOD_MS` | 10 ms | 控制周期 |

视觉端 V2 之后按**感知帧**发送，实测约 **30～34 Hz**；13 字节在 115200 8N1 下
约 1.13 ms，带宽不是瓶颈。每个新感知结果都要递增 `seq`，重复帧不会刷新观测。
即使丢线，也应继续发送 `lost=1` 的新序号帧，便于诊断页观察，
但 `lost=1` 帧**不会**延长上面的 300 ms 安全时限。

## 3. 固件如何使用各字段

### 3.1 启动条件

`Race_Start()` 要求存在一条**可用且足够新**的观测：

```text
sample_id != 0
lost == 0
elem != 6
(now - received_ms) < 80 ms
```

`valid` **不再参与启动门控**；它只缩放转向权限。
任一条件不满足，Start 申请被拒绝并显示 `Vision not ready`。

### 3.2 运行中

| 字段 | 作用 |
|---|---|
| `dev`、`k` | 只在几何可用且帧足够新时计算转向目标 |
| `valid` | 只缩放转向权限：`valid >= Valid min`（默认 40）为 100%，0 为 50%，中间线性插值；不触发故障 |
| `lost`、`elem=6` | 保持最后跟踪舵量并 300 ms 线性衰减；速度限制到 Degr speed；300 ms 后故障 |
| `conf` | 只用于终点斑马线确认，阈值 `CAR_ELEMENT_CONF_MIN = 70` |
| `elem=5` | 起点离开计数清零；终点需连续 3 个高置信度斑马线帧 |
| `phase` | 当前不参与控制 |

`conf` **不评价循迹质量**：低 `conf`、低 `valid` 但 `lost=0` 时仍会跟踪，
只是转向权限降低；`conf` 只影响终点停车判定。

故障后若连续出现 **3 个新的可用帧**（`sample_id` 递增、`lost=0`、`elem≠6`、
IMU 有效），主控自动回到 `RUN`，不再要求人工重新 Start；如果故障发生在
终点 `STOPPING` 阶段，恢复后仍保持 `STOPPING` 并继续停车到 `DONE`。

### 3.3 丢线时间线

以最后一个**可用**帧为起点：

| 时间 | 行为 |
|---:|---|
| 收到 `lost=1`/`elem=6` 立即 | 保持最后跟踪舵量，按 `CAR_LOST_DECAY_MS` 线性衰减；速度目标限制到 Degr speed |
| 80 ms | 明确丢线时继续按时间衰减；仅停发新帧时冻结舵量 |
| 300 ms | 舵量衰减到 0，进入 `RACE_FAULT`：两轮制动、舵机回中 |
| 恢复后 | 连续 3 个新的可用帧自动回到 `RUN`；终点阶段恢复为 `STOPPING` |

`last_good_ms` 只在 `usable && fresh` 时刷新，因此连续发送 `lost=1`
不会延长安全时间。详细的丢线处理见 [实时调参说明](../../application/tuning/car_tuning.md)。

### 3.4 视觉端 V1–V5 对齐

| 字段/行为 | 视觉端当前实现 | 主控用法 |
|---|---|---|
| `seq` | 感知序号，约 30～34 Hz；只在新感知时发帧 | 重复 seq 视为重复帧，不刷新时效 |
| `valid` | 可用行比例（几何质量） | 只缩放转向权限，不触发故障 |
| `lost` | 真丢线（约 `valid < 12%`）时为 1 | 保持+300 ms 线性衰减 |
| `elem=6` | 与 `lost=1` 等同 | 同上 |
| `conf` | 斑马线恒为 90 | 只用于终点判定（阈值 70） |
| `elem=3/4` | 当前安装几何下基本识别不到 | 不要依赖环岛识别 |
| `elem=2` | 十字全白：`dev` 冻结、`k=0`、`conf=85`、`lost=0` | 建议按直行处理；当前主控仍使用冻结的 `dev` |
| `k` | V5 已扣除横向偏移分量 | 直道横偏时 `k≈0`，KH 需实车重标 |

KH 重标步骤：先在直道横向摆偏（`dev≈±0.8`）确认 `k≈0`；若直道开始来回摆，
可先把 KP 降到 0.7 倍再微调 KH。`elem=2` 的十字策略未包含在本次主控修改中。

## 4. 发送端示例

以下 Python 片段演示组帧；串口发送前请按实际设备名修改 `port`。

```python
import struct
import time
import serial

def make_frame(seq, elem, phase, dev, k, conf, valid, lost):
    dev_raw = max(-1000, min(1000, int(round(dev * 1000))))
    k_raw = max(-128, min(127, int(round(k * 100))))

    body = (
        bytes([seq & 0xFF, elem, phase])
        + struct.pack(">h", dev_raw)   # 偏移 5～6，大端有符号
        + struct.pack("b", k_raw)      # 偏移 7，有符号 int8
        + bytes([conf, valid, lost])   # 偏移 8～10
    )
    checksum = sum(body) & 0xFF
    return b"\x55\xAA" + body + bytes([checksum, 0x0D])

port = serial.Serial("COMx", 115200, timeout=0)
seq = 0

while True:
    frame = make_frame(seq, 0, 0, dev=0.0, k=0.0,
                       conf=90, valid=90, lost=0)
    port.write(frame)
    seq = (seq + 1) & 0xFF
    time.sleep(0.02)   # 50 Hz；应按实际感知周期调整
```

注意：

- 必须用 **HEX/二进制** 发送；串口助手发送文本 `55 AA` 会变成 `35 35 20 41 41`。
- `seq` 每个新测量递增；重复帧不会刷新固件观测时间。
- 不要发送 COBS、XOR 或行协议，当前固件不识别旧格式。

## 5. 合法帧示例

以下帧的校验均已按 `sum(bytes[2..10]) & 0xFF` 核对，并用真实解析器验证过。
它们是独立的测试帧；连续发送时必须递增 `seq`，否则会被判为重复帧。

```text
# 普通道路，seq=1，dev=0，k=0，conf=90，valid=100，lost=0
55 AA 01 00 00 00 00 00 5A 64 00 BF 0D

# 普通道路，seq=1，dev=-0.300，k=-0.12，conf=90，valid=80，lost=0
55 AA 01 00 00 FE D4 F4 5A 50 00 71 0D

# 斑马线，seq=10，phase=0，dev=0，k=0，conf=90，valid=100，lost=0
55 AA 0A 05 00 00 00 00 5A 64 00 CD 0D

# 丢线，seq=11，dev=0，k=0，conf=0，valid=0，lost=1
55 AA 0B 00 00 00 00 00 00 00 01 0C 0D

# 急弯，seq=12，dev=+0.500，k=+0.20，conf=80，valid=90，lost=0
55 AA 0C 01 00 01 F4 14 50 5A 00 C0 0D

# 左环岛入环，seq=13，phase=1，dev=0，k=0，conf=85，valid=88，lost=0
55 AA 0D 03 01 00 00 00 55 58 00 BE 0D
```

## 6. 诊断

OLED 首页 **UART raw** 页可直接查看解析前的最近 24 字节和分层计数，
操作与计数含义见 [视觉 UART 原始接收排查](../../application/vision_rx_debug.md)。

快速定位：

| 现象 | 优先检查 |
|---|---|
| `SYNC` 不增长 | 视觉端是否真的发送 `55 AA`，波特率/接线/共地 |
| `SYNC` 增长但 `OK` 不增长 | 看 `T/C/R`：帧尾、校验和或字段范围 |
| `R` 增长 | `elem/phase/conf/valid/lost/dev` 是否超范围 |
| `DUP` 增长 | 视觉端 `seq` 没有递增 |
| `OLD` 增长 | 序号回退或发送端重启；100 ms 后自动重同步 |
| `DROP/OVR/ERR` 增长 | 主循环阻塞、硬件溢出或信号质量 |

Vision 列表的 `Age ms` 是最后一个**被接受新帧**的年龄，
HEX 页的 `A` 是最后一个**字节**的年龄。两者含义不同，不能互相替代。
