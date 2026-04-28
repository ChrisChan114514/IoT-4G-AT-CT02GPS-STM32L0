# STM32L0 CT02 GPS Guard

本工程是基于 STM32L051 的 CT02 4G/GPS 模块运行固件。核心目标是让 MCU 在现场独立维护 CT02 的 AT/MQTT/GPS 状态，并把关键状态同步到 HMI/TJC 串口屏。

## 功能概览

- CT02 AT 固件守护程序：通过 `USART2` 驱动 CT02 模块，维持 AT 可用、MQTT 连接、订阅状态和 GPS 上报流程。
- MQTT 下行控制：订阅 `ct02/{deviceId}/down`，解析 `gps.control` JSON，下行动作包括 `status`、`enable`、`disable`、`set_interval`，代码中也保留 `snapshot` 支持。
- MQTT 上行发布：按协议向 `ct02/{deviceId}/ack`、`ct02/{deviceId}/up`、`ct02/{deviceId}/gps` 发布 `gps.control.ack`、`gps.status`、`gps.report`。
- GPS 数据处理：缓存 CT02 GPS 样本，做坐标修正、有效性判断、冷启动收敛处理，并按设定周期自动上报。
- HMI 屏幕串口转发：通过 `USART1` 向 TJC/Nextion 类屏幕发送 GPS、上报次数、当前周期、MQTT 地址/连接状态等显示命令。
- HMI 本地改周期：屏幕可发 5 字节帧 `05 52 LL HH 26` 修改上报周期，MCU 会复用 CT02 `set_interval` 流程产生一致的 MQTT 状态/ACK。

## 串口与外设分工

| 通道 | 用途 | 引脚/配置 |
| --- | --- | --- |
| `USART2` | CT02 4G/GPS 模块 AT 通道 | `PA2=TX`, `PA15=RX`, 115200, DMA RX/TX, IDLE 中断 |
| `USART1` | HMI/TJC 屏幕通道 | `PA9=TX`, `PA10=RX`, 115200, 中断收发 |

`USART2` 是 modem 专用路径，CT02 守护状态机、AT 命令、MQTT publish/subscribe 都走这一路。`USART1` 只负责屏幕数据刷新和本地控制输入，不复用 CT02 的收发路径。

## HMI 屏幕协议

屏幕刷新采用 ASCII 命令，每条命令独立发送，并以 `FF FF FF` 结束。GPS 刷新会拆成四条命令：

- `x0.val=<latitude microdegrees>`
- `t8.txt="N|S"`
- `x1.val=<longitude microdegrees>`
- `t9.txt="E|W"`

其他显示项包括：

- `n0.val=<intervalSeconds>`：当前 GPS/MQTT 上报周期。
- `t11.txt="120.26.111.75:6011"`：MQTT 平台地址显示。
- `t11.bco=<color>`：MQTT 连接状态颜色。

屏幕下发周期修改帧格式为：

```text
05 52 LL HH 26
```

其中 `LL HH` 是小端 `uint16_t` 秒数，当前固件接受范围为 10 到 65535 秒。

## MQTT 协议

默认设备 ID fallback 为 `ct02-001`。主题由设备 ID 生成：

- 下行：`ct02/{deviceId}/down`
- ACK：`ct02/{deviceId}/ack`
- 状态：`ct02/{deviceId}/up`
- GPS：`ct02/{deviceId}/gps`

下行 payload 以 `note/mqtt_gps_down_up_format.txt` 为协议依据。平台发送 `gps.control`，MCU 返回 JSON 形式的 ACK/状态/GPS 报文。周期上报和 HMI 显示共用同一份修正后的 GPS 缓存，避免屏幕与 MQTT 数据不一致。

## 目录结构

```text
Inc/                         公共头文件和 CubeMX 生成头文件
Src/main.c                   CubeMX 主文件、USART1 HMI、USART2 CT02 接入层
Src/ct02_guard.c             CT02 guard 汇总入口，包含拆分后的业务模块
Src/CT02_Guard/              CT02 AT/MQTT/GPS 守护状态机
note/                        MQTT down/up 协议说明
MDK-ARM/hal_template.uvprojx Keil MDK 工程文件
hal_template.ioc             STM32CubeMX 工程配置
```

## 构建说明

本仓库只提交业务设计、CubeMX 配置和 Keil 工程文件，不提交 `Drivers/`、Keil 编译输出、`.uvguix/.uvoptx`、`.vscode/` 等本地或可再生文件。

拉取后需要用 STM32CubeMX 或本机 STM32L0 HAL/Pack 恢复 `Drivers/`。使用 Keil MDK 打开 `MDK-ARM/hal_template.uvprojx` 编译。修改 `Src/CT02_Guard/*.c` 后建议执行完整 rebuild，避免 Keil 增量构建漏掉 `Src/ct02_guard.c` 的 include 依赖。

## 关键入口

- `CT02_Guard_AppInit()`：初始化 CT02 guard，绑定 `USART2`，设置默认设备 ID、周期和连接参数。
- `CT02_App_Tick()`：运行 CT02 守护 tick、DMA RX 恢复和软件看门狗状态。
- `ct02_guard_tick()`：CT02 AT/MQTT/GPS 状态机主循环。
- `ct02_guard_on_mqtt_up_sample()`：MQTT 上行样本产生后同步刷新 HMI。
- `HMI_Screen_RxTick()`：处理屏幕下发的周期修改帧。
