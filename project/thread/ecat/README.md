# HPM5E31 EtherCAT 通信

复用 `E:/Zephyr/zephyr_user` 的 EtherCAT 协议栈、HPM ESC HAL 和 PHY 驱动。
Port0 使用 RTL8201F（MDIO 地址 1），Port1 关闭；SII 使用固件 Flash 只读镜像。线程周期为 1 ms。

## PDO

两方向各 **256 字节**，布局定义见 `pdo_layout.hpp`，逐字段填充，不整包 `memcpy` 业务结构体。
协议说明（给主站看）：`doc/上位机PDO协议说明-2026-11-25.md`。

| 方向（从站视角） | 内容 |
| --- | --- |
| RxPDO，主站 → 下位机 | `PdoOutputs`：`CommandHeader` 48 B + 电机命令区 128 B（预留）+ DeviceOperation 32 B（预留）+ `DiagnosticRequest` 16 B + 保留 32 B |
| TxPDO，下位机 → 主站 | `PdoInputs`：`FeedbackHeader` 40 B + `RemoteFeedback` 32 B + `ImuFeedback` 56 B + 电机反馈区 128 B（预留） |

数值采用小端，float 为 IEEE 754 binary32。`pdo_layout.hpp` 用 `static_assert` 固定总长 256 与各块偏移。

## 本版能力

- TxPDO 回传 remote 与 IMU 的最新快照，均带 `source_seq` 与采样时刻 `sample_tick_us`。
- remote/IMU 通过 `zbus_chan_read(K_NO_WAIT)` 每拍取一次，取不到保留上一帧。
- RxPDO 只做帧级校验：`magic`、`byte_length==256`、`slot_map_version`、`mcu_boot_id_echo`、OP 状态。
- 校验通过后记录 `accepted_host_session` 与 `command_seq`；掉 OP 或 100 ms 无新帧清除会话。
- **不驱动任何外设**：电机命令、DeviceOperation、DiagnosticRequest 本版均不执行。
- 电机反馈区写零，主站按"无数据"处理。

`GetOutputs()` 返回最近一次接收 PDO 快照，`GetInputs()` 返回最近一次回传 PDO 快照。
PDO 与 PDI ISR 之间只在短暂复制期间屏蔽中断，Poll、互斥锁和计算在临界区之外。

故障报告：`ecat fault`、`ecat fault clear`。
