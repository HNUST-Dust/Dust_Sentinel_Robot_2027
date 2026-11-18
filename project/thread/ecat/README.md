# HPM5E31 EtherCAT 基础收发

本项目复用 `E:/Zephyr/zephyr_user` 的 EtherCAT 门面、协议栈、HPM ESC HAL 和 PHY 驱动。

| ESC 端口 | PHY | MDIO 地址 |
| --- | --- | --- |
| Port0 | RTL8201F | 1 |
| Port1 | 未焊接，关闭 | - |

按用户确认的当前实物，仅初始化 Port0 的 RTL8201F，MDIO 地址为 1。Port1 的 TJA1101B 未焊接，不创建 PHY 实例，并在 overlay 中明确关闭 Port1。ESC 使用 EEPROM 仿真模式，SII 来自固件 Flash 中的只读镜像（`CONFIG_DUST_ECAT_SII_FLASH_READONLY`），支持主站在线读取和 ESC reload，拒绝写入。不启用可写 Flash EEPROM 存储库、RAM 索引表或独立持久化分区。

`CONFIG_TRD_ECAT` 在 HPM5E31 板级 conf 中开启。任务周期 1 ms，初始化阶段先启动 ESC，再初始化 Port0 PHY，最后开放协议栈事件。

- RxPDO：主站 -> 从站，`PdoOutputs::counter[64]`，256 字节。
- TxPDO：从站 -> 主站，`PdoInputs::counter[64]`，256 字节。
- TxPDO 的 64 个字每拍填写相同的递增计数，行为与参考线程一致。

业务线程可包含 `trd_ecat.hpp`，通过 `instance::ecat::GetOutputs()` / `GetInputs()` 读取最近一次快照。接口内部加互斥锁；快照本身不代表通信有效性，未接收时为零，停更时保留旧值。本次只接入基础收发，未将 RxPDO 接入电机控制。

门面缓冲由 PDI ISR 访问，任务只在完整 PDO 复制期间屏蔽中断；`Poll()`、互斥锁和日志均在临界区之外。

故障诊断命令：`ecat fault`、`ecat fault clear`。

本次完成源代码接入与静态差异检查，尚未编译、烧录或验证主站进入 OP、双口链路及实际 PDO 收发。
