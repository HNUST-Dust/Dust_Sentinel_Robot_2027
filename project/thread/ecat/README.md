# HPM5E31 EtherCAT 导航收发

复用 `E:/Zephyr/zephyr_user` 的 EtherCAT 协议栈、HPM ESC HAL 和 PHY 驱动。
Port0 使用 RTL8201F（MDIO 地址 1），Port1 关闭；SII 使用固件 Flash 只读镜像。线程周期为 1 ms。

## PDO 与协议包

为兼容现有主站映射，两方向仍为 **64 × uint32_t = 256 字节**，不是 15/20 字节 PDO。
协议定义见 `navigation_packet.hpp`，以字节复制，不逐个 uint32_t 转换 float。

| 方向（从站视角） | PDO 起始字节 | 内容 | 其余字节 |
| --- | --- | --- | --- |
| RxPDO，主站 -> 下位机 | 0–14 | `CmdVelPacket`，帧头 0x6B | 忽略 |
| TxPDO，下位机 -> 主站 | 0–19 | `StatusPacket`，帧头 0x6D | 全零 |

数值采用小端，float 为 IEEE 754 binary32。CRC16 初值 0xFFFF、反射多项式 0x8408、无最终异或，低字节在前。
命令 CRC 覆盖字节 0–12，存入 13–14；状态 CRC 覆盖 0–17，存入 18–19。
状态业务字段当前全部为零，仅帧头、每次任务聚合的 sequence（255 后回绕）和 CRC 正常生成。
sequence 表示从站聚合次数，主站读取时可能跳号。

## 底盘接管

EtherCAT 线程校验帧头、CRC 和三个速度的有限性，利用 SM2 收帧计数判断 PDO 更新。
通过 `extern instance::ecat::chassis_command_` 共享命令，读写均持有 `chassis_command_mutex_`。
同值命令持续下发也算更新；主站停止更新业务但继续循环发送同一 PDO，当前协议无法识别。

- SB=UP（`custom1=On`）：导航接管，`linear_x/y` 直接作为底盘系 m/s，不乘 `chassis_v`，不经过云台偏角旋转。
- 当前为固定朝向联调模式：旋转目标固定为零，导航应发送 `angular_z=0`，SC 小陀螺不叠加。
- SB 其他档位：维持遥控速度与 SC 小陀螺逻辑。
- SA 总开关关闭、遥控未收到或停更 100 ms：四轮输出电流归零。
- 每拍通过 PHY 的 `LinkUp()` 检查 Port0；检测到断链即清零底盘命令及接收 PDO 快照，不等待 100 ms 停更超时。
- 导航接管时，EtherCAT 非 OP、未收到有效命令、帧头/CRC/数值校验失败、有效 PDO 停更 100 ms：底盘命令、接收 PDO 快照及四轮输出电流归零。
- `CONFIG_TRD_ECAT=n` 时，SB=UP 不使能底盘。

`GetOutputs()` 返回接收 PDO 快照，命令无效时清零；`GetInputs()` 返回当前回传 PDO 快照。
`log on ecat_cmd 10` 打印处理后交给底盘的 `vx,vy,wz,valid`，每 10 拍输出一次；断联后显示 `0.000,0.000,0.000,0`。
PDO 与 PDI ISR 之间只在短暂复制期间屏蔽中断，Poll、互斥锁和计算在临界区之外。

故障报告：`ecat fault`、`ecat fault clear`。
实机 OP、主站映射、PDO 收发及 SB 接管效果仍需联调验证。
