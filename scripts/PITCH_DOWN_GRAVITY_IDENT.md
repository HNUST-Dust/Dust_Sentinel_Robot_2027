# 下 pitch 重力保持力矩辨识

先编译并烧录带 `pitch_down_ident` 日志的固件。保持上 pitch 姿态和机械负载不变；SA 下档、SB 上档，下 pitch 当前实际角度在 0～80° 硬限位内，脚本先以不超过 1° 的小步移动到 10°。关闭占用 COM8 的串口助手。

```powershell
python scripts/pitch_down_gravity_ident.py --port COM8
```

默认 10～70°、5° 测点间隔、两个往复周期。目标按不超过 1° 的小步、2°/s 移动，每点先等待 5 秒，再采集 3 秒；全程约 9 分钟。不修改 PID，不自动烧录，不添加重力前馈。脚本持续输出阶段和心跳。

角度、指令力矩、反馈力矩和角速度统一为逆时针正方向；原始 CSV 的单位见列名（ms、mdeg、mN*m、mrad/s）。只收通过静止检查的数据。移动超速、过大跟踪误差、SB 离开上档或日志超时会终止采集；SA 仍由原控制线程处理失能。退出时将 var 目标设为最后观测的实际角度（仅当该角度在 0～80° 硬限位内），关闭日志，不恢复旧目标，不自动失能。PC 断开时没有额外 MCU 看门狗，原目标仍由位置环保持。

参考 `pitch_ident.py`，按实际角度插值配对正反向保持力矩，均值拟合 `A*sin(theta)+B*cos(theta)`，theta 单位 rad。正反向均值仅在摩擦近似对称、姿态稳定时估计重力保持力矩；半差是保持滞回量，不能直接宣称是库仑摩擦。端点没有双向接近过程，拟合仅采用两方向的实际角度公共区间，默认主要覆盖 15～65°。

输出 `pitch_down_gravity.csv`、`pitch_down_gravity_paired.csv`、`pitch_down_gravity_fit.json`。CSV 同时保留指令和反馈力矩；拟合用指令力矩，未校准电机实际输出力矩，不代表外部力矩传感器测量。

```powershell
python scripts/pitch_down_gravity_ident.py --file pitch_down_gravity.csv
python -m unittest discover -s scripts/tests -p test_pitch_down_gravity_ident.py
```

自测使用模拟串口和已知重力曲线，覆盖命令顺序、完整往复采集、列序、拟合和保护退出；不代表硬件辨识结果。失败时保留已完成测点的 CSV，整轮未完成的数据不应当作完整验证结论。
