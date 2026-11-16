# 下 pitch 重力补偿对照

固件增加 `pitch_down_gscale`，默认 1（完整补偿），0 关闭。模型是 `-0.581958*sin(theta)+1.381200*cos(theta)` N*m；theta 为归一化后、逆时针为正的实际电机角度，单位 rad。PID 与前馈相加后统一限幅并执行原边界约束，发送时按原方向取反。PID 参数不变。

需先烧录本次固件一次。后续改测试角度、等待时间和补偿比例无需烧录。保持上 pitch 姿态及机械负载不变，SA 下档、SB 上档，关闭串口助手。

```powershell
python scripts/pitch_down_gravity_check.py --port COM8
```

默认在 10、20、30、40、50、60、70° 各做一次 0%/100%/0% 对照。目标每次变化不超过 1°，速度 2°/s；每阶段先等待 5 秒再采 3 秒。起始姿态允许在 0～80° 硬限位内；开始时先对齐当前实际角度，再逐步进入测点。记录 PID、前馈、最终总力矩、反馈力矩、角度、速度与 MCU 实际补偿比例。不因单帧力矩饱和中断。沿用辨识脚本的遥控档位、日志超时、移动跟踪误差及速度检查。

结束或中止时，目标设为最后观测到的实际角度（须在硬限位内），补偿比例恢复测试前读到的值，关闭日志；不自动失能。SA 可随时失能。串口物理断开会使脚本无法恢复比例，脚本将报告命令失败；MCU 保持最后的控制参数。

输出 `pitch_down_gravity_check.csv`（稳定等待后的原始采样）、`_summary.csv`（每阶段统计）、`_comparison.csv`（每角度对照）。

“补偿了多少”按两次 0% 基线的平均 PID 绝对力矩，与 100% 时的 PID 绝对力矩比较：

`减少量 = baseline_mean_abs_PID - compensated_mean_abs_PID`

`减少比例 = 减少量 / baseline_mean_abs_PID * 100%`

同时报告绝对角度误差、RMS、抖动、峰峰值、窗口漂移、总力矩及反馈力矩。基线 PID 小于 0.05 N*m 时不报百分比。0% 前后基线不重复、采样仍有运动、样本不足或输出受限时标记对照无效；保留数据，不自动终止试验。判定阈值只用于报告，不改变控制：基线力矩差不超过 max(0.05 N*m, 25%)、误差差不超过 0.5°，各窗口标准差/首尾漂移不超过 0.2°、速度不超过 0.03 rad/s。

这里的百分比是 PID 保持负担减少比例，不是重力模型精确度。补偿后 PID 正负抵消、摩擦状态变化都需要结合角度质量判断。总保持力矩通常不会下降；原本由 PID 提供的一部分改由前馈提供。

```powershell
python scripts/pitch_down_gravity_check.py --angles 20 40 60 --settle 8 --measure 5
python scripts/pitch_down_gravity_check.py --file pitch_down_gravity_check.csv
python -m unittest discover -s scripts/tests -p "test_pitch_down_gravity*.py"
```

模拟测试覆盖完整串口时序、比例确认、字段对应、对照计算、基线漂移和异常收尾；不代表实机补偿效果。
