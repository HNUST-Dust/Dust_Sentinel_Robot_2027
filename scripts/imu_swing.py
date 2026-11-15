"""IMU 摆动数据采集与漂移分析。

采集：PC 打开 DBG 流，用户手动上下摆动云台，脚本记录 imu_raw。
分析：陀螺积分 vs 解算角对比、加速度模长、静止段漂移。

python scripts/imu_swing.py --port COM8 --seconds 30
python scripts/imu_swing.py --file imu_swing.csv
"""
import argparse
import csv
import re
import time
from pathlib import Path

FIELDS = [
    "t_ms",
    "gx_mrad", "gy_mrad", "gz_mrad",
    "ax_mm", "ay_mm", "az_mm",
    "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+(?:,-?\d+){8})\s*$")
BAUD = 921600
GRAVITY = 9.80665


def clean(raw: bytes) -> str:
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


def capture(args) -> None:
    import serial

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        def send(command: str) -> None:
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        def configure(command: str) -> None:
            send(command)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                line = clean(ser.readline())
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        ser.reset_input_buffer()
        configure("log off")
        configure("log mode imu_raw on")
        configure(f"log on imu_raw {args.divisor}")

        print(f"[i] 采集 {args.seconds}s，请上下摆动云台", flush=True)
        start = time.monotonic()
        count = 0
        last_beat = start

        try:
            while time.monotonic() - start < args.seconds:
                line = clean(ser.readline())
                match = DATA.search(line)
                if not match:
                    continue

                values = [int(x) for x in match.group(1).split(",")]
                t_ms = int((time.monotonic() - start) * 1000)
                writer.writerow([t_ms] + values)
                count += 1

                now = time.monotonic()
                if now - last_beat >= 2:
                    output.flush()
                    print(f"[hb] {now - start:.0f}s 样本={count}", flush=True)
                    last_beat = now
        finally:
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)


def analyze(path: str) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    t = np.array([float(r["t_ms"]) / 1000.0 for r in rows])
    gx = np.array([float(r["gx_mrad"]) / 1000.0 for r in rows])
    gy = np.array([float(r["gy_mrad"]) / 1000.0 for r in rows])
    gz = np.array([float(r["gz_mrad"]) / 1000.0 for r in rows])
    ax = np.array([float(r["ax_mm"]) / 1000.0 for r in rows])
    ay = np.array([float(r["ay_mm"]) / 1000.0 for r in rows])
    az = np.array([float(r["az_mm"]) / 1000.0 for r in rows])
    roll = np.array([float(r["roll_mdeg"]) / 1000.0 for r in rows])
    pitch = np.array([float(r["pitch_mdeg"]) / 1000.0 for r in rows])
    yaw = np.array([float(r["yaw_mdeg"]) / 1000.0 for r in rows])

    acc_norm = np.sqrt(ax * ax + ay * ay + az * az)
    dt = np.diff(t, prepend=t[0])
    dt[0] = np.median(dt[1:]) if len(dt) > 1 else 0.001

    print(f"[i] 样本 {len(rows)}，时长 {t[-1] - t[0]:.1f}s，平均 {1.0 / np.mean(dt):.0f} Hz")
    print()

    # 1) 加速度模长：静止应约 9.81，摆动时运动加速度混入会偏离
    print("[加速度模长] 静止时约 9.81 m/s^2，偏离说明有运动加速度混入")
    print(f"  mean={acc_norm.mean():.3f}  std={acc_norm.std():.3f}")
    print(f"  min ={acc_norm.min():.3f}  max={acc_norm.max():.3f}")
    dev = np.abs(acc_norm - GRAVITY)
    print(f"  偏离 >0.5 m/s^2 的比例: {100.0 * np.mean(dev > 0.5):.1f}%")
    print(f"  偏离 >1.0 m/s^2 的比例: {100.0 * np.mean(dev > 1.0):.1f}%")
    print()

    # 2) 陀螺零偏：静止段（角速度小、加速度接近重力）的均值
    still = (np.abs(gx) < 0.1) & (np.abs(gy) < 0.1) & (np.abs(gz) < 0.1) & (dev < 0.3)
    print(f"[陀螺零偏] 判定静止样本 {int(still.sum())} / {len(rows)}")
    if still.sum() > 10:
        print(f"  gx={gx[still].mean():+.4f} gy={gy[still].mean():+.4f} gz={gz[still].mean():+.4f} rad/s")
        print(f"  (静止段标准差 gx={gx[still].std():.4f} gy={gy[still].std():.4f} gz={gz[still].std():.4f})")
    else:
        print("  静止样本太少，跳过")
    print()

    # 3) 陀螺积分角 vs EKF 解算角
    print("[陀螺积分 vs EKF] 积分角漂移大 = 零偏未补偿或积分误差累积")
    for name, w, ang in (("pitch(x)", gx, pitch), ("roll(y)", gy, roll), ("yaw(z)", gz, yaw)):
        integrated = np.degrees(np.cumsum(w * dt))
        integrated -= integrated[0]
        delta = ang - ang[0]
        err = integrated - delta
        print(f"  {name:10s} 积分末值={integrated[-1]:+8.2f}°  EKF变化={delta[-1]:+8.2f}°  "
              f"残差末值={err[-1]:+8.2f}°  残差峰峰={err.max() - err.min():7.2f}°")
    print()

    # 4) 摆动幅度与角速度峰值
    print("[摆动幅度]")
    for name, ang in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
        print(f"  {name:6s} min={ang.min():+8.2f}° max={ang.max():+8.2f}° 峰峰={ang.max() - ang.min():7.2f}°")
    for name, w in (("gx", gx), ("gy", gy), ("gz", gz)):
        print(f"  {name:6s} 峰值={np.abs(w).max():6.3f} rad/s")
    print()

    # 5) 静止段姿态漂移：连续静止样本内的角度变化
    print("[静止段漂移] 连续静止区间内角度应保持不变")
    if still.sum() > 10:
        idx = np.flatnonzero(still)
        runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        longest = max(runs, key=len)
        if len(longest) > 10:
            span = t[longest[-1]] - t[longest[0]]
            print(f"  最长静止段 {span:.2f}s（{len(longest)} 样本）")
            print(f"  pitch 变化 {pitch[longest[-1]] - pitch[longest[0]]:+.3f}°")
            print(f"  roll  变化 {roll[longest[-1]] - roll[longest[0]]:+.3f}°")
            print(f"  yaw   变化 {yaw[longest[-1]] - yaw[longest[0]]:+.3f}°")
        else:
            print("  无足够长的连续静止段")
    else:
        print("  静止样本太少")


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU 摆动数据采集与漂移分析")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--seconds", type=float, default=30.0, help="采集时长（秒）")
    parser.add_argument("--divisor", type=int, default=1, help="DBG 分频，1=每拍都打")
    parser.add_argument("--save", default="imu_swing.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
