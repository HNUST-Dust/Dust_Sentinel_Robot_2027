"""上 pitch 电机角度与 IMU pitch 对比采集。

PC 逐点设置 pitch_up_target，记录电机反馈角与 IMU 解算 pitch，输出偏差表。

python scripts/pitch_imu_check.py --port COM8 --low 5 --high -60 --points 14
python scripts/pitch_imu_check.py --file pitch_imu_check.csv
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "point", "target_deg", "motor_deg", "imu_deg", "torque_nm"]
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\s*$")
BAUD = 921600


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
        configure("log mode pitch_track on")
        configure("log on pitch_track 10")

        points = [args.low + (args.high - args.low) * i / max(1, args.points - 1)
                  for i in range(args.points)]

        start = time.monotonic()
        total = 0

        try:
            for index, point in enumerate(points):
                target = round(point, 2)
                send(f"var set pitch_up_target {target}")
                print(f"[{index + 1}/{len(points)}] 目标 {target:+.1f}°", flush=True)

                settle_until = time.monotonic() + args.settle_ms / 1000.0
                while time.monotonic() < settle_until:
                    line = clean(ser.readline())
                    match = DATA.search(line)
                    if not match:
                        continue

                    values = [float(x) for x in match.groups()]
                    t_ms = int((time.monotonic() - start) * 1000)
                    writer.writerow([t_ms, f"{point:.2f}"] + values)
                    total += 1

            output.flush()
            print(f"[i] 已保存 {total} 条到 {args.save}", flush=True)
        finally:
            send("log off")


def analyze(path: str) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    groups = {}
    for row in rows:
        groups.setdefault(row["point"], []).append(
            (float(row["motor_deg"]), float(row["imu_deg"]), float(row["torque_nm"])))

    print(f"{'target':>8} {'motor':>9} {'imu':>9} {'imu-motor':>11} {'torque':>8} {'n':>5}")
    table = []
    for point in sorted(groups, key=float):
        values = np.asarray(groups[point])
        motor = values[:, 0].mean()
        imu = values[:, 1].mean()
        torque = values[:, 2].mean()
        table.append((float(point), motor, imu, imu - motor, torque))
        print(f"{float(point):8.2f} {motor:9.2f} {imu:9.2f} {imu - motor:11.2f} {torque:8.3f} {len(values):5d}")

    data = np.asarray(table)
    offset = data[:, 3]
    print()
    print(f"[i] imu-motor 偏差  mean={offset.mean():+.2f}°  std={offset.std():.2f}°  "
          f"min={offset.min():+.2f}°  max={offset.max():+.2f}°")

    # 线性拟合：imu = k * motor + b，k 偏离 1 说明比例不一致，b 是零点偏差
    motor = data[:, 1]
    imu = data[:, 2]
    if len(motor) >= 2 and np.ptp(motor) > 1.0:
        k, b = np.polyfit(motor, imu, 1)
        residual = imu - (k * motor + b)
        print(f"[fit] imu = {k:.4f} * motor + {b:+.3f}°")
        print(f"[fit] 斜率 {k:.4f}（1.0 = 完全一致），截距 {b:+.3f}°")
        print(f"[fit] RMS = {np.sqrt(np.mean(residual ** 2)):.3f}°")


def main() -> None:
    parser = argparse.ArgumentParser(description="上 pitch 电机角与 IMU pitch 对比")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--low", type=float, default=5.0, help="起始目标角（度）")
    parser.add_argument("--high", type=float, default=-60.0, help="结束目标角（度）")
    parser.add_argument("--points", type=int, default=14, help="测量点数")
    parser.add_argument("--settle-ms", type=int, default=2500, help="每点停留时间")
    parser.add_argument("--save", default="pitch_imu_check.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
