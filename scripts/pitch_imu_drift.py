"""上 pitch 往复扫描，检查 IMU pitch 在云台运动后是否漂移。

思路：同一目标角多次经过时，IMU pitch 应当一致。各轮之间的差异即漂移量。
电机角度与 IMU 的绝对偏差不作为判据。

python scripts/pitch_imu_drift.py --port COM8 --low 5 --high -60 --points 8 --cycles 4
python scripts/pitch_imu_drift.py --file pitch_imu_drift.csv
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "cycle", "point", "target_deg", "motor_deg", "torque_nm", "imu_deg"]
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
            for cycle in range(args.cycles):
                order = points if cycle % 2 == 0 else points[::-1]
                print(f"[轮 {cycle + 1}/{args.cycles}] {'上行' if cycle % 2 == 0 else '下行'}", flush=True)

                for point in order:
                    send(f"var set pitch_up_target {round(point, 2)}")
                    settle_until = time.monotonic() + args.settle_ms / 1000.0
                    while time.monotonic() < settle_until:
                        line = clean(ser.readline())
                        match = DATA.search(line)
                        if not match:
                            continue

                        values = [float(x) for x in match.groups()]
                        t_ms = int((time.monotonic() - start) * 1000)
                        writer.writerow([t_ms, cycle, f"{point:.2f}"] + values)
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

    # (point, cycle) -> imu 均值
    groups = {}
    for row in rows:
        key = (float(row["point"]), int(row["cycle"]))
        groups.setdefault(key, []).append(float(row["imu_deg"]))

    points = sorted({k[0] for k in groups})
    cycles = sorted({k[1] for k in groups})

    print(f"{'point':>8} " + " ".join(f"{'c' + str(c):>9}" for c in cycles) +
          f" {'峰峰':>9}")
    print("-" * (10 + 10 * len(cycles) + 10))

    spreads = []
    for point in points:
        values = []
        for cycle in cycles:
            samples = groups.get((point, cycle))
            values.append(np.mean(samples) if samples else float("nan"))

        valid = [v for v in values if not np.isnan(v)]
        spread = max(valid) - min(valid) if len(valid) > 1 else 0.0
        spreads.append(spread)

        cells = " ".join(f"{v:9.2f}" if not np.isnan(v) else f"{'-':>9}" for v in values)
        print(f"{point:8.2f} {cells} {spread:9.2f}")

    spreads = np.asarray(spreads)
    print()
    print(f"[i] 各目标点的多轮 IMU 峰峰：mean={spreads.mean():.2f}°  "
          f"max={spreads.max():.2f}°  median={np.median(spreads):.2f}°")

    # 首末轮整体偏移：衡量长时间累积漂移
    if len(cycles) >= 2:
        first, last = cycles[0], cycles[-1]
        diffs = []
        for point in points:
            a = groups.get((point, first))
            b = groups.get((point, last))
            if a and b:
                diffs.append(np.mean(b) - np.mean(a))

        if diffs:
            diffs = np.asarray(diffs)
            print(f"[i] 第 {first + 1} 轮 → 第 {last + 1} 轮 IMU 偏移："
                  f"mean={diffs.mean():+.2f}°  std={diffs.std():.2f}°  "
                  f"min={diffs.min():+.2f}°  max={diffs.max():+.2f}°")

    # 运动幅度：确认扫描覆盖够大
    imu_all = np.array([float(r["imu_deg"]) for r in rows])
    print(f"[i] 全程 IMU pitch 范围 {imu_all.min():.2f}° ~ {imu_all.max():.2f}°"
          f"（峰峰 {imu_all.max() - imu_all.min():.2f}°）")


def main() -> None:
    parser = argparse.ArgumentParser(description="上 pitch 往复扫描 IMU 漂移检查")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--low", type=float, default=5.0, help="一端目标角（度）")
    parser.add_argument("--high", type=float, default=-60.0, help="另一端目标角（度）")
    parser.add_argument("--points", type=int, default=8, help="单程测量点数")
    parser.add_argument("--cycles", type=int, default=4, help="往复轮数")
    parser.add_argument("--settle-ms", type=int, default=2000, help="每点停留时间")
    parser.add_argument("--save", default="pitch_imu_drift.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
