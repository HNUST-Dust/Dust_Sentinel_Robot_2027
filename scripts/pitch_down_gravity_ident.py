"""下 pitch 重力保持力矩辨识：锁目标角、关重力前馈，纯靠 PID 顶住。

固件侧 `pitch_down_ident=1` 时目标角只取 `pitch_down_target`，重力前馈强制为 0，
PID 输出即维持该角度所需的全部力矩。结构照 pitch_ident.py。

python scripts/pitch_down_gravity_ident.py --port COM8
python scripts/pitch_down_gravity_ident.py --file temp/pitch_down_gravity.csv

单位：CSV 时间 ms，角度 mdeg，力矩 mN*m。
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "point", "phase", "target_mdeg", "angle_mdeg", "torque_mnm", "pid_mnm",
          "int_mnm", "fb_mnm", "omega_mrad_s", "imu_pitch_mdeg", "imu_roll_mdeg",
          "up_angle_mdeg", "up_torque_mnm"]
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
# PDIDENT: t, target, actual, total, pid, integral, fb, omega, [imu_pitch, imu_roll, up_angle, up_torque,] sb
# 兼容 9 列（旧固件）和 13 列（带耦合信息）两种格式
DATA = re.compile(r"PDIDENT,((?:-?\d+,){8,12}-?\d+)\s*$")

BAUD = 921600


def clean(raw):
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


class Link:
    """串口读写封装：子类可替换 data_pattern 匹配不同的 DBG 日志。"""
    data_pattern = DATA

    def __init__(self, ser, clock=time.monotonic):
        self.ser = ser
        self.clock = clock
        self.latest = None
        self.last_beat = clock()

    def send(self, command):
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()

    def read(self):
        line = clean(self.ser.readline())
        match = self.data_pattern.search(line)
        data = [int(x) for x in match.groups()] if match else None
        if data:
            self.latest = data
        if self.clock() - self.last_beat >= 1:
            print(f"[hb] {self.latest}", flush=True)
            self.last_beat = self.clock()
        return line, data

    def command(self, command, prefix="ok"):
        self.send(command)
        deadline = self.clock() + 3
        while self.clock() < deadline:
            line, _ = self.read()
            if "not found" in line or line.startswith(("error", "usage:")):
                raise RuntimeError(f"命令失败: {command}: {line}")
            if line == prefix or line.startswith(prefix + " "):
                return
        raise RuntimeError(f"命令超时: {command}")

    def sample(self, args):
        deadline = self.clock() + args.timeout
        while self.clock() < deadline:
            _, d = self.read()
            if not d:
                continue
            angle, speed = d[2] / 1000.0, d[5] / 1000.0
            if not 0 <= angle <= 80:
                raise RuntimeError(f"角度越界: {angle:.3f}°")
            if abs(speed) > args.max_speed:
                raise RuntimeError(f"速度保护: {speed:.3f} rad/s")
            return d
        raise RuntimeError("辨识日志超时；检查 SA、遥控、电机及固件")


def move(link, target, args):
    """每次最多 1° 逼近目标，等 MCU 确认目标已跟随。"""
    current = link.latest[1] / 1000.0

    while abs(current - target) > 0.001:
        current += max(-1.0, min(1.0, target - current))
        link.command(f"var set pitch_down_target {current:.3f}")

        start = link.clock()
        while True:
            d = link.sample(args)

            if abs(d[1] / 1000.0 - current) > 0.002:
                if link.clock() - start > args.timeout:
                    raise RuntimeError("MCU 目标未跟随 var")
                continue

            if abs(d[2] / 1000.0 - current) > args.max_error:
                raise RuntimeError("移动跟踪误差过大")

            if link.clock() - start >= 1.0 / args.rate:
                break


def parser():
    p = argparse.ArgumentParser(description="下 pitch 重力保持力矩辨识")
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--low", type=float, default=10.0, help="最低目标角（度）")
    p.add_argument("--high", type=float, default=70.0, help="最高目标角（度）")
    p.add_argument("--points", type=int, default=9, help="均匀分布的测量角数量")
    p.add_argument("--cycles", type=int, default=2, help="往复轮数")
    p.add_argument("--settle-ms", type=int, default=5000, help="每个角度停留时间")
    p.add_argument("--target-tol", type=float, default=0.05, help="目标到位容差 度")
    p.add_argument("--stable-span", type=float, default=0.05, help="稳态判定：最近 20 帧角度峰峰值 度")
    p.add_argument("--save", default="temp/pitch_down_gravity.csv")
    p.add_argument("--file", help="只分析已有 CSV")
    return p


def validate(a):
    if not 0.0 <= a.low < a.high <= 80.0:
        raise ValueError("测量范围须在 0~80° 内")
    if a.points < 3 or a.cycles < 1 or a.settle_ms <= 0:
        raise ValueError("points/cycles/settle-ms 必须为正且 points ≥ 3")


def capture(args):
    import serial

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        def send(command):
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        def receive():
            line = clean(ser.readline())
            match = DATA.search(line)
            if match:
                return line, [int(x) for x in match.group(1).split(",")]
            return line, None

        def configure(command):
            send(command)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                line, _ = receive()
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        ser.reset_input_buffer()
        configure("log off")
        configure("var set pitch_down_ident 1")
        configure("log mode pitch_down_ident on")
        configure("log on pitch_down_ident 1")

        points = [args.low + (args.high - args.low) * i / max(1, args.points - 1) for i in range(args.points)]

        start = beat = time.monotonic()
        print(f"[i] 目标点 {points}", flush=True)

        # 首轮正向各点的实际角度，直接作为后续轮次的目标
        actual_ref = {}

        # 每个目标点只在"目标已到位 + 角度稳定"之后才开始写入
        def collect(target: float, point: float, direction: int) -> float:
            settle_until = time.monotonic() + args.settle_ms / 1000.0
            last = None
            recent = []
            written = 0

            while time.monotonic() < settle_until:
                line, data = receive()
                if not data:
                    continue

                last = data
                recent.append(data[2] / 1000.0)
                recent = recent[-20:]

                # 目标到位且最近 20 帧角度波动小于稳定阈值，才认为进入稳态
                tgt_ok = abs(data[1] / 1000.0 - target) <= args.target_tol
                stable = (len(recent) == 20
                          and max(recent) - min(recent) <= args.stable_span)

                if not (tgt_ok and stable):
                    continue

                t_ms = int((time.monotonic() - start) * 1000)
                # data[1..5]=target/actual/total/pid/integral
                # data[6..9]=fb/omega/imu_pitch/imu_roll、data[10..11]=up_angle/up_torque（13 列时）
                fields = data[1:6]
                fields += data[6:10] if len(data) >= 13 else [0, 0, 0, 0]
                fields += data[10:12] if len(data) >= 13 else [0, 0]
                writer.writerow([t_ms, f"{point:.2f}", direction] + fields)
                written += 1

            if last is None:
                raise RuntimeError(f"{point:.2f}° 采集阶段没有收到任何 PDIDENT 数据；检查日志条目与固件")

            if written == 0:
                print(f"[!] {point:.2f}° 未进入稳态（目标 {target:.2f}°，实际 {last[2]/1000.0:.3f}°），"
                      f"本点无样本", flush=True)

            return last[2] / 1000.0

        try:
            # 一轮内正向走完立刻反向回来，保证每个点都有正反两次接近
            for cycle in range(args.cycles):
                for direction, sweep in ((1, points), (-1, points[::-1])):
                    for point in sweep:
                        target = round(actual_ref.get(point, point), 2)

                        send(f"var set pitch_down_target {target:.2f}")
                        actual = collect(target, point, direction)

                        if point not in actual_ref:
                            actual_ref[point] = actual
                            print(f"[ref] {point:.2f}° 实际 {actual:.3f}° 作为后续目标", flush=True)
                        else:
                            print(f"[dir {direction:+d}] {point:.2f}° 目标 {target:.2f}° "
                                  f"实际 {actual:.3f}° 偏差 {actual - actual_ref[point]:+.3f}°", flush=True)

                        output.flush()
                        now = time.monotonic()
                        if now - beat >= 1:
                            print(f"[hb] {now-start:.1f}s cycle={cycle} dir={direction:+d} "
                                  f"point={point:.1f} target={target:.2f} actual={actual:.3f}", flush=True)
                            beat = now
        finally:
            send("var set pitch_down_ident 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {args.save}", flush=True)


def analyze(path):
    import numpy as np

    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("CSV 为空")

    groups = {}
    for row in rows:
        key = (row["point"], row["phase"])
        groups.setdefault(key, []).append((float(row["angle_mdeg"]) / 1000.0,
                                           float(row["torque_mnm"]) / 1000.0,
                                           float(row["pid_mnm"]) / 1000.0,
                                           float(row["int_mnm"]) / 1000.0,
                                           float(row["imu_pitch_mdeg"]) / 1000.0,
                                           float(row["up_angle_mdeg"]) / 1000.0,
                                           float(row["up_torque_mnm"]) / 1000.0))

    print(f"[i] 共 {len(rows)} 帧，{len(groups)} 组")
    print(f"{'point':>7} {'dir':>4} {'angle':>8} {'std':>6} {'total':>8} {'pid':>8} {'int':>8} "
          f"{'imuP':>8} {'upAng':>8} {'upTrq':>8} {'n':>5}")
    for key in sorted(groups, key=lambda k: (float(k[0]), k[1])):
        v = np.asarray(groups[key])
        print(f"{float(key[0]):7.2f} {key[1]:>4} {v[:, 0].mean():8.3f} {v[:, 0].std():6.4f} "
              f"{v[:, 1].mean():8.4f} {v[:, 2].mean():8.4f} {v[:, 3].mean():8.4f} "
              f"{v[:, 4].mean():8.3f} {v[:, 5].mean():8.3f} {v[:, 6].mean():8.4f} {len(v):5d}")

    # 上轴姿态是否真的稳定（耦合影响判据）
    imu_span = np.ptp([float(r["imu_pitch_mdeg"]) / 1000.0 for r in rows])
    up_span = np.ptp([float(r["up_angle_mdeg"]) / 1000.0 for r in rows])
    print(f"[couple] IMU pitch 全程范围 {imu_span:.3f}°，上轴角度范围 {up_span:.3f}°")
    if imu_span > 0.5 or up_span > 0.5:
        print("[couple] 上轴姿态在变化，下轴重力辨识会混入上轴耦合，结论需谨慎")

    # 往复两轮按实际角度插值配对：同一 θ 上比较正反向力矩，避免把重力矩差算进摩擦
    by_phase = {}
    for (point, phase), values in groups.items():
        by_phase.setdefault(int(phase), []).append(np.asarray(values))

    if len(by_phase) < 2:
        print("[!] 只有单个方向，无法分离摩擦；跳过配对")
        return

    # 每个方向按实际角度排序；用积分输出作为保持力矩（PI 稳态下 P 项趋零）
    profiles = {}
    for phase, chunks in by_phase.items():
        chunks.sort(key=lambda c: c[:, 0].mean())
        angles = np.asarray([c[:, 0].mean() for c in chunks])
        torques = np.asarray([c[:, 3].mean() for c in chunks])
        profiles[phase] = (angles, torques)

    phases = sorted(profiles)
    forward, backward = profiles[phases[0]], profiles[phases[1]]

    # 公共角度区间：两方向都覆盖到的部分
    lo = max(forward[0][0], backward[0][0])
    hi = min(forward[0][-1], backward[0][-1])
    if hi - lo < 1.0:
        print(f"[!] 正反向公共角度区间只有 {hi-lo:.2f}°，无法配对")
        return

    common = np.linspace(lo, hi, max(3, len(forward[0])))
    tau_f = np.interp(common, forward[0], forward[1])
    tau_b = np.interp(common, backward[0], backward[1])
    gravity = (tau_f + tau_b) / 2.0
    friction = np.abs(tau_f - tau_b) / 2.0

    theta = np.deg2rad(common)
    gravity = (tau_f + tau_b) / 2.0
    friction = np.abs(tau_f - tau_b) / 2.0

    print(f"\n[i] 公共区间 {lo:.2f}° ~ {hi:.2f}°，配对 {len(common)} 点")
    print(f"{'angle':>8} {'tau_fwd':>10} {'tau_bwd':>10} {'gravity':>10} {'friction':>10}")
    for i in range(len(common)):
        print(f"{common[i]:8.2f} {tau_f[i]:10.4f} {tau_b[i]:10.4f} {gravity[i]:10.4f} {friction[i]:10.4f}")

    # 两种模型都拟合：带常数项可吸收"上 pitch 保持水平时传到下轴的恒定重力矩"
    models = {
        "A*sin+B*cos": np.column_stack([np.sin(theta), np.cos(theta)]),
        "A*sin+B*cos+C": np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)]),
    }

    for name, design in models.items():
        coeff, _, rank, _ = np.linalg.lstsq(design, gravity, rcond=None)
        pred = design @ coeff
        rms = float(np.sqrt(np.mean((pred - gravity) ** 2)))
        terms = " ".join(f"{v:+.6f}*{'sin' if i == 0 else ('cos' if i == 1 else '1')}"
                         for i, v in enumerate(coeff))
        print(f"[fit] {name:<15} {terms} N*m  RMS={rms:.5f} 条件数={np.linalg.cond(design):.1f}")

    print(f"[fit] 摩擦 平均={friction.mean():.4f} 中位={np.median(friction):.4f} "
          f"最小={friction.min():.4f} 最大={friction.max():.4f} N*m")
    print("[note] 常数项 C 可能来自上 pitch 保持水平时的恒定重力矩，也可能是其他偏置；"
          "不能单凭 RMS 下降认定它是重力。", flush=True)


def main():
    args = parser().parse_args()

    if args.file:
        analyze(args.file)
        return

    validate(args)
    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
