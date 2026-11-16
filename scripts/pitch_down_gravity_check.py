"""下 pitch 重力补偿 0%/100%/0% 对照；不修改 PID，不因力矩瞬态中断。

每个角度依次跑 0%、100%、0% 三个阶段，每阶段先从同一侧重新接近测点，
再等停稳、采样。比较两次 0% 基线平均 |PID| 与 100% 时的 |PID|，
得到前馈替 PID 分担的比例。

python scripts/pitch_down_gravity_check.py --port COM8
python scripts/pitch_down_gravity_check.py --file temp/pitch_down_gravity_check.csv

单位：CSV 时间 ms，角度 mdeg，力矩 mN*m，角速度 mrad/s。
"""
import argparse
import csv
import math
import re
import statistics as st
import time
from pathlib import Path

FIELDS = ["cycle", "angle_deg", "stage", "scale", "t_ms", "target_mdeg", "actual_mdeg",
          "total_mnm", "feedback_mnm", "omega_mrad_s", "sb_on", "pid_mnm", "gravity_mnm", "scale_permille"]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"PDCHECK,((?:-?\d+,){9}-?\d+)\s*$")
BAUD = 921600


def clean(raw):
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


class Link:
    """串口读写：PDCHECK 日志 + shell 命令确认。"""

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
        match = DATA.search(line)
        data = [int(x) for x in match.group(1).split(",")] if match else None

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

    def get_float(self, name):
        self.send(f"var get {name}")
        deadline = self.clock() + 3
        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

        while self.clock() < deadline:
            line, _ = self.read()
            match = re.fullmatch(rf"(?:{re.escape(name)}\s*=\s*)?({number})", line)

            if match and math.isfinite(float(match[1])):
                return float(match[1])

            if "not found" in line:
                break

        raise RuntimeError(f"无法读取 {name}")

    def sample(self, args):
        deadline = self.clock() + args.timeout

        while self.clock() < deadline:
            _, d = self.read()

            if not d:
                continue

            angle, speed = d[2] / 1000.0, d[5] / 1000.0

            if not 0.0 <= angle <= 80.0:
                raise RuntimeError(f"角度越界: {angle:.3f}°")

            if abs(speed) > args.max_speed:
                raise RuntimeError(f"速度保护: {speed:.3f} rad/s")

            return d

        raise RuntimeError("对照日志超时；检查 SA、遥控、电机及固件")


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


def capture(args, serial_factory=None, clock=time.monotonic):
    if serial_factory is None:
        from serial import Serial
        serial_factory = Serial

    with serial_factory(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)
        link = Link(ser, clock)
        original = None

        try:
            link.send("log off")
            deadline = clock() + 0.2
            while clock() < deadline:
                link.read()
            ser.reset_input_buffer()

            link.command("log off", "log off: ok")
            original = link.get_float("pitch_down_gscale")
            link.command("log on pitch_down_gravity 10", "log on: ok")
            link.command("log mode pitch_down_gravity on", "log mode: vofa")

            d = link.sample(args)
            initial = d[2] / 1000.0
            link.command(f"var set pitch_down_target {initial:.3f}")

            deadline = clock() + args.timeout
            while abs(link.sample(args)[1] / 1000.0 - initial) > 0.002:
                if clock() > deadline:
                    raise RuntimeError("初始目标未生效")

            for cycle in range(args.cycles):
                for angle in args.angles:
                    for stage, scale in enumerate((0, 1, 0)):
                        print(f"[stage] cycle={cycle + 1} angle={angle:g}° {scale * 100}% "
                              f"settle={args.settle:g}s measure={args.measure:g}s", flush=True)
                        link.command(f"var set pitch_down_gscale {scale}")

                        # 每阶段从同一侧重新接近测点，避免滞回差异
                        approach = angle - 5 if angle - 5 >= 10 else angle + 5
                        move(link, approach, args)

                        approach_start = link.sample(args)[0]
                        while ((link.sample(args)[0] - approach_start) & 0xFFFFFFFF) / 1000.0 < args.settle:
                            pass

                        move(link, angle, args)

                        start = None
                        deadline = clock() + args.timeout
                        count = 0

                        while True:
                            d = link.sample(args)

                            if abs(d[1] / 1000.0 - angle) > 0.002:
                                raise RuntimeError("MCU 目标被改变")

                            if d[9] != scale * 1000:
                                if clock() > deadline:
                                    raise RuntimeError("MCU 补偿比例未生效")
                                continue

                            if start is None:
                                start = d[0]

                            elapsed = ((d[0] - start) & 0xFFFFFFFF) / 1000.0

                            if elapsed >= args.settle:
                                writer.writerow([cycle, angle, stage, scale] + d)
                                count += 1

                            if elapsed >= args.settle + args.measure:
                                break

                        output.flush()
                        print(f"[stage done] n={count}", flush=True)
        finally:
            try:
                if link.latest and 0.0 <= link.latest[2] / 1000.0 <= 80.0:
                    link.command(f"var set pitch_down_target {link.latest[2] / 1000.0:.3f}")
                if original is not None:
                    link.command(f"var set pitch_down_gscale {original}")
            finally:
                link.send("log off")
                output.flush()
                print(f"[saved] {args.save}", flush=True)


def write_table(path, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def analyze(path):
    with open(path, encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    groups = {}

    for r in rows:
        key = (int(r["cycle"]), float(r["angle_deg"]), int(r["stage"]))
        groups.setdefault(key, []).append(r)

    summaries = []

    for (cycle, angle, stage), chunk in sorted(groups.items()):
        errors = [(float(r["actual_mdeg"]) - float(r["target_mdeg"])) / 1000.0 for r in chunk]
        pid = [float(r["pid_mnm"]) / 1000.0 for r in chunk]
        actual = [float(r["actual_mdeg"]) / 1000.0 for r in chunk]
        n = max(1, len(chunk) // 5)
        drift = st.mean(actual[-n:]) - st.mean(actual[:n])

        summaries.append(dict(
            cycle=cycle, angle_deg=angle, stage=stage, scale=int(chunk[0]["scale"]),
            samples=len(chunk), pid_mean_nm=st.mean(pid), pid_abs_nm=st.mean(map(abs, pid)),
            ff_mean_nm=st.mean(float(r["gravity_mnm"]) / 1000.0 for r in chunk),
            total_mean_nm=st.mean(float(r["total_mnm"]) / 1000.0 for r in chunk),
            feedback_mean_nm=st.mean(float(r["feedback_mnm"]) / 1000.0 for r in chunk),
            error_mean_deg=st.mean(errors), error_abs_deg=st.mean(map(abs, errors)),
            error_rms_deg=math.sqrt(st.mean(e * e for e in errors)),
            jitter_deg=st.pstdev(actual), span_deg=max(actual) - min(actual), drift_deg=drift,
            max_speed_rad_s=max(abs(float(r["omega_mrad_s"])) / 1000.0 for r in chunk),
            limited_samples=sum(abs(float(r["pid_mnm"]) + float(r["gravity_mnm"]) - float(r["total_mnm"])) > 3
                                for r in chunk)))

    if not summaries:
        raise ValueError("CSV 没有采样")

    base = str(Path(path).with_suffix(""))
    write_table(base + "_summary.csv", summaries)
    comparisons = []

    for cycle, angle in sorted({(r["cycle"], r["angle_deg"]) for r in summaries}):
        stages = {r["stage"]: r for r in summaries if r["cycle"] == cycle and r["angle_deg"] == angle}

        if set(stages) != {0, 1, 2}:
            print(f"[incomplete] cycle={cycle + 1} angle={angle:g}° 阶段不全，不比较", flush=True)
            continue

        a, on, b = (stages[i] for i in range(3))
        baseline = a["pid_abs_nm"]
        reduction = baseline - on["pid_abs_nm"]
        percent = 100 * reduction / baseline if baseline >= 0.05 else None
        baseline_error = a["error_abs_deg"]
        repeatable = (abs(a["pid_mean_nm"] - b["pid_mean_nm"]) <= max(0.05, baseline * 0.25)
                      and abs(a["error_mean_deg"] - b["error_mean_deg"]) <= 0.5)
        stable = all(r["samples"] >= 10 and r["jitter_deg"] <= 0.2 and abs(r["drift_deg"]) <= 0.2
                     and r["max_speed_rad_s"] <= 0.03 and r["limited_samples"] == 0 for r in (a, on, b))
        quality = repeatable and stable
        reasons = []

        if not repeatable:
            reasons.append(f"关闭前后PID={a['pid_mean_nm']:.3f}/{b['pid_mean_nm']:.3f} N*m、"
                           f"误差={a['error_mean_deg']:.3f}/{b['error_mean_deg']:.3f}°不重复")

        for index, r in enumerate((a, on, b)):
            if r["samples"] < 10:
                reasons.append(f"阶段{index}样本不足")
            if r["jitter_deg"] > 0.2:
                reasons.append(f"阶段{index}抖动={r['jitter_deg']:.3f}°")
            if abs(r["drift_deg"]) > 0.2:
                reasons.append(f"阶段{index}漂移={r['drift_deg']:.3f}°")
            if r["max_speed_rad_s"] > 0.03:
                reasons.append(f"阶段{index}速度={r['max_speed_rad_s']:.3f} rad/s")
            if r["limited_samples"]:
                reasons.append(f"阶段{index}限幅={r['limited_samples']}帧")

        comparison = dict(
            cycle=cycle, angle_deg=angle, baseline_abs_pid_nm=baseline,
            compensated_abs_pid_nm=on["pid_abs_nm"], reduction_nm=reduction,
            reduction_percent=percent, baseline_abs_error_deg=baseline_error,
            compensated_abs_error_deg=on["error_abs_deg"],
            error_reduction_deg=baseline_error - on["error_abs_deg"],
            baseline_jitter_deg=max(a["jitter_deg"], b["jitter_deg"]),
            compensated_jitter_deg=on["jitter_deg"],
            repeatable_baseline=repeatable, stable_unlimited=stable, comparison_valid=quality)

        comparison.update(
            pid_after_nm=b["pid_abs_nm"],
            reduction_vs_after_percent=(100 * (b["pid_abs_nm"] - on["pid_abs_nm"]) / b["pid_abs_nm"]
                                        if b["pid_abs_nm"] else None),
            ff_mean_nm=on["ff_mean_nm"], total_before_nm=a["total_mean_nm"],
            total_on_nm=on["total_mean_nm"], total_after_nm=b["total_mean_nm"],
            reason="; ".join(reasons))

        comparisons.append(comparison)
        percent_text = f"{percent:+.1f}%" if percent is not None else "基线接近零，不计算百分比"

        print(f"[compare] {angle:g}° PID |τ| {baseline:.4f} -> {on['pid_abs_nm']:.4f} N*m; "
              f"减少 {reduction:+.4f} N*m ({percent_text}); "
              f"|误差| {baseline_error:.3f} -> {on['error_abs_deg']:.3f}°; "
              f"抖动 {comparison['baseline_jitter_deg']:.3f} -> {on['jitter_deg']:.3f}°; "
              f"关闭后PID={b['pid_abs_nm']:.3f} N*m 前馈={on['ff_mean_nm']:+.3f} N*m；"
              f"{'对照数据有效' if quality else '; '.join(reasons)}", flush=True)

    if comparisons:
        write_table(base + "_comparison.csv", comparisons)

    print("[note] 百分比是 PID 保持负担减少比例，不是重力模型精确度；总保持力矩不要求下降。", flush=True)

    return comparisons


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--angles", nargs="+", type=float, default=[10, 20, 30, 40, 50, 60, 70])
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument("--rate", type=float, default=2.0, help="目标移动速度 deg/s")
    p.add_argument("--settle", type=float, default=5.0, help="到位后等待 s")
    p.add_argument("--measure", type=float, default=3.0, help="采样时长 s")
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--max-error", type=float, default=5.0)
    p.add_argument("--max-speed", type=float, default=0.5)
    p.add_argument("--save", default="temp/pitch_down_gravity_check.csv")
    p.add_argument("--file", help="只分析已有 CSV")
    return p


if __name__ == "__main__":
    args = parser().parse_args()

    if args.file:
        analyze(args.file)
    else:
        if not all(math.isfinite(a) and 10 <= a <= 70 for a in args.angles):
            raise ValueError("测试角度必须在 10～70° 内")
        capture(args)
        analyze(args.save)
