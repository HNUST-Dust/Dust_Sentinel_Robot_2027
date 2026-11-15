"""C620 摩擦前馈 A/B 对比：同一段轨迹关/开前馈各跑一遍，比较四项指标。

前馈模型来自 scripts/chassis_friction.py 的空载辨识（库仑 + 黏性，正反分开），
地面负载等额外阻力由 PID 补偿——换场地不必重辨识。

四项指标：
  1. 稳态速度误差（目标 - 实际）
  2. PID 输出（前馈分担越多此值越小）
  3. 低速启动表现（1 rad/s 附近从零起步的响应）
  4. 正反转切换表现（过零时的速度波谷与电流跳变）

python scripts/chassis_ff_ab.py --port COM8
python scripts/chassis_ff_ab.py --file chassis_ff_ab.csv

单位：CSV 时间 ms，转速 mrad/s，电流 mA。
"""
import argparse
import csv
import re
import time

FIELDS = (["t_ms", "phase"]
          + [f"target{w}" for w in range(4)]
          + [f"omega{w}" for w in range(4)]
          + [f"pid{w}" for w in range(4)]
          + [f"ff{w}" for w in range(4)])

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
BAUD = 921600

# 测试轨迹：目标转速序列（rad/s），每段保持 seconds
# 覆盖静摩擦到高速全区间，含两次过零换向
TRACK = [
    (1.0, 4.0),     # 低速启动，静摩擦区
    (5.0, 3.0),
    (10.0, 3.0),
    (16.0, 3.0),    # 高速
    (20.0, 3.0),
    (-5.0, 3.0),    # 高速正转 → 反转，大跨度过零
    (-12.0, 3.0),
    (-20.0, 3.0),
    (5.0, 3.0),     # 反转 → 正转，大跨度过零
    (0.0, 2.0),     # 回零
]


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
                if "not found" in line:
                    raise RuntimeError(f"{command} 失败: {line}")
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        # 先停掉所有调试流，否则残留输出会冲掉命令回复
        ser.write(b"log off\r\n")
        ser.flush()
        time.sleep(0.3)

        ser.reset_input_buffer()
        configure("log off")

        # 开测试通道：四轮锁 0 转速，先让 DBG 条目被创建
        configure("var set chassis_test_omega 0")
        configure("var set chassis_test_enable 1")

        # 先 log on 让 chassis_ff 条目被创建出来，再切纯文本模式
        configure("log on chassis_ff 1")
        configure("log mode chassis_ff on")

        # 给固件几拍时间打出第一条，确认通道真的通
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = clean(ser.readline())
            if DATA.search(line):
                print(f"[i] 采集通道已通: {line}", flush=True)
                break
        else:
            raise RuntimeError("chassis_ff 无数据，检查 chassis_test_enable 是否生效")

        start = beat = time.monotonic()
        count = 0
        phase = 0

        def run_track(ff: float, label: str) -> None:
            nonlocal start, beat, count, phase

            send(f"var set chassis_ff {ff}")
            print(f"[i] ===== {label}（chassis_ff={ff:.0f}）=====", flush=True)
            time.sleep(1.0)

            for idx, (omega, hold) in enumerate(TRACK):
                send(f"var set chassis_test_omega {omega}")
                print(f"[i] {label} 段{idx} 目标 {omega:+.1f} rad/s 保持 {hold:.0f}s", flush=True)

                hold_until = time.monotonic() + hold
                while time.monotonic() < hold_until:
                    line = clean(ser.readline())
                    match = DATA.search(line)
                    if not match:
                        continue

                    values = [int(x) for x in match.groups()]
                    t_ms = int((time.monotonic() - start) * 1000)
                    writer.writerow([t_ms, phase] + values)
                    count += 1

                    now = time.monotonic()
                    if now - beat >= 2:
                        output.flush()
                        wheels = " ".join(f"w{i}={values[4 + i] / 1000.0:+.2f}" for i in range(4))
                        print(f"[hb] {label} 段{idx} 目标={values[0] / 1000.0:+.2f} "
                              f"{wheels} PID输出={values[8] / 1000.0:+.3f} 前馈I={values[12] / 1000.0:+.3f} "
                              f"样本={count}", flush=True)
                        beat = now

                phase += 1

            send("var set chassis_test_omega 0")
            time.sleep(1.0)

        try:
            run_track(0.0, "A-无前馈")
            run_track(1.0, "B-有前馈")
        finally:
            send("var set chassis_test_enable 0")
            send("var set chassis_test_omega 0")
            send("var set chassis_ff 1")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)


def analyze(path: str, wheel_filter: int | None = None) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    # 前 7 个 phase 是 A（无前馈），后 7 个是 B（有前馈）
    phases = sorted({int(r["phase"]) for r in rows})
    half = len(phases) // 2

    print(f"[i] 样本 {len(rows)}，段 {len(phases)} 个（A 段 {half}，B 段 {len(phases) - half}）")

    wheels = list(range(4)) if wheel_filter is None else [wheel_filter]

    labels = ["低速启动", "中速5", "中高10", "高速16", "最高20",
              "换向-5", "反向12", "反向20", "换向+5", "回零"]

    for wheel in wheels:
        print()
        print(f"=== 轮 {wheel} ===")
        print(f"{'指标':>12} {'A无前馈':>12} {'B有前馈':>12} {'变化':>10}")

        # 指标1：稳态速度误差（每段末 30%）
        err_a, err_b = [], []
        # 指标2：PID 输出（同段末 30%，取绝对值；前馈分担越多此值越小）
        int_a, int_b = [], []
        # 指标3：低速启动到 90% 目标所需时间
        rise_a = rise_b = None
        # 指标4：过零波谷（换向段的 |omega| 最小值）
        cross_a, cross_b = [], []

        tgt_col = f"target{wheel}"
        om_col = f"omega{wheel}"
        pid_col = f"pid{wheel}"

        for p_a, p_b in zip(phases[:half], phases[half:]):
            for p, err_list, int_list in ((p_a, err_a, int_a), (p_b, err_b, int_b)):
                seg = [r for r in rows if int(r["phase"]) == p]
                if not seg:
                    continue

                n = len(seg)
                tail = seg[int(0.7 * n):] if n > 3 else seg
                tgt = np.array([int(r[tgt_col]) for r in tail]) / 1000.0
                om = np.array([int(r[om_col]) for r in tail]) / 1000.0
                ii = np.abs(np.array([int(r[pid_col]) for r in tail])) / 1000.0

                err_list.append(float(np.mean(tgt - om)))
                int_list.append(float(np.median(ii)))

                # 低速启动：第 0 段从段首到首次达到 90% 目标的时间（统一用 mrad/s 比较）
                idx = phases.index(p_a) if p == p_a else phases.index(p_b)
                if idx % len(TRACK) == 0:
                    t0 = int(seg[0]["t_ms"])
                    target_mrad = abs(int(seg[0][tgt_col]))
                    reach = [r for r in seg if abs(int(r[om_col])) >= 0.9 * target_mrad]
                    dt = (int(reach[0]["t_ms"]) - t0) / 1000.0 if reach else float("nan")

                    if p == p_a:
                        rise_a = dt
                    else:
                        rise_b = dt

                # 过零：换向段（5=高速正转→反转，8=高速反转→正转）的 |omega| 最小值
                if idx % len(TRACK) in (5, 8):
                    cross_v = float(np.min(np.abs(om)))
                    if p == p_a:
                        cross_a.append(cross_v)
                    else:
                        cross_b.append(cross_v)

        def row(name, a, b, unit="", fmt="{:+.3f}"):
            if a is None or b is None or (a != a) or (b != b):
                print(f"{name:>12} {'未测到':>12} {'未测到':>12} {'-':>10}")
                return

            delta = b - a
            print(f"{name:>12} {fmt.format(a) + unit:>12} {fmt.format(b) + unit:>12} {delta:+.3f}{unit:>4}")

        row("稳态误差", float(np.mean(err_a)) if err_a else None, float(np.mean(err_b)) if err_b else None, " rad/s")
        row("PID输出", float(np.mean(int_a)) if int_a else None, float(np.mean(int_b)) if int_b else None, " A")
        row("低速上升", rise_a, rise_b, " s")
        row("换向波谷", float(np.mean(cross_a)) if cross_a else None,
            float(np.mean(cross_b)) if cross_b else None, " rad/s")

        if err_a and err_b:
            ea, eb = float(np.mean(err_a)), float(np.mean(err_b))
            if abs(ea) > 1e-9:
                print(f"  {'':>10}稳态误差降低 {(abs(ea) - abs(eb)) / abs(ea) * 100.0:.1f}%")


def main() -> None:
    parser = argparse.ArgumentParser(description="C620 摩擦前馈 A/B 对比")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--wheel", type=int, default=None, choices=[0, 1, 2, 3], help="只看某一轮")
    parser.add_argument("--save", default="chassis_ff_ab.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file, args.wheel)
        return

    capture(args)
    analyze(args.save, args.wheel)


if __name__ == "__main__":
    main()
