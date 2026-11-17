"""小 yaw 摩擦前馈 A/B：同一组角度阶跃，分别关/开摩擦补偿，对比响应。

前提：
  - 固件已编译烧录，遥控开机且整车使能（否则 yaw 线程一直 Retry，没有数据）
  - 遥控 SB 上档（custom1=On），否则 var 目标角不生效
  - 测试模式关闭（yaw_test_enable=0）

激励：0 → +A → 0 → -A → 0，每段保持 hold 秒，先关补偿跑一遍，再开补偿跑一遍。
采样：固件 yaw_trace_enable=1 时按 RAW 直发 YSTEP 行。
       YSTEP, t_ms, tgt_mdeg, world_mdeg, joint_mdeg, torque_mnm, fric_flag

python scripts/yaw_fric_ab.py --port COM8
python scripts/yaw_fric_ab.py --file temp/yaw_fric_ab.csv
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "tgt_mdeg", "world_mdeg", "joint_mdeg", "torque_mnm", "fric", "phase"]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"YSTEP,(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
BAUD = 921600


def clean(raw: bytes) -> str:
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


def capture(args, serial_factory=None, clock=None) -> None:
    clock = clock or time.monotonic
    if serial_factory is None:
        import serial

        serial_factory = serial.Serial

    with serial_factory(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        last_beat = [clock()]
        count = [0]

        def send(command: str) -> None:
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        def configure(command: str) -> None:
            send(command)
            deadline = clock() + 3.0
            while clock() < deadline:
                line = clean(ser.readline())
                if "not found" in line:
                    raise RuntimeError(f"{command} 失败: {line}")
                if line.startswith(("ok", "log off: ok")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        def set_target(deg: float) -> None:
            send(f"var set yaw_small_target {deg:.3f}")

        def hold(seconds: float, phase: str) -> None:
            deadline = clock() + seconds
            while clock() < deadline:
                line = clean(ser.readline())
                match = DATA.search(line)
                if not match:
                    continue
                row = tuple(int(v) for v in match.groups())
                writer.writerow((*row, phase))
                count[0] += 1
                now = clock()
                if now - last_beat[0] >= args.beat:
                    last_beat[0] = now
                    print(f"[hb] {phase} 样本 {count[0]} tgt={row[1] / 1000.0:+6.2f}° "
                          f"world={row[2] / 1000.0:+7.2f}° torque={row[4] / 1000.0:+6.3f} N*m",
                          flush=True)

        try:
            configure("log off")
            send("var set yaw_test_enable 0")
            send("var set yaw_test_torque 0")
            configure("var set yaw_trace_enable 1")

            for phase, enable in (("off", 0), ("on", 1)):
                configure(f"var set yaw_fric_enable {enable}")
                print(f"[i] === 摩擦补偿 {phase} ===", flush=True)
                set_target(0.0)
                hold(args.hold, phase)
                for target in (args.amplitude, 0.0, -args.amplitude, 0.0):
                    print(f"[i] {phase} 目标 {target:+.1f}°", flush=True)
                    set_target(target)
                    hold(args.hold, phase)
            print(f"[i] 采集完成，{count[0]} 样本 -> {args.save}", flush=True)
        finally:
            send("var set yaw_small_target 0")
            send("var set yaw_trace_enable 0")
            send("var set yaw_fric_enable 1")
            send("log off")


def _events(rows, phase):
    """按目标角变化切出阶跃事件。"""
    events = []
    current = None
    for row in rows:
        if row["phase"] != phase:
            continue
        if current is None:
            current = {"target": row["tgt_mdeg"] / 1000.0, "rows": []}
        elif abs(row["tgt_mdeg"] / 1000.0 - current["target"]) > 0.5:
            events.append(current)
            current = {"target": row["tgt_mdeg"] / 1000.0, "rows": []}
        current["rows"].append(row)
    if current:
        events.append(current)
    return events


def _metrics(event):
    """一次阶跃的上升时间、超调、稳态误差。"""
    rows = event["rows"]
    if len(rows) < 10:
        return None

    target = rows[-1]["tgt_mdeg"] / 1000.0
    start = rows[0]["world_mdeg"] / 1000.0
    direction = 1.0 if target > start else -1.0
    initial = abs(target - start)
    if initial < 1.0:
        return None

    # 世界角跟随方向的外包络：上升时间按 10%/90% 判
    t0 = rows[0]["t_ms"]
    reach_10 = reach_90 = None
    overshoot = 0.0
    for row in rows:
        world = row["world_mdeg"] / 1000.0
        progress = direction * (world - start) / initial
        if reach_10 is None and progress >= 0.1:
            reach_10 = (row["t_ms"] - t0) / 1000.0
        if reach_90 is None and progress >= 0.9:
            reach_90 = (row["t_ms"] - t0) / 1000.0
        overshoot = max(overshoot, direction * (world - target))

    tail = rows[int(0.7 * len(rows)):]
    steady = sum(abs(r["world_mdeg"] / 1000.0 - target) for r in tail) / len(tail)
    torque_tail = sum(abs(r["torque_mnm"]) for r in tail) / len(tail) / 1000.0

    reached = reach_90 is not None
    return {"reached": reached,
            "rise": (reach_90 - reach_10) if reached and reach_10 is not None else None,
            "overshoot": overshoot if reached else None,
            "steady": steady, "torque": torque_tail}


def analyze(path: str):
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")
    for row in rows:
        for key in ("t_ms", "tgt_mdeg", "world_mdeg", "joint_mdeg", "torque_mnm", "fric"):
            row[key] = int(row[key])

    result = {}
    for phase in ("off", "on"):
        metrics = [m for m in (_metrics(e) for e in _events(rows, phase)) if m]
        if not metrics:
            print(f"[w] {phase} 相位没有可用阶跃")
            continue
        reached = [m for m in metrics if m["reached"]]
        result[phase] = {
            "reached_rate": len(reached) / len(metrics),
            "rise": sum(m["rise"] for m in reached) / len(reached) if reached else None,
            "overshoot": sum(m["overshoot"] for m in reached) / len(reached) if reached else None,
            "steady": sum(m["steady"] for m in metrics) / len(metrics),
            "torque": sum(m["torque"] for m in metrics) / len(metrics),
            "n": len(metrics),
            "reached": len(reached),
        }

    if len(result) < 2:
        print("[w] 只有一种补偿状态，无法对比")
        return

    off, on = result["off"], result["on"]
    print(f"{'指标':<14}{'关补偿':>10}{'开补偿':>10}{'改善':>10}")
    print(f"{'到位率':<12}{off['reached_rate'] * 100:>9.0f}%{on['reached_rate'] * 100:>9.0f}%")
    for key, label, unit in (("rise", "90-10%上升", "s"), ("overshoot", "超调", "°"),
                             ("steady", "稳态误差", "°"), ("torque", "稳态力矩", "N*m")):
        a, b = off[key], on[key]
        if a is None or b is None:
            print(f"{label:<12}{'—' if a is None else f'{a:.3f}':>10}{'—' if b is None else f'{b:.3f}':>10}      —  {unit}")
            continue
        # 稳态力矩只报变化量：停住时顶住摩擦的力矩大是正常的，不是指标
        if key == "torque":
            print(f"{label:<12}{a:>10.3f}{b:>10.3f}{(b - a) / a * 100.0:>+9.0f}%  {unit}（变化）")
            continue
        change = (a - b) / a * 100.0 if a else 0.0
        print(f"{label:<12}{a:>10.3f}{b:>10.3f}{change:>9.0f}%  {unit}")
    print(f"阶跃数：关 {off['n']}（到位 {off['reached']}），开 {on['n']}（到位 {on['reached']}）")
    return result


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="小 yaw 摩擦前馈 A/B")
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--save", default="temp/yaw_fric_ab.csv")
    p.add_argument("--file", help="只分析已有 CSV")
    p.add_argument("--amplitude", type=float, default=6.0, help="阶跃幅值（度）")
    p.add_argument("--hold", type=float, default=1.5, help="每段保持 s")
    p.add_argument("--beat", type=float, default=2.0, help="心跳间隔 s")
    return p


def main() -> None:
    args = parser().parse_args()
    if args.file:
        analyze(args.file)
        return
    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
