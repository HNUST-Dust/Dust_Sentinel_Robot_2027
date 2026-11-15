"""大 yaw 摩擦力辨识：恒力矩斜坡，找静摩擦起转阈值与动摩擦稳态速度。

做法：yaw 线程切到开环（yaw_test_enable=1），逐档下发恒力矩，每档保持若干秒，
记录角度/角速度/下发力矩。静止段力矩逐渐加大直到起转 = 静摩擦；起转后速度
随时间线性上涨的斜率 = 1/惯量，截距过零点 = 动摩擦。

python scripts/yaw_friction.py --port COM8
python scripts/yaw_friction.py --file yaw_friction.csv

单位：CSV 时间 ms，角度 mrad，角速度 mrad/s，力矩 mN*m。
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "target_rad_mrad", "target_omega_mrads", "target_torque_mnm",
          "tx4", "tx5", "tx6", "tx7", "fb_mnm", "omega_mrads", "err", "tmos"]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
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
                if "not found" in line:
                    raise RuntimeError(f"{command} 失败: {line}")
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        ser.reset_input_buffer()
        configure("log off")
        configure("var set yaw_test_torque 0")
        # 先开环（力矩 0，竖直轴无重力不会动），让 yaw_friction 条目被创建出来
        configure("var set yaw_test_enable 1")
        configure("log mode yaw_friction on")
        configure("log on yaw_friction 1")

        start = beat = time.monotonic()
        count = 0
        steps = []

        torque = 0.0
        while torque <= args.max_torque + 1e-6:
            steps.append(round(torque, 3))
            torque += args.step

        print(f"[i] 力矩档位 {steps} N*m，每档 {args.hold}s", flush=True)

        try:
            for torque in steps:
                send(f"var set yaw_test_torque {torque}")
                print(f"[i] 力矩 {torque:.3f} N*m", flush=True)

                hold_until = time.monotonic() + args.hold
                while time.monotonic() < hold_until:
                    line = clean(ser.readline())
                    match = DATA.search(line)
                    if not match:
                        continue

                    values = [int(x) for x in match.groups()]

                    fb_mnm = values[7]
                    omega_rad_s = values[8] / 1000.0
                    err = values[9]

                    t_ms = int((time.monotonic() - start) * 1000)
                    writer.writerow([t_ms] + values)
                    count += 1

                    now = time.monotonic()
                    if now - beat >= 2:
                        output.flush()
                        print(f"[hb] {now - start:.0f}s τtgt={values[2] / 1000.0:+.3f} fb={fb_mnm / 1000.0:+.3f} "
                              f"ω={omega_rad_s:+.3f} tx={values[3]:02x},{values[4]:02x},{values[5]:02x},{values[6]:02x} "
                              f"err={err} 样本={count}", flush=True)
                        beat = now

                    if abs(omega_rad_s) >= args.omega_limit:
                        print(f"[!] 角速度 {omega_rad_s:+.3f} rad/s 超过保护阈值，停止", flush=True)
                        return
        finally:
            send("var set yaw_test_torque 0")
            send("var set yaw_test_enable 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)


def analyze(path: str, omega_th: float = 0.05, accel_th: float = 0.5) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    t = np.array([float(r["t_ms"]) / 1000.0 for r in rows])
    torque = np.array([float(r["target_torque_mnm"]) / 1000.0 for r in rows])
    fb = np.array([float(r["fb_mnm"]) / 1000.0 for r in rows])
    omega = np.array([float(r["omega_mrads"]) / 1000.0 for r in rows])
    err = np.array([int(r["err"]) for r in rows])
    tx7 = np.array([int(r["tx7"]) for r in rows])

    levels = sorted(set(np.round(torque, 3)))
    print(f"[i] 样本 {len(rows)}，力矩档 {len(levels)} 个，时长 {t[-1] - t[0]:.1f}s")
    print()
    print(f"{'tgt_Nm':>8} {'fb_med':>8} {'fb_std':>8} {'omega_med':>10} {'omega_std':>10} "
          f"{'domega_dt':>10} {'tx7':>5} {'err':>5} {'n':>5}")

    levels_info = []
    for level in levels:
        idx = np.flatnonzero(np.isclose(torque, level))
        tail = idx[int(0.7 * len(idx)):] if len(idx) > 3 else idx
        w = omega[tail]
        f = fb[tail]
        # 该档内角速度变化率：末段线性拟合斜率
        dw = 0.0
        if len(tail) > 3 and np.ptp(t[tail]) > 1e-6:
            dw = float(np.polyfit(t[tail], w, 1)[0])
        levels_info.append((level, float(np.median(w)), dw))
        print(f"{level:8.3f} {np.median(f):8.4f} {f.std():8.4f} {np.median(w):10.4f} {w.std():10.4f} "
              f"{dw:10.3f} {int(np.median(tx7[tail])):5d} {int(np.median(err[tail])):5d} {len(tail):5d}")

    print()

    still = [x for x in levels_info if abs(x[1]) < omega_th]
    moving = [x for x in levels_info if abs(x[1]) >= omega_th]

    if still:
        print(f"[未起转] 最大静止档 = {still[-1][0]:.3f} N*m（|ω| < {omega_th} rad/s）")
        print("         这是电机尚未起转的最大力矩，不是静摩擦值本身")
    else:
        print("[未起转] 无（第一档就转了）")

    if not moving:
        print("[已起转] 无（直到最大力矩都没转）")
        return

    print(f"[已起转] 首个转动档 = {moving[0][0]:.3f} N*m")

    accel = [x for x in moving if abs(x[2]) >= accel_th]
    flat = [x for x in moving if abs(x[2]) < accel_th]

    if accel:
        print("\n[加速段] τ = 动摩擦 + J·dω/dt，只凭 τ 和 ω 无法分离摩擦与惯量")
        for lvl, w, dw in accel:
            print(f"  τ={lvl:6.3f} N*m  ω={w:7.3f} rad/s  dω/dt={dw:8.3f} rad/s^2")
    else:
        print("\n[加速段] 无")

    if flat:
        print("\n[匀速段] dω/dt≈0，此处 τ 即动摩擦+恒定阻力")
        for lvl, w, dw in flat:
            print(f"  τ={lvl:6.3f} N*m  ω={w:7.3f} rad/s  dω/dt={dw:8.3f} rad/s^2")
        print(f"\n[动摩擦] = 匀速段力矩中位数 {np.median([x[0] for x in flat]):.4f} N*m（共 {len(flat)} 档）")
    else:
        print("\n[匀速段] 无——本次没有采到稳态，无法直接读出动摩擦")
        print("        需要更长保持时间，或让电机撞到限位/负载使其匀速")


def main() -> None:
    parser = argparse.ArgumentParser(description="大 yaw 摩擦力辨识（恒力矩斜坡）")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--max-torque", type=float, default=0.8, help="最大下发力矩 N*m")
    parser.add_argument("--step", type=float, default=0.05, help="每档力矩增量 N*m")
    parser.add_argument("--hold", type=float, default=1.0, help="每档保持时间 s")
    parser.add_argument("--omega-limit", type=float, default=5.0, help="角速度保护阈值 rad/s")
    parser.add_argument("--save", default="yaw_friction.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
