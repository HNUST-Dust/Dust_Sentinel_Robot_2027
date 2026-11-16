"""C620 减速箱等效惯量辨识：开环电流阶跃 → 角加速度回归。

原理：J·alpha = Kt·(I - I_f(omega))，所以
    alpha = (I - I_f(omega)) / J_eq
对净驱动电流与角加速度做回归，斜率取倒数为 J_eq。

激励基准用已辨识的静摩擦起转电流（scripts/chassis_model_ident.py 的输出），
让每个轮子都能真正转起来；I_f 用动摩擦模型（chassis_friction.py 模型二）。

分两种做法：
  粗档：净驱动 0.15 / 0.3 A，验证轮子都能转、加速度方向正确
  细档：净驱动 0.1 / 0.2 / 0.3 A，每段分段平均后回归

python scripts/chassis_inertia_ident.py --port COM8
python scripts/chassis_inertia_ident.py --file temp/chassis_inertia.csv

单位：CSV 时间 ms，转速 mrad/s，电流 mA。
"""
import argparse
import csv
import re
import time

WHEELS = 4
FIELDS = (["t_ms", "seg"]
          + [f"omega{w}" for w in range(WHEELS)]
          + [f"cur{w}" for w in range(WHEELS)]
          + [f"acc{w}" for w in range(WHEELS)]
          + ["dt_us"])

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+)," * 12 + r"(-?\d+)\s*$")
BAUD = 921600

# 动摩擦模型（chassis_friction.py 模型二，A）
FRIC_A = [0.33983, 0.27259, 0.53885, 0.49975]
FRIC_B = [0.01966, 0.02159, 0.04012, 0.02587]
FRIC_A_NEG = [0.31649, 0.28193, 0.47052, 0.50784]
FRIC_B_NEG = [0.02408, 0.02293, 0.04561, 0.02563]

# 静摩擦起转电流（chassis_model_ident.py 输出）
BREAK_POS = [0.65, 0.63, 0.84, 0.74]
BREAK_NEG = [0.44, 0.53, 1.15, 0.68]


def clean(raw: bytes) -> str:
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


def friction_current(wheel: int, omega: float) -> float:
    if omega >= 0.0:
        return FRIC_A[wheel] + FRIC_B[wheel] * omega
    return -(FRIC_A_NEG[wheel] + FRIC_B_NEG[wheel] * abs(omega))


def capture(args) -> None:
    import serial

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        def send(command: str) -> None:
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        # 先停掉残留调试流，再清缓冲
        ser.write(b"log off\r\n")
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()

        send("log off")
        time.sleep(0.2)

        # test_omega=0 → 开环恒电流模式，用 chassis_test_i0~3
        send("var set chassis_test_omega 0")
        for i in range(WHEELS):
            send(f"var set chassis_test_i{i} 0")
        send("var set chassis_test_enable 1")
        time.sleep(0.2)
        send("log on chassis_ident 1")
        time.sleep(0.3)
        send("log mode chassis_ident on")
        time.sleep(0.3)
        ser.reset_input_buffer()

        start = beat = time.monotonic()
        count = 0
        seg = 0

        def apply(net: float) -> None:
            for i in range(WHEELS):
                if net == 0.0:
                    send(f"var set chassis_test_i{i} 0")
                    continue

                base = BREAK_POS[i] if net > 0 else BREAK_NEG[i]
                cmd = (abs(net) + base) * (1.0 if net > 0 else -1.0)
                send(f"var set chassis_test_i{i} {cmd:.3f}")

        def hold(seconds: float, seg_id: int) -> None:
            nonlocal count, beat

            until = time.monotonic() + seconds
            while time.monotonic() < until:
                match = DATA.search(clean(ser.readline()))
                if not match:
                    continue

                values = [int(x) for x in match.groups()]
                t_ms = int((time.monotonic() - start) * 1000)
                writer.writerow([t_ms, seg_id] + values)
                count += 1

                now = time.monotonic()
                if now - beat >= 2:
                    output.flush()
                    wheels = " ".join(f"w{i}={values[i] / 1000.0:+.2f}" for i in range(WHEELS))
                    print(f"[hb] {now - start:.0f}s 段{seg_id} {wheels} 样本={count}", flush=True)
                    beat = now

        try:
            steps = list(args.levels) + list(-v for v in args.levels)

            for cycle in range(args.cycles):
                for net in steps:
                    apply(net)
                    print(f"[i] 轮次{cycle + 1} 净驱动 {net:+.2f} A", flush=True)
                    hold(args.hold, seg)
                    seg += 1

                    apply(0.0)
                    hold(args.settle, -1)   # 回零段标 -1，分析时剔除
        finally:
            apply(0.0)
            send("var set chassis_test_enable 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)

        if count == 0:
            raise RuntimeError("没收到 chassis_ident 数据")


def analyze(path: str, acc_min: float = 1.0) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    print(f"[i] 样本 {len(rows)}")
    print()
    print(f"{'轮':>3} {'段数':>5} {'J_eq A/(rad/s^2)':>18} {'R2':>8} {'RMSE(rad/s^2)':>14}")

    for wheel in range(WHEELS):
        omega = np.array([float(r[f"omega{wheel}"]) / 1000.0 for r in rows])
        cur = np.array([float(r[f"cur{wheel}"]) / 1000.0 for r in rows])
        acc = np.array([float(r[f"acc{wheel}"]) / 1000.0 for r in rows])
        seg = np.array([int(r["seg"]) for r in rows])

        xs, ys = [], []

        for s in sorted({v for v in seg if v >= 0}):
            idx = np.flatnonzero(seg == s)
            if len(idx) < 5:
                continue

            # 跳过段首 2 拍，避开电流建立过程
            idx = idx[2:]

            net = np.mean([cur[i] - friction_current(wheel, omega[i]) for i in idx])
            w_mean = np.mean(omega[idx])
            a_mean = np.mean(acc[idx])

            # 只取转速在摩擦辨识范围内、驱动方向一致的段
            if abs(w_mean) < 1.0 or abs(w_mean) > 20.0:
                continue
            if net * w_mean <= 0 or abs(net) < 0.03:
                continue
            if abs(a_mean) < acc_min:
                continue

            xs.append(net)
            ys.append(a_mean)

        if len(xs) < 6:
            print(f"{wheel:3d} {len(xs):5d}  有效段不足，跳过")
            continue

        x = np.array(xs)
        y = np.array(ys)

        slope = float((x * y).sum() / (x * x).sum())
        if slope <= 0:
            print(f"{wheel:3d} {len(x):5d}  斜率非正，数据方向异常")
            continue

        j_eq = 1.0 / slope
        pred = slope * x
        ss_res = float(((y - pred) ** 2).sum())
        ss_tot = float(((y - y.mean()) ** 2).sum())
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        rmse = float(np.sqrt(((y - pred) ** 2).mean()))

        print(f"{wheel:3d} {len(x):5d} {j_eq:18.5f} {r2:8.4f} {rmse:14.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="C620 减速箱等效惯量辨识")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--levels", type=float, nargs="+", default=[0.1, 0.2, 0.3],
                        help="净驱动电流 A（在起转电流之上的增量）")
    parser.add_argument("--cycles", type=int, default=5, help="阶跃序列重复轮数")
    parser.add_argument("--hold", type=float, default=0.8, help="每档保持时间 s")
    parser.add_argument("--settle", type=float, default=0.8, help="档间回零时间 s")
    parser.add_argument("--acc-min", type=float, default=1.0, help="参与拟合的最小角加速度 rad/s^2")
    parser.add_argument("--save", default="temp/chassis_inertia.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file, args.acc_min)
        return

    capture(args)
    analyze(args.save, args.acc_min)


if __name__ == "__main__":
    main()
