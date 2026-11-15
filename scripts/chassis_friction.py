"""C620 减速箱摩擦辨识：速度环锁转速，读稳态维持电流。

做法：底盘速度环保持原样，用 var 把四轮目标转速锁成同一个值
（chassis_test_enable=1 + chassis_test_omega）。静摩擦大的轮子需要更大电流
才能维持同一转速，稳态电流直接反映摩擦大小——纯 P 环的静差由积分/比例
自动补足，不影响辨识。

每个转速档保持若干秒，取末段稳态的中位数作为该转速下的维持电流。
正反转分开测：正转和反转的摩擦一般不同。

四个减速箱精度不同，四轮静摩擦不同，同一目标转速下四轮电流会不同——
这就是要测的结果，不是异常。任一轮失控只报告，不中断。

python scripts/chassis_friction.py --port COM8 --direction forward
python scripts/chassis_friction.py --file chassis_friction.csv

单位：CSV 时间 ms，电流 mA，角速度 mrad/s。
"""
import argparse
import csv
import re
import time

WHEELS = 4
FIELDS = ["t_ms", "cmd_omega", "tgt_i0", "tgt_i1", "tgt_i2", "tgt_i3",
          "omega0", "omega1", "omega2", "omega3",
          "fb_i0", "fb_i1", "fb_i2", "fb_i3"]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
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
        configure("var set chassis_test_omega 0")
        # 先挂上空载速度环（目标 0），让 chassis_friction 条目被创建出来
        configure("var set chassis_test_enable 1")
        configure("log mode chassis_friction on")
        configure("log on chassis_friction 1")

        start = beat = time.monotonic()
        count = 0
        warned = set()

        # 每个方向各自一组档位；--direction both 时正反都跑
        directions = [1.0, -1.0] if args.direction == "both" else \
                     ([1.0] if args.direction == "forward" else [-1.0])

        def dwell(seconds: float, note: str) -> None:
            # 逐秒打印，不许静默等待
            for remaining in range(int(seconds), 0, -1):
                print(f"[i] {note} {remaining}s", flush=True)
                time.sleep(1.0)

        try:
            for sign in directions:
                steps = []
                level = args.min_omega
                while level <= args.max_omega + 1e-6:
                    steps.append(round(sign * level, 3))
                    level += args.step

                name = "正转" if sign > 0 else "反转"
                print(f"[i] === {name}：转速档 {steps} rad/s，每档 {args.hold}s，档间回零 {args.settle}s ===", flush=True)

                for omega in steps:
                    send(f"var set chassis_test_omega {omega}")
                    print(f"[i] {name} 目标转速 {omega:+.3f} rad/s", flush=True)

                    hold_until = time.monotonic() + args.hold
                    while time.monotonic() < hold_until:
                        line = clean(ser.readline())
                        match = DATA.search(line)
                        if not match:
                            continue

                        values = [int(x) for x in match.groups()]
                        omegas = [values[4 + i] / 1000.0 for i in range(WHEELS)]
                        currents = [values[8 + i] / 1000.0 for i in range(WHEELS)]

                        t_ms = int((time.monotonic() - start) * 1000)
                        writer.writerow([t_ms, int(omega * 1000)] + values)
                        count += 1

                        now = time.monotonic()
                        if now - beat >= 2:
                            output.flush()
                            wheels = " ".join(f"w{i}={omegas[i]:+.2f}/{currents[i]:+.2f}A" for i in range(WHEELS))
                            print(f"[hb] {now - start:.0f}s 目标={omega:+.2f}rad/s {wheels} 样本={count}", flush=True)
                            beat = now

                        # 任一轮失控只报告，其余轮继续采集
                        for i in range(WHEELS):
                            if abs(omegas[i]) >= args.omega_limit and i not in warned:
                                warned.add(i)
                                print(f"[!] 轮 {i} 角速度 {omegas[i]:+.3f} rad/s 超过 {args.omega_limit}", flush=True)

                    # 档间回零：清掉上一档的转速，避免带进下一档
                    send("var set chassis_test_omega 0")
                    dwell(args.settle, f"{name} {omega:+.1f} rad/s 后回零")
        finally:
            send("var set chassis_test_omega 0")
            send("var set chassis_test_enable 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)


def r2(actual, pred) -> float:
    import numpy as np

    ss_res = float(((actual - pred) ** 2).sum())
    ss_tot = float(((actual - actual.mean()) ** 2).sum())
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0


def rmse(actual, pred) -> float:
    import numpy as np

    return float(np.sqrt(((actual - pred) ** 2).mean()))


def analyze(path: str, omega_th: float = 0.1, settle_ratio: float = 0.7) -> None:
    """按实际转速分档：P 环静差只改变实际转速，不改变同一转速下电流与摩擦的对应关系。

    每档（目标转速保持期间）取末段稳态，用该段实际转速中位数当横坐标，
    反馈电流中位数当纵坐标。正反方向分开输出，因为摩擦正反不对称。
    """
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    t = np.array([float(r["t_ms"]) / 1000.0 for r in rows])
    tgt = np.array([float(r["cmd_omega"]) / 1000.0 for r in rows])
    print(f"[i] 样本 {len(rows)}，时长 {t[-1] - t[0]:.1f}s")

    levels = sorted(set(np.round(tgt, 3)))

    for wheel in range(WHEELS):
        omega = np.array([float(r[f"omega{wheel}"]) / 1000.0 for r in rows])
        cur = np.array([float(r[f"fb_i{wheel}"]) / 1000.0 for r in rows])

        print()
        print(f"=== 轮 {wheel} ===")
        print(f"{'方向':>4} {'目标rad/s':>10} {'实际rad/s':>10} {'维持电流A':>10} {'电流std':>8} {'n':>5}")

        # 静摩擦：目标档存在但实际转速≈0，读该档末段电流
        still = []
        steady_w = []
        steady_i = []

        for level in levels:
            idx = np.flatnonzero(np.isclose(np.round(tgt, 3), level))
            tail = idx[int(settle_ratio * len(idx)):] if len(idx) > 3 else idx
            w = omega[tail]
            c = cur[tail]

            w_med = float(np.median(w))
            i_med = float(np.median(c))
            direction = "正" if level > 0 else "反"

            print(f"{direction:>4} {level:10.3f} {w_med:10.3f} {i_med:10.4f} {c.std():8.4f} {len(tail):5d}")

            if abs(w_med) < omega_th:
                still.append((direction, level, i_med))
            else:
                steady_w.append(w_med)
                steady_i.append(i_med)

        if still:
            print("[静摩擦] 未起转档的维持电流（可能含 PID 积分累积，仅供参考）")
            for direction, level, i_med in still:
                print(f"  {direction}向 目标 {level:+6.1f} rad/s  电流 {i_med:+.4f} A")

        if len(steady_w) < 3:
            print("[拟合] 稳态档不足 3 个，跳过")
            continue

        w = np.array(steady_w)
        i = np.array(steady_i)

        # 模型一：正反合并，I = a*sgn(omega) + b*omega（用原符号，sgn 隐含在 omega 的方向里）
        A1 = np.column_stack([np.sign(w), w])
        coef1, *_ = np.linalg.lstsq(A1, i, rcond=None)
        pred1 = A1 @ coef1
        print()
        print("[模型一] 正反合并  I = a*sgn(omega) + b*omega")
        print(f"  a = {coef1[0]:+.5f} A    b = {coef1[1]:+.5f} A/(rad/s)")
        print(f"  R^2 = {r2(i, pred1):.5f}   RMSE = {rmse(i, pred1):.5f} A")

        # 模型二：正反分开，I = a_+ + b_+omega (omega>0)，I = -a_- + b_-omega (omega<0)
        pos = w > 0
        neg = w < 0
        print()
        print("[模型二] 正反分开")
        coef_pos = coef_neg = None

        if pos.sum() >= 2:
            A = np.column_stack([np.ones(pos.sum()), w[pos]])
            coef_pos, *_ = np.linalg.lstsq(A, i[pos], rcond=None)
            print(f"  正转(omega>0)  a_+ = {coef_pos[0]:+.5f} A   b_+ = {coef_pos[1]:+.5f} A/(rad/s)   n={pos.sum()}")

        if neg.sum() >= 2:
            A = np.column_stack([np.ones(neg.sum()), w[neg]])
            coef_neg, *_ = np.linalg.lstsq(A, i[neg], rcond=None)
            print(f"  反转(omega<0)  a_- = {-coef_neg[0]:+.5f} A  b_- = {coef_neg[1]:+.5f} A/(rad/s)  n={neg.sum()}")
            if coef_pos is not None:
                print(f"  不对称：库仑差 {coef_pos[0] - (-coef_neg[0]):+.5f} A，"
                      f"黏性差 {coef_pos[1] - coef_neg[1]:+.5f} A/(rad/s)")

        # 残差：判断是否需要 Stribeck
        print()
        print("[残差] 模型一逐点")
        print(f"  {'omega':>9} {'I实测':>9} {'I拟合':>9} {'残差':>9} {'相对':>8}")
        for wi, ii, pi in zip(w, i, pred1):
            rel = (ii - pi) / abs(pi) * 100.0 if abs(pi) > 1e-9 else 0.0
            print(f"  {wi:9.3f} {ii:9.4f} {pi:9.4f} {ii - pi:+9.4f} {rel:+7.1f}%")

        resid = i - pred1
        low = np.abs(w) < 2.0
        if low.sum() >= 2:
            print(f"  低速段(|omega|<2) 残差均值 {resid[low].mean():+.5f} A，"
                  f"高速段 {resid[~low].mean():+.5f} A")
            if abs(resid[low].mean()) > 2 * abs(resid[~low].mean()):
                print("  -> 低速残差明显偏离，考虑 Stribeck 模型")

    summary(rows, tgt, omega_th, settle_ratio)


def summary(rows, tgt, omega_th: float, settle_ratio: float) -> None:
    """四轮汇总：模型二的正反库仑/黏性系数并排列出，便于横向比较。"""
    import numpy as np

    print()
    print("=" * 88)
    print("[汇总] 模型二  正转 I = a_+ + b_+*omega   反转 I = -a_- + b_-*omega")
    print(f"{'轮':>3} {'a_+ A':>9} {'b_+ A/(rad/s)':>14} {'a_- A':>9} {'b_- A/(rad/s)':>14} "
          f"{'库仑差%':>9} {'黏性比':>8} {'R2':>7}")

    results = []

    for wheel in range(WHEELS):
        omega = np.array([float(r[f"omega{wheel}"]) / 1000.0 for r in rows])
        cur = np.array([float(r[f"fb_i{wheel}"]) / 1000.0 for r in rows])

        levels = sorted(set(np.round(tgt, 3)))
        steady_w = []
        steady_i = []

        for level in levels:
            idx = np.flatnonzero(np.isclose(np.round(tgt, 3), level))
            tail = idx[int(settle_ratio * len(idx)):] if len(idx) > 3 else idx
            w_med = float(np.median(omega[tail]))
            if abs(w_med) < omega_th:
                continue
            steady_w.append(w_med)
            steady_i.append(float(np.median(cur[tail])))

        if len(steady_w) < 4:
            print(f"{wheel:3d}  稳态档不足，跳过")
            continue

        w = np.array(steady_w)
        i = np.array(steady_i)
        pos = w > 0
        neg = w < 0

        a_pos = b_pos = a_neg = b_neg = None

        if pos.sum() >= 2:
            A = np.column_stack([np.ones(pos.sum()), w[pos]])
            cp, *_ = np.linalg.lstsq(A, i[pos], rcond=None)
            a_pos, b_pos = float(cp[0]), float(cp[1])

        if neg.sum() >= 2:
            A = np.column_stack([np.ones(neg.sum()), w[neg]])
            cn, *_ = np.linalg.lstsq(A, i[neg], rcond=None)
            a_neg, b_neg = float(-cn[0]), float(cn[1])

        pred = np.where(pos, (a_pos + b_pos * w) if a_pos is not None else np.nan,
                        (-a_neg + b_neg * w) if a_neg is not None else np.nan)
        mask = ~np.isnan(pred)

        asym = (a_neg - a_pos) / a_pos * 100.0 if (a_pos and a_neg) else None
        visc_ratio = b_neg / b_pos if (b_pos and b_neg) else None

        results.append({ "wheel": wheel, "a_pos": a_pos, "a_neg": a_neg, "b_pos": b_pos, "b_neg": b_neg })

        def fmt(v, spec):
            return format(v, spec) if v is not None else "      -"

        print(f"{wheel:3d} {fmt(a_pos, '9.5f')} {fmt(b_pos, '14.5f')} {fmt(a_neg, '9.5f')} {fmt(b_neg, '14.5f')} "
              f"{fmt(asym, '8.1f')}{'%' if asym is not None else ' '} {fmt(visc_ratio, '8.3f')} "
              f"{r2(i[mask], pred[mask]):7.4f}")

    if results:
        # 只比较能算出的量：正转库仑、反转库仑分别看
        pos_ok = [r for r in results if r["a_pos"] is not None]
        neg_ok = [r for r in results if r["a_neg"] is not None]

        print()

        if len(pos_ok) >= 2:
            worst = max(pos_ok, key=lambda r: r["a_pos"])
            best = min(pos_ok, key=lambda r: r["a_pos"])
            print(f"[结论-正转] 库仑摩擦最大 轮{worst['wheel']} {worst['a_pos']:.4f} A，"
                  f"最小 轮{best['wheel']} {best['a_pos']:.4f} A，相差 {worst['a_pos'] / best['a_pos']:.2f} 倍")

        if len(neg_ok) >= 2:
            worst = max(neg_ok, key=lambda r: r["a_neg"])
            best = min(neg_ok, key=lambda r: r["a_neg"])
            print(f"[结论-反转] 库仑摩擦最大 轮{worst['wheel']} {worst['a_neg']:.4f} A，"
                  f"最小 轮{best['wheel']} {best['a_neg']:.4f} A，相差 {worst['a_neg'] / best['a_neg']:.2f} 倍")

        if len(pos_ok) < WHEELS or len(neg_ok) < WHEELS:
            print(f"[!] 数据不完整：正转 {len(pos_ok)} 轮、反转 {len(neg_ok)} 轮有解（应为 {WHEELS}）")

        print("  注意：电流为 C620 反馈原始值，未乘 torque_k 换算成 N*m")


def main() -> None:
    parser = argparse.ArgumentParser(description="C620 减速箱摩擦力辨识（速度环锁转速）")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--direction", default="both", choices=["forward", "reverse", "both"], help="正转/反转/两者都跑")
    parser.add_argument("--min-omega", type=float, default=1.0, help="起始转速 rad/s")
    parser.add_argument("--max-omega", type=float, default=20.0, help="最大转速 rad/s")
    parser.add_argument("--step", type=float, default=1.0, help="每档转速增量 rad/s")
    parser.add_argument("--hold", type=float, default=6.0, help="每档保持时间 s，需覆盖加速到稳态")
    parser.add_argument("--settle", type=float, default=2.0, help="档间回零停稳时间 s")
    parser.add_argument("--omega-limit", type=float, default=40.0, help="角速度报警阈值 rad/s")
    parser.add_argument("--save", default="chassis_friction.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
