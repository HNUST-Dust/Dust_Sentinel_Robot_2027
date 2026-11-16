"""C620 减速箱静摩擦辨识：逐档加电流，找每个轮子的起转电流。

四个减速箱精度不同，静摩擦也不同，所以四轮各自独立判定起转；
正反转摩擦通常不对称，两个方向分开测。

做法：四轮同时从 sweep_min 开始逐档加电流，每档保持 sweep_hold 秒。
某轮转速超过起转阈值就记下当前档位作为它的起转电流，并把该轮电流
固定在这个值不再加（避免先起转的轮子飞车），其余轮继续加到都起转或到上限。

python scripts/chassis_model_ident.py --port COM8
python scripts/chassis_model_ident.py --file temp/chassis_ident.csv

单位：CSV 时间 ms，转速 mrad/s，电流 mA。
"""
import argparse
import csv
import re
import time

WHEELS = 4
FIELDS = (["t_ms", "dir"]
          + [f"cmd{w}" for w in range(WHEELS)]
          + [f"omega{w}" for w in range(WHEELS)]
          + [f"cur{w}" for w in range(WHEELS)]
          + [f"acc{w}" for w in range(WHEELS)]
          + ["dt_us"])

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+)," * 12 + r"(-?\d+)\s*$")
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

        # 先停掉残留调试流，再清缓冲
        ser.write(b"log off\r\n")
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()

        send("log off")
        time.sleep(0.2)

        # 开环电流通道，先给 0 让条目建立
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
        result = {}          # (dir, wheel) -> 起转电流
        cmd = [0.0] * WHEELS # 每轮当前指令电流，写 CSV 用

        def set_currents(values: list) -> None:
            for i in range(WHEELS):
                cmd[i] = values[i]
                send(f"var set chassis_test_i{i} {values[i]:.3f}")

        def zero_all() -> None:
            set_currents([0.0] * WHEELS)

        def wait_stopped() -> None:
            """清零后轮询到四轮都停稳，超时则放弃。"""
            zero_all()
            deadline = time.monotonic() + args.stop_timeout
            still_since = None

            while time.monotonic() < deadline:
                match = DATA.search(clean(ser.readline()))
                if not match:
                    continue

                values = [int(x) for x in match.groups()]
                if max(abs(values[i]) for i in range(WHEELS)) / 1000.0 < 0.05:
                    if still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since >= 0.3:
                        return
                else:
                    still_since = None

            print("[!] 等待停稳超时，继续", flush=True)

        try:
            for direction, sign in (("正", 1.0), ("反", -1.0)):
                wait_stopped()

                found = [None] * WHEELS
                level = args.sweep_min

                while level <= args.sweep_max + 1e-6 and any(f is None for f in found):
                    # 已起转的轮子断电自由减速（继续驱动会飞车），未起转的给当前档
                    set_currents([0.0 if found[i] is not None else sign * level for i in range(WHEELS)])

                    peak = [0.0] * WHEELS
                    until = time.monotonic() + args.sweep_hold
                    while time.monotonic() < until:
                        line = clean(ser.readline())
                        match = DATA.search(line)
                        if not match:
                            continue

                        values = [int(x) for x in match.groups()]
                        for i in range(WHEELS):
                            peak[i] = max(peak[i], abs(values[i]) / 1000.0)

                        t_ms = int((time.monotonic() - start) * 1000)
                        writer.writerow([t_ms, direction] + [int(c * 1000) for c in cmd] + values)
                        count += 1

                        now = time.monotonic()
                        if now - beat >= 2:
                            output.flush()
                            wheels = " ".join(f"w{i}={values[i] / 1000.0:+.2f}" for i in range(WHEELS))
                            print(f"[hb] {now - start:.0f}s {direction}向 {level:.2f}A {wheels}",
                                  flush=True)
                            beat = now

                    newly = [i for i in range(WHEELS) if found[i] is None and peak[i] >= args.threshold]

                    for i in newly:
                        found[i] = level

                    # 刚起转的轮子保持一下让肉眼能看到，其余轮断电
                    if newly and args.confirm > 0:
                        set_currents([sign * found[i] if (found[i] is not None and i in newly) else 0.0
                                      for i in range(WHEELS)])

                        print(f"[i] {direction}向 轮 {newly} 起转 @ {level:.2f}A，保持 {args.confirm:.1f}s",
                              flush=True)

                        until = time.monotonic() + args.confirm
                        while time.monotonic() < until:
                            match = DATA.search(clean(ser.readline()))
                            if not match:
                                continue
                            values = [int(x) for x in match.groups()]
                            t_ms = int((time.monotonic() - start) * 1000)
                            writer.writerow([t_ms, direction] + [int(c * 1000) for c in cmd] + values)
                            count += 1

                        for i in newly:
                            send(f"var set chassis_test_i{i} 0")
                            cmd[i] = 0.0

                    level += args.sweep_step

                zero_all()

                # 粗扫已确认「该档能起转」→ 它是上界；下界往回收一个 span
                # 粗扫没测到 → 上界就是 sweep_max
                hi = [(found[i] if found[i] is not None else args.sweep_max) for i in range(WHEELS)]
                lo = [max(args.sweep_min, hi[i] - args.refine_span) for i in range(WHEELS)]

                print(f"[i] {direction}向粗扫 {[round(f, 2) if f is not None else None for f in found]}，"
                      f"开始二分细化 {args.refine_iters} 轮", flush=True)

                for _ in range(args.refine_iters):
                    wait_stopped()

                    # 每轮用自己的中点，四轮同时测
                    mid = [0.5 * (lo[i] + hi[i]) for i in range(WHEELS)]
                    set_currents([sign * m for m in mid])

                    peak = [0.0] * WHEELS
                    until = time.monotonic() + args.refine_hold
                    while time.monotonic() < until:
                        match = DATA.search(clean(ser.readline()))
                        if not match:
                            continue
                        values = [int(x) for x in match.groups()]
                        for i in range(WHEELS):
                            peak[i] = max(peak[i], abs(values[i]) / 1000.0)

                        t_ms = int((time.monotonic() - start) * 1000)
                        writer.writerow([t_ms, direction] + [int(c * 1000) for c in cmd] + values)
                        count += 1

                    zero_all()

                    for i in range(WHEELS):
                        if peak[i] >= args.threshold:
                            hi[i] = mid[i]       # 这一档能起转 → 真值在低半边
                        else:
                            lo[i] = mid[i]       # 起不来 → 真值在高半边

                    print(f"[i] {direction}向 二分 {[round(m, 3) for m in mid]} A  "
                          f"|| 区间宽度 {[round(hi[i] - lo[i], 3) for i in range(WHEELS)]}", flush=True)

                for i in range(WHEELS):
                    result[(direction, i)] = hi[i]

                print(f"[i] {direction}向起转电流: "
                      f"{[round(hi[i], 3) for i in range(WHEELS)]} A", flush=True)
        finally:
            zero_all()
            send("var set chassis_test_enable 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)

        if count == 0:
            raise RuntimeError("没收到 chassis_ident 数据")

        print()
        print("起转电流表（A）")
        print(f"{'方向':>4} " + " ".join(f"{'轮' + str(i):>8}" for i in range(WHEELS)))
        for direction in ("正", "反"):
            print(f"{direction:>4} " + " ".join(f"{result[(direction, i)]:8.2f}" for i in range(WHEELS)))


def analyze(path: str, threshold: float = 0.8) -> None:
    """从 CSV 复核起转电流：按方向、按轮，找转速首次超过阈值的档位。"""
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    print(f"[i] 样本 {len(rows)}")
    print()
    print(f"{'方向':>4} " + " ".join(f"{'轮' + str(i):>8}" for i in range(WHEELS)))

    for direction in ("正", "反"):
        seg = [r for r in rows if r["dir"] == direction]
        out = []

        for w in range(WHEELS):
            col = f"cmd{w}"
            found = None
            # 跳过 0 档：它包含制动/惯性滑行，转速不是电流驱动的结果
            for level in sorted({abs(int(r[col])) for r in seg if abs(int(r[col])) > 0}):
                idx = [r for r in seg if abs(int(r[col])) == level]
                peak = max((abs(int(r[f"omega{w}"])) / 1000.0 for r in idx), default=0.0)
                if peak >= threshold:
                    found = level / 1000.0
                    break
            out.append(found)

        print(f"{direction:>4} " + " ".join(
            (f"{v:8.2f}" if v is not None else f"{'未测到':>8}") for v in out))


def main() -> None:
    parser = argparse.ArgumentParser(description="C620 减速箱静摩擦辨识（起转电流）")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--sweep-min", type=float, default=0.2, help="起始电流 A")
    parser.add_argument("--sweep-max", type=float, default=1.5, help="最大电流 A")
    parser.add_argument("--sweep-step", type=float, default=0.02, help="每档增量 A")
    parser.add_argument("--sweep-hold", type=float, default=0.25, help="每档保持时间 s")
    parser.add_argument("--threshold", type=float, default=0.8, help="判定起转的转速阈值 rad/s（0.8≈7.6rpm，肉眼可见）")
    parser.add_argument("--confirm", type=float, default=1.0, help="起转后保持时间 s，便于肉眼确认")
    parser.add_argument("--refine-span", type=float, default=0.15, help="二分细化初始区间半宽 A")
    parser.add_argument("--refine-iters", type=int, default=8, help="二分细化轮数")
    parser.add_argument("--refine-hold", type=float, default=0.5, help="二分每档保持时间 s")
    parser.add_argument("--stop-timeout", type=float, default=10.0, help="等待停稳的超时 s")
    parser.add_argument("--save", default="temp/chassis_ident.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file, args.threshold)
        return

    capture(args)
    analyze(args.save, args.threshold)


if __name__ == "__main__":
    main()
