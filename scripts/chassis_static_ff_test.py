"""底盘静摩擦前馈效果测试：低速阶跃响应。

固件在摩擦前馈里加了静摩擦项（低速时用起转电流、随转速平方指数衰减回库仑值）。
本脚本用 chassis_test_omega 直接给速度环低速阶跃，考察：

  1. 起步时间：从零到 90% 目标转速所需时间
  2. 稳态误差：保持段末 30% 的目标 - 实际
  3. 维持电流：保持段末 30% 的实测电流

低速档（0.3~2 rad/s）是静摩擦项起作用的区间，档位越低越明显。

python scripts/chassis_static_ff_test.py --port COM8
python scripts/chassis_static_ff_test.py --file temp/chassis_static_ff.csv

单位：CSV 时间 ms，转速 mrad/s，电流 mA。
"""
import argparse
import csv
import re
import time

WHEELS = 4
FIELDS = (["t_ms", "cmd_omega"]
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

        send("var set chassis_test_omega 0")
        send("var set chassis_test_enable 1")
        time.sleep(0.2)
        send("log on chassis_ident 1")
        time.sleep(0.3)
        send("log mode chassis_ident on")
        time.sleep(0.3)
        ser.reset_input_buffer()

        start = beat = time.monotonic()
        count = 0

        def record(until: float, omega: float) -> None:
            nonlocal count, beat

            while time.monotonic() < until:
                match = DATA.search(clean(ser.readline()))
                if not match:
                    continue

                values = [int(x) for x in match.groups()]
                t_ms = int((time.monotonic() - start) * 1000)
                writer.writerow([t_ms, int(omega * 1000)] + values)
                count += 1

                now = time.monotonic()
                if now - beat >= 2:
                    output.flush()
                    wheels = " ".join(f"w{i}={values[i] / 1000.0:+.2f}" for i in range(WHEELS))
                    print(f"[hb] {now - start:.0f}s 目标={omega:+.2f} {wheels} 样本={count}", flush=True)
                    beat = now

        try:
            for omega in args.levels:
                # 先回零停稳，保证每次阶跃都从静止开始
                send("var set chassis_test_omega 0")
                record(time.monotonic() + args.settle, 0.0)

                send(f"var set chassis_test_omega {omega}")
                print(f"[i] 阶跃到 {omega:+.2f} rad/s，保持 {args.hold}s", flush=True)
                record(time.monotonic() + args.hold, omega)

            send("var set chassis_test_omega 0")
            record(time.monotonic() + args.settle, 0.0)
        finally:
            send("var set chassis_test_omega 0")
            send("var set chassis_test_enable 0")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)

        if count == 0:
            raise RuntimeError("没收到 chassis_ident 数据")


def analyze(path: str, settle_ratio: float = 0.7) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    t = np.array([float(r["t_ms"]) / 1000.0 for r in rows])
    cmd = np.array([float(r["cmd_omega"]) / 1000.0 for r in rows])
    print(f"[i] 样本 {len(rows)}，时长 {t[-1] - t[0]:.1f}s")

    # 从 CSV 的指令目标里取档位，不依赖命令行参数
    levels = sorted({round(v, 3) for v in cmd if v != 0.0})

    for wheel in range(WHEELS):
        omega = np.array([float(r[f"omega{wheel}"]) / 1000.0 for r in rows])
        cur = np.array([float(r[f"cur{wheel}"]) / 1000.0 for r in rows])

        print()
        print(f"=== 轮 {wheel} ===")
        print(f"{'目标':>7} {'起步时间s':>10} {'稳态误差':>10} {'维持电流A':>10} {'样本':>6}")

        for level in levels:
            idx = np.flatnonzero(np.isclose(np.round(cmd, 3), round(level, 3)))
            if len(idx) < 10:
                continue

            # 每段从第一次出现该目标开始
            seg_start = idx[0]
            seg_end = idx[-1] + 1

            # 起步时间：从段首到首次达到 90% 目标
            reach = [i for i in range(seg_start, seg_end)
                     if abs(omega[i]) >= 0.9 * abs(level)]
            rise = (t[reach[0]] - t[seg_start]) if reach else float("nan")

            # 稳态：末段 30%
            n = seg_end - seg_start
            tail = range(seg_start + int(settle_ratio * n), seg_end)
            err = np.mean([level - omega[i] for i in tail])
            cur_med = np.median([cur[i] for i in tail])

            print(f"{level:7.2f} {rise:10.3f} {err:10.3f} {cur_med:10.3f} {n:6d}")


def main() -> None:
    parser = argparse.ArgumentParser(description="底盘静摩擦前馈低速阶跃测试")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--levels", type=float, nargs="+", default=[0.3, 0.5, 1.0, 2.0, 5.0],
                        help="阶跃目标转速 rad/s")
    parser.add_argument("--hold", type=float, default=3.0, help="每档保持时间 s")
    parser.add_argument("--settle", type=float, default=1.5, help="档间回零等待 s")
    parser.add_argument("--save", default="temp/chassis_static_ff.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
