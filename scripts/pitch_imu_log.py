"""下 pitch 采集：摆动 + 静止两段，记录电机角、IMU 姿态与 EKF 内部状态。

只读日志，不发任何控制命令。

python scripts/pitch_imu_log.py --port COM8 --seconds 30
python scripts/pitch_imu_log.py --file pitch_imu_log.csv
"""
import argparse
import csv
import re
import time

FIELDS = [
    "t_ms", "phase",
    "target_deg", "motor_deg", "torque_nm",
    "imu_pitch", "imu_roll", "imu_yaw",
    "bg_z_urad", "chi2_e9", "k_scale_e3",
    "w_norm_mrad", "a_norm_mm", "stable",
]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
TRACK = re.compile(r"(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
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
        configure(f"log on pitch_track {args.divisor}")

        start = time.monotonic()
        count = 0

        def pump(phase: int, deadline: float, label: str) -> None:
            nonlocal count
            last_beat = time.monotonic()
            while time.monotonic() < deadline:
                line = clean(ser.readline())
                m = TRACK.search(line)
                if m:
                    values = [float(x) for x in m.groups()]
                    t_ms = int((time.monotonic() - start) * 1000)
                    writer.writerow([t_ms, phase] + values)
                    count += 1

                now = time.monotonic()
                if now - last_beat >= 2:
                    output.flush()
                    print(f"[hb] {label} {now - start:.0f}s 样本={count}", flush=True)
                    last_beat = now

        try:
            print(f"[i] 摆动段 {args.seconds}s，请手动摆动云台", flush=True)
            pump(0, start + args.seconds, "摆动")

            # 暂停输出，防止等待放平时积压的运动样本被标成静止段。
            configure("log off")
            output.flush()
            hold_until = time.monotonic() + args.hold
            while time.monotonic() < hold_until:
                remaining = hold_until - time.monotonic()
                print(f">>> 请把云台放回水平位置并保持静止，{remaining:.0f}s 后继续采集 <<<", flush=True)
                time.sleep(min(1.0, remaining))

            ser.reset_input_buffer()
            configure(f"log on pitch_track {args.divisor}")
            print(f"[i] 静止段 {args.seconds}s", flush=True)
            pump(1, time.monotonic() + args.seconds, "静止")
        finally:
            send("log off")
        output.flush()
        print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)


def analyze(path: str) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    t = np.array([float(r["t_ms"]) / 1000.0 for r in rows])
    phase = np.array([int(float(r["phase"])) for r in rows])
    target = np.array([float(r["target_deg"]) / 1000.0 for r in rows])
    motor = np.array([float(r["motor_deg"]) / 1000.0 for r in rows])
    torque = np.array([float(r["torque_nm"]) / 1000.0 for r in rows])
    pitch = np.array([float(r["imu_pitch"]) / 1000.0 for r in rows])
    roll = np.array([float(r["imu_roll"]) / 1000.0 for r in rows])
    yaw = np.array([float(r["imu_yaw"]) / 1000.0 for r in rows])
    bgz = np.array([float(r["bg_z_urad"]) for r in rows])
    chi2 = np.array([float(r["chi2_e9"]) for r in rows])
    kscale = np.array([float(r["k_scale_e3"]) for r in rows])
    wnorm = np.array([float(r["w_norm_mrad"]) for r in rows])
    anorm = np.array([float(r["a_norm_mm"]) for r in rows])
    stable = np.array([int(float(r["stable"])) for r in rows])

    print(f"[i] 样本 {len(rows)}，时长 {t[-1] - t[0]:.1f}s，平均 {len(rows) / (t[-1] - t[0]):.0f} Hz")
    print()

    for ph, name in ((0, "摆动段"), (1, "静止段")):
        sel = phase == ph
        if sel.sum() < 10:
            print(f"[{name}] 样本不足，跳过")
            continue

        print(f"[{name}] {int(sel.sum())} 样本")
        print(f"  电机 {motor[sel].min():+.2f}° ~ {motor[sel].max():+.2f}°（峰峰 {np.ptp(motor[sel]):.2f}°）")
        print(f"  pitch {pitch[sel].min():+.2f}° ~ {pitch[sel].max():+.2f}°（峰峰 {np.ptp(pitch[sel]):.2f}°）")
        print(f"  roll  {roll[sel].min():+.2f}° ~ {roll[sel].max():+.2f}°")
        print(f"  yaw   {yaw[sel].min():+.2f}° ~ {yaw[sel].max():+.2f}°")
        print(f"  bgz  {bgz[sel].mean():+.1f} urad/s")
        print(f"  chi2 中位={np.median(chi2[sel]):.1f} 最大={chi2[sel].max():.1f}（×1e9，运动门控阈值 1000；静止恢复另行限速）")
        print(f"  k_scale 中位={np.median(kscale[sel]):.0f} 最小={kscale[sel].min():.0f}（×1e3，满值 1000）")
        print(f"  非零增益占比 {100.0 * np.mean(kscale[sel] > 0):.1f}%（日志整数化后）")
        print(f"  w_norm 中位={np.median(wnorm[sel]):.1f} mrad/s")
        print(f"  a_norm 中位={np.median(anorm[sel]):.0f} mm/s^2（重力约 9807）")
        print(f"  stable 占比 {100.0 * stable[sel].mean():.0f}%")
        print()

    # 只用最后一次明显运动后再稳定 2 秒的连续尾段计算漂移。
    # 兼容旧 CSV 中 phase=1 开头混入运动的情况。
    sel = stationary_tail(t, phase, motor, wnorm, stable)
    if sel.sum() > 50:
        print(f"[静止尾段] {t[sel][0]:.2f}s ~ {t[sel][-1]:.2f}s，"
              f"电机峰峰 {np.ptp(motor[sel]):.3f}°")
        k, _ = np.polyfit(t[sel], pitch[sel], 1)
        print(f"[静止段漂移] IMU {pitch[sel][0]:+.2f}° → {pitch[sel][-1]:+.2f}°"
              f"（变化 {pitch[sel][-1] - pitch[sel][0]:+.3f}°），速率 {k:+.4f} °/s")
        kb, _ = np.polyfit(t[sel], bgz[sel], 1)
        print(f"  bg_z 趋势 {kb:+.3f} urad/s^2")
        print()
        # 单列最后 5 秒，区分运动后恢复校正与最终稳态变化。
        end_sel = sel & (t >= t[sel][-1] - 5.0)
        if end_sel.sum() > 20 and np.ptp(t[end_sel]) >= 4.5:
            end_k, _ = np.polyfit(t[end_sel], pitch[end_sel], 1)
            print(f"[末 5 秒] pitch 峰峰 {np.ptp(pitch[end_sel]):.3f}°，"
                  f"变化 {pitch[end_sel][-1] - pitch[end_sel][0]:+.3f}°，"
                  f"速率 {end_k:+.4f} °/s，"
                  f"非零增益 {100.0 * np.mean(kscale[end_sel] > 0):.1f}%")
            print()
    else:
        print("[静止段漂移] 连续静止尾段不足或电机仍有运动，跳过")

    # 摆动段线性关系
    sel = phase == 0
    if sel.sum() > 50 and np.ptp(motor[sel]) > 1.0:
        k, b = np.polyfit(motor[sel], pitch[sel], 1)
        residual = pitch[sel] - (k * motor[sel] + b)
        print(f"[摆动段线性] imu = {k:+.4f} * motor {b:+.3f}，RMS {np.sqrt(np.mean(residual ** 2)):.2f}°")
        print()


def stationary_tail(t, phase, motor, wnorm, stable):
    import numpy as np

    indices = np.flatnonzero(phase == 1)
    sel = np.zeros(len(t), dtype=bool)
    if not len(indices):
        return sel
    movement = (wnorm[indices] > 20.0) | (stable[indices] == 0)
    movement[1:] |= np.abs(np.diff(motor[indices])) > 0.05
    last_motion = indices[movement][-1] if movement.any() else indices[0]
    sel = (phase == 1) & (t >= t[last_motion] + 2.0)
    if sel.any() and (np.ptp(motor[sel]) > 0.2 or np.ptp(t[sel]) < 5.0):
        sel[:] = False
    return sel


def main() -> None:
    parser = argparse.ArgumentParser(description="下 pitch 采集（摆动+静止，含 EKF 内部状态）")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--seconds", type=float, default=30.0, help="每段采集时长（秒）")
    parser.add_argument("--hold", type=float, default=5.0, help="两段之间等待放平的秒数")
    parser.add_argument("--divisor", type=int, default=10, help="DBG 分频，10=100Hz")
    parser.add_argument("--save", default="pitch_imu_log.csv")
    parser.add_argument("--file", help="只分析已有 CSV")
    args = parser.parse_args()

    if args.file:
        analyze(args.file)
        return

    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
