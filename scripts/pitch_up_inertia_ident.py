"""上 pitch 角度环惯量辨识：目标阶跃往复 → 角加速度回归。

## 物理模型

上 pitch 位置环（ki=0）输出 PD 力矩，运动方程：

    J·theta_ddot = tau_pid + tau_g(theta) - tau_f·sign(theta_dot)

其中 tau_pid = kp·e + kd·e_dot，tau_g(theta) 为固件同一重力前馈式：

    tau_g(theta) = 0.041619·sin(theta) + 0.315195·cos(theta)

tau_f 是摩擦，静止段会主导数据、且符号随运动方向翻转，直接回归会把 J 估大。

## 辨识做法

同一目标阶跃的**正向段与反向段**配对（角度、速度幅值接近）：

    J·(theta_ddot_f + theta_ddot_b) = (tau_f + tau_b) + 2·tau_g(theta)

摩擦项一正一负相消，只剩重力与惯量。对 (theta_ddot_f+theta_ddot_b) 与 (tau_f+tau_b+2·tau_g) 做过原点
线性回归，斜率即 1/J。

## 前提

- 固件 DBG `pitch_up_track` = target_deg, imu_pitch_deg, up_torque_nm, gyro_y_rad_s
- 上 pitch 无软件限位，脚本必须自己限制目标角范围（--low/--high）
- 下 pitch 当前下发零力矩，不影响上轴
- 需遥控 SB 上档（custom1=On）才会走 `pitch_up_target` 目标

python scripts/pitch_up_inertia_ident.py --port COM8
python scripts/pitch_up_inertia_ident.py --file temp/pitch_up_inertia.csv

单位：CSV 时间 ms，角度 mdeg，力矩 mN*m，角速度 mrad/s。
"""
import argparse
import csv
import re
import time

FIELDS = ["t_ms", "seg", "target_mdeg", "angle_mdeg", "torque_mnm", "gyro_mrad_s"]

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DATA = re.compile(r"(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\s*$")

BAUD = 921600

# 固件 GravityTorque 的拟合系数（trd_pitch.cpp）
GRAVITY_SIN = 0.041619
GRAVITY_COS = 0.315195


def clean(raw: bytes) -> str:
    return ANSI.sub("", raw.decode("utf-8", errors="replace")).strip()


def gravity_torque(angle_rad: float) -> float:
    """与固件 GravityTorque 同一公式，用于从总力矩里扣除重力分量。"""
    import math
    return GRAVITY_SIN * math.sin(angle_rad) + GRAVITY_COS * math.cos(angle_rad)


class Link:
    """串口读写：子类可替换 data_pattern 匹配不同 DBG。"""

    data_pattern = DATA

    def __init__(self, ser, clock=time.monotonic):
        self.ser = ser
        self.clock = clock
        self.latest = None
        self.last_beat = clock()

    def send(self, command: str) -> None:
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()

    def read(self):
        line = clean(self.ser.readline())
        match = self.data_pattern.search(line)
        data = [float(x) for x in match.groups()] if match else None
        if data:
            self.latest = data
        if self.clock() - self.last_beat >= 1:
            print(f"[hb] {self.latest}", flush=True)
            self.last_beat = self.clock()
        return line, data

    def command(self, command: str, prefix: str = "ok") -> None:
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
            target, angle, torque, gyro = d
            if not args.low - 5.0 <= angle <= args.high + 5.0:
                raise RuntimeError(f"角度越界: {angle:.3f}°（限位 {args.low}~{args.high}）")
            if abs(gyro) > args.max_speed:
                raise RuntimeError(f"速度保护: {gyro:.3f} rad/s")
            if abs(torque) >= args.max_torque * 0.98:
                raise RuntimeError(f"力矩饱和: {torque:.3f} N*m")
            return d
        raise RuntimeError("辨识日志超时；检查 SB 上档、遥控、电机及固件")


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="上 pitch 角度环惯量辨识")
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--low", type=float, default=-10.0, help="最低目标角（度）")
    p.add_argument("--high", type=float, default=40.0, help="最高目标角（度）")
    p.add_argument("--amplitude", type=float, default=15.0, help="阶跃幅值（度）")
    p.add_argument("--cycles", type=int, default=5, help="阶跃往复轮数")
    p.add_argument("--hold-ms", type=int, default=1500, help="每段保持时间 ms")
    p.add_argument("--timeout", type=float, default=3.0, help="单次采样超时 s")
    p.add_argument("--max-speed", type=float, default=8.0, help="角速度保护 rad/s")
    p.add_argument("--max-torque", type=float, default=7.0, help="力矩饱和判据 N*m")
    p.add_argument("--save", default="temp/pitch_up_inertia.csv")
    p.add_argument("--file", help="只分析已有 CSV")
    return p


def validate(args) -> None:
    if not args.low < args.high:
        raise ValueError("low 必须小于 high")
    if args.high - args.low < args.amplitude:
        raise ValueError("范围必须能容纳至少一个阶跃幅值")
    if args.amplitude <= 1.0:
        raise ValueError("阶跃幅值太小，无法激起足够角加速度")
    if args.cycles < 1 or args.hold_ms <= 0:
        raise ValueError("cycles/hold-ms 必须为正")


def capture(args, serial_factory=None, clock=time.monotonic) -> None:
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial

    with serial_factory(args.port, args.baud, timeout=0.05, write_timeout=1) as ser, \
            open(args.save, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)

        def send(command: str) -> None:
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()

        def receive():
            line = clean(ser.readline())
            match = DATA.search(line)
            if match:
                return line, [float(x) for x in match.groups()]
            return line, None

        def configure(command: str) -> None:
            send(command)
            deadline = clock() + 3
            while clock() < deadline:
                line, _ = receive()
                if "not found" in line or line.startswith(("error", "usage:")):
                    raise RuntimeError(f"命令失败: {command}: {line}")
                if line.startswith(("ok", "log off: ok", "log on: ok", "log mode:")):
                    return
            raise RuntimeError(f"命令未确认: {command}")

        # 先停残留流，再切日志
        ser.write(b"log off\r\n")
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()

        configure("log off")
        # 先 log on 创建条目，再切 log mode；顺序反了会报 not found
        configure("log on pitch_up_track 1")
        configure("log mode pitch_up_track on")

        # 起点：范围中点；每次阶跃在 中点±amplitude 之间往复
        center = (args.low + args.high) * 0.5
        low_pt = max(args.low, center - args.amplitude)
        high_pt = min(args.high, center + args.amplitude)

        start = beat = clock()
        seg = 0
        count = 0

        def hold(target: float, seg_id: int) -> None:
            nonlocal count, beat

            until = clock() + args.hold_ms / 1000.0
            while clock() < until:
                _, data = receive()
                if not data:
                    continue

                # 保护：角度越界、超速、力矩饱和任一命中即中止
                if not args.low - 5.0 <= data[1] <= args.high + 5.0:
                    raise RuntimeError(f"角度越界: {data[1]:.3f}°（限位 {args.low}~{args.high}）")
                if abs(data[3]) > args.max_speed:
                    raise RuntimeError(f"速度保护: {data[3]:.3f} rad/s")
                if abs(data[2]) >= args.max_torque * 0.98:
                    raise RuntimeError(f"力矩饱和: {data[2]:.3f} N*m")

                t_ms = int((clock() - start) * 1000)
                writer.writerow([t_ms, seg_id,
                                 int(data[0] * 1000), int(data[1] * 1000),
                                 int(data[2] * 1000), int(data[3] * 1000)])
                count += 1

                now = clock()
                if now - beat >= 2:
                    output.flush()
                    print(f"[hb] {now - start:.0f}s 段{seg_id} 目标={data[0]:.2f}° "
                          f"实际={data[1]:.2f}° 力矩={data[2]:+.3f} 速度={data[3]:+.2f} "
                          f"样本={count}", flush=True)
                    beat = now

        print(f"[i] 阶跃范围 {low_pt:.1f}° <-> {high_pt:.1f}°，{args.cycles} 轮", flush=True)

        try:
            # 先到位再开始，避免第一段从任意位置大阶跃
            configure(f"var set pitch_up_target {center:.2f}")
            hold(center, -1)

            for cycle in range(args.cycles):
                for target, direction in ((high_pt, 1), (low_pt, -1)):
                    configure(f"var set pitch_up_target {target:.2f}")
                    print(f"[i] 轮{cycle + 1} 阶跃 {direction:+d} → {target:.2f}°", flush=True)
                    hold(target, seg)
                    seg += 1
        finally:
            send(f"var set pitch_up_target {center:.2f}")
            send("log off")
            output.flush()
            print(f"[i] 已保存 {count} 条到 {args.save}", flush=True)

        if count == 0:
            raise RuntimeError("没收到 pitch_up_track 数据")


def analyze(path: str, min_speed: float = 0.3, min_acc: float = 2.0,
            smooth: int = 5) -> None:
    import numpy as np

    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError("CSV 为空")

    seg = np.array([int(r["seg"]) for r in rows])
    angle = np.array([float(r["angle_mdeg"]) / 1000.0 for r in rows])
    torque = np.array([float(r["torque_mnm"]) / 1000.0 for r in rows])
    gyro = np.array([float(r["gyro_mrad_s"]) / 1000.0 for r in rows])

    print(f"[i] 样本 {len(rows)}，有效段 {len(set(seg) - {-1})}")

    # 采样周期：t_ms 是整数毫秒，1kHz 下相邻差为 0，不能用它算加速度。
    # 改为用总时长 / 样本数估平均周期（DBG 是固定节拍）。
    total_ms = float(rows[-1]["t_ms"]) - float(rows[0]["t_ms"])
    if total_ms <= 0:
        raise ValueError("CSV 时间戳无跨度，无法估采样周期")
    dt = total_ms / 1000.0 / max(len(rows) - 1, 1)
    print(f"[i] 平均采样周期 {dt * 1000.0:.3f} ms（{1.0 / dt:.0f} Hz）")

    # 角加速度：先滑动平均压噪声，再做中心差分
    def moving_average(x, n):
        if n <= 1 or len(x) < n:
            return x.copy()
        kernel = np.ones(n) / n
        padded = np.concatenate([np.full(n // 2, x[0]), x, np.full(n // 2, x[-1])])
        return np.convolve(padded, kernel, mode="valid")[:len(x)]

    gyro_s = moving_average(gyro, smooth)
    alpha = np.zeros_like(gyro_s)
    if len(gyro_s) >= 3:
        alpha[1:-1] = (gyro_s[2:] - gyro_s[:-2]) / (2.0 * dt)

    # 重力分量（按 IMU 角度，度 → rad）
    gravity = np.array([gravity_torque(np.deg2rad(a)) for a in angle])

    # 净驱动力矩 = 总力矩 - 重力前馈
    net = torque - gravity

    # 按段切分，每段只取"运动段"：速度够大、加速度够大，避开静止摩擦主导
    segments = {}
    for s in sorted(set(seg)):
        if s < 0:
            continue
        idx = np.flatnonzero(seg == s)
        if len(idx) < 10:
            continue
        moving = idx[np.abs(gyro[idx]) >= min_speed]
        moving = moving[np.abs(alpha[moving]) >= min_acc]
        if len(moving) < 5:
            print(f"[!] 段{s} 运动样本不足（{len(moving)}），跳过")
            continue
        segments[s] = moving

    if len(segments) < 2:
        raise RuntimeError("有效段不足 2 个，无法配对；检查阶跃幅值/保持时间")

    print(f"\n{'段':>3} {'样本':>5} {'angle':>8} {'speed':>8} {'alpha':>9} {'net':>9}")
    for s, idx in segments.items():
        print(f"{s:3d} {len(idx):5d} {angle[idx].mean():8.3f} {gyro[idx].mean():8.3f} "
              f"{alpha[idx].mean():9.3f} {net[idx].mean():9.4f}")

    # 两未知量最小二乘：J·alpha + tau_f·sign(omega) = net
    # 设计矩阵 [-alpha, sign(omega)]，解 [J, tau_f]；用所有运动样本联合求解，
    # 比"正反向配对相减"稳（配对要求两方向速度/加速度幅值严格对称，实际做不到）。
    alpha_all, net_all, sign_all = [], [], []
    for idx in segments.values():
        alpha_all.append(alpha[idx])
        net_all.append(net[idx])
        sign_all.append(np.sign(gyro[idx]))

    a = np.concatenate(alpha_all)
    b = np.concatenate(net_all)
    sgn = np.concatenate(sign_all)

    # net = J*alpha + tau_f*sgn  →  未知量 [J, tau_f]
    design = np.column_stack([a, sgn])
    coeff, _, rank, _ = np.linalg.lstsq(design, b, rcond=None)

    if rank < 2:
        raise RuntimeError("设计矩阵秩不足：正反向样本不对称，无法分离 J 与摩擦")

    j_est, tau_f = float(coeff[0]), float(coeff[1])
    pred = design @ coeff
    ss_res = float(((b - pred) ** 2).sum())
    ss_tot = float(((b - b.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(((b - pred) ** 2).mean()))

    # 共线性诊断：J 与 tau_f 是否可分。两列相关系数接近 ±1 时摩擦项不可辨识，
    # 此时 J 仍可信（它是主项），但 tau_f 不能当结论用。
    cond = float(np.linalg.cond(design))

    print(f"\n[fit] J = {j_est:.6f} kg*m^2")
    print(f"[fit] 摩擦 tau_f = {tau_f:+.4f} N*m（仅当条件数低时可信）")
    print(f"[fit] R2 = {r2:.4f}  RMSE = {rmse:.4f} N*m  样本 = {len(a)}  条件数 = {cond:.1f}")

    if j_est <= 0:
        print("[!] J 非正：激励不足或符号约定不符，结果不可用", flush=True)
    elif r2 < 0.5:
        print("[!] R2 偏低：摩擦模型不充分或数据噪声大，J 仅供参考", flush=True)
    elif cond > 30.0:
        print(f"[!] 条件数 {cond:.0f} 偏高：alpha 与 sign(omega) 共线，tau_f 不可辨识。"
              "J 仍可用，但需增加匀速段（延长保持时间）才能分离摩擦。", flush=True)
    else:
        print("[ok] 拟合可接受", flush=True)

    return j_est, tau_f, r2


def main() -> None:
    args = parser().parse_args()

    if args.file:
        analyze(args.file)
        return

    validate(args)
    capture(args)
    analyze(args.save)


if __name__ == "__main__":
    main()
