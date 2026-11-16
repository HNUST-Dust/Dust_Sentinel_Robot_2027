"""上 pitch 角度环惯量辨识：模拟串口跑完整采集与拟合，并验证保护。不会操作硬件。"""
import contextlib
import csv
import importlib.util
import io
import math
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location(
    "up_ident", Path(__file__).resolve().parents[1] / "pitch_up_inertia_ident.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)

J_TRUE = 0.05          # 合成模型真值 kg*m^2
TAU_F = 0.5            # 合成模型摩擦 N*m
DT = 0.001


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class FakeSerial:
    """二阶假动力学：J*acc = kp*e + kd*vel + gravity - tau_f*sign(vel)。"""

    def __init__(self, clock, **faults):
        self.clock, self.faults = clock, faults
        self.queue, self.commands, self.targets = [], [], []
        self.target = 15.0
        self.angle, self.vel = 15.0, 0.0
        self.on = False
        self.created = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def reset_input_buffer(self):
        self.queue.clear()

    def flush(self):
        pass

    def write(self, raw):
        cmd = raw.decode().strip()
        self.commands.append(cmd)
        if cmd == "log off":
            self.on = False
            self.queue.append("log off: ok")
        elif cmd.startswith("log on"):
            self.on, self.created = True, True
            self.queue.append("log on: ok x1")
        elif cmd.startswith("log mode"):
            self.queue.append("log mode: vofa" if self.created else "log mode: not found")
        elif cmd.startswith("var set pitch_up_target"):
            if not self.faults.get("ignore_target"):
                self.target = float(cmd.split()[-1])
            self.targets.append(self.target)
            self.queue.append("ok")
        else:
            raise AssertionError(f"unexpected command: {cmd}")

    def readline(self):
        self.clock.now += DT
        if self.queue:
            return (self.queue.pop(0) + "\r\n").encode()
        if not self.on or self.faults.get("no_data"):
            return b""

        tgt_rad = math.radians(self.target)
        err = tgt_rad - math.radians(self.angle)
        pid = 1.2 * err + 0.08 * self.vel
        gravity = M.gravity_torque(math.radians(self.angle))
        torque = pid + gravity
        acc = (torque - TAU_F * (1.0 if self.vel >= 0 else -1.0)) / J_TRUE
        self.vel += acc * DT
        self.angle += math.degrees(self.vel * DT)

        if self.faults.get("speed"):
            self.vel = self.faults["speed"]
        if self.faults.get("saturation"):
            torque = 7.0

        line = (f"{self.target:.3f},{self.angle:.3f},{torque:.3f},{self.vel:.3f}")
        return (line + "\r\n").encode()


class IdentTests(unittest.TestCase):
    def run_capture(self, tmp, **faults):
        clock = Clock()
        ser = FakeSerial(clock, **faults)
        a = M.parser().parse_args([
            "--save", str(Path(tmp) / "capture.csv"),
            "--low", "-10", "--high", "40", "--amplitude", "15",
            "--cycles", "6", "--hold-ms", "250",
        ])
        M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
        return a, ser

    def test_capture_writes_all_columns(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a, ser = self.run_capture(tmp)
            with Path(a.save).open(encoding="utf-8") as f:
                rows = list(csv.DictReader(f))

            self.assertTrue(rows)
            self.assertEqual(list(rows[0]), M.FIELDS)
            self.assertEqual(ser.commands[-1], "log off")
            self.assertLess(ser.commands.index("log on pitch_up_track 1"),
                            ser.commands.index("log mode pitch_up_track on"))

    def test_targets_stay_inside_requested_range(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            _, ser = self.run_capture(tmp)
            self.assertTrue(all(-10 <= t <= 40 for t in ser.targets))
            self.assertIn(0.0, ser.targets)     # 起点是中点

    def test_analyze_recovers_known_inertia(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a, _ = self.run_capture(tmp)
            result = M.analyze(a.save)
            self.assertIsNotNone(result)
            j_est, _tau_f, r2 = result
            self.assertAlmostEqual(j_est, J_TRUE, delta=J_TRUE * 0.25)
            self.assertGreater(r2, 0.9)

    def test_speed_protection_aborts_and_cleans_up(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                self.run_capture(tmp, speed=99.0)
            # 异常路径也要发 log off 收尾
            a = Path(tmp) / "capture.csv"
            self.assertTrue(a.exists())

    def test_torque_saturation_aborts(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                self.run_capture(tmp, saturation=True)

    def test_no_data_aborts(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                self.run_capture(tmp, no_data=True)

    def test_angle_outside_range_aborts(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp) / "c.csv"),
                                       "--low", "-10", "--high", "40"])
            clock = Clock()
            ser = FakeSerial(clock)
            ser.angle = 90.0
            with self.assertRaises(RuntimeError):
                M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)

    def test_validate_rejects_bad_ranges(self):
        cases = [
            ["--low", "40", "--high", "-10"],
            ["--low", "0", "--high", "10", "--amplitude", "20"],
            ["--amplitude", "0.5"],
            ["--cycles", "0"],
        ]
        for extra in cases:
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                M.validate(M.parser().parse_args(extra))

    def test_single_direction_cannot_fit(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "one.csv"
            with p.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(M.FIELDS)
                for i in range(200):
                    w.writerow([i, 0, 30000, 30000, 500, 0])

            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(RuntimeError):
                    M.analyze(str(p))


if __name__ == "__main__":
    unittest.main()
