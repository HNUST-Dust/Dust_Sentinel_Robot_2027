"""模拟串口跑完整采集与拟合，并验证采集保护。不会操作硬件。"""
import contextlib
import csv
import importlib.util
import io
import math
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location(
    "down_ident", Path(__file__).resolve().parents[1]/"pitch_down_gravity_ident.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class FakeSerial:
    def __init__(self, clock, **faults):
        self.clock, self.faults = clock, faults
        self.queue, self.commands, self.targets = [], [], []
        self.target, self.direction, self.on = 10.0, 1, False
        self.created = False
        self.pulse = False
        self.pulse_count = 0

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
            self.queue.append("log on: ok x10")
        elif cmd.startswith("log mode"):
            self.queue.append("log mode: vofa" if self.created else "log mode: not found")
        elif cmd.startswith("var set pitch_down_target"):
            target = float(cmd.split()[-1])
            if target != self.target:
                self.direction = 1 if target > self.target else -1
                self.pulse = bool(self.faults.get("saturation_pulse"))
            if not self.faults.get("ignore_target"):
                self.target = target
            self.targets.append(target)
            self.queue.append("ok")
        else:
            raise AssertionError(f"unexpected command: {cmd}")

    def readline(self):
        self.clock.now += 0.02
        if self.queue:
            return (self.queue.pop(0)+"\r\n").encode()
        if not self.on or self.faults.get("no_data"):
            return b""
        torque = .2*math.sin(math.radians(self.target))+.3*math.cos(math.radians(self.target))
        torque += self.direction*.04
        if self.pulse:
            torque = self.direction*3.0
            self.pulse = False
            self.pulse_count += 1
        values = [int(self.clock.now*1000), round(self.target*1000), round(self.target*1000),
                  round(torque*1000), round(torque*1000), self.faults.get("speed", 0),
                  self.faults.get("sb", 1)]
        return ("PDIDENT,"+",".join(map(str, values))+"\r\n").encode()


class IdentTests(unittest.TestCase):
    def test_movement_saturation_pulses_do_not_abort_capture(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"pulse.csv"), "--cycles", "1"])
            clock = Clock()
            ser = FakeSerial(clock, saturation_pulse=True)
            M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
            self.assertGreater(ser.pulse_count, 100)
            fit = M.analyze(a.save)
            self.assertAlmostEqual(fit['sin_nm'], .2, delta=.003)
            self.assertAlmostEqual(fit['cos_nm'], .3, delta=.003)
            self.assertEqual(ser.commands[-1], "log off")

    def test_start_outside_measurement_range_ramps_without_jump(self):
        for initial in (0, 5, 75, 80):
            with self.subTest(initial=initial), tempfile.TemporaryDirectory() as tmp, \
                    contextlib.redirect_stdout(io.StringIO()):
                a = M.parser().parse_args(["--save", str(Path(tmp)/"capture.csv"), "--cycles", "1"])
                clock = Clock()
                ser = FakeSerial(clock)
                ser.target = initial
                M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
                self.assertEqual(ser.targets[0], initial)
                self.assertTrue(all(0 <= t <= 80 for t in ser.targets))
                self.assertTrue(all(abs(a-b) <= 1.001 for a, b in zip(ser.targets, ser.targets[1:])))
                with Path(a.save).open(encoding="utf-8") as f:
                    rows = list(csv.DictReader(f))
                self.assertTrue(rows)
                self.assertTrue(all(10 <= float(r['point_deg']) <= 70 for r in rows))

    def test_early_abort_holds_current_angle_outside_measurement_range(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"bad.csv")])
            clock = Clock()
            ser = FakeSerial(clock, speed=800)
            ser.target = 5
            with self.assertRaises(RuntimeError):
                M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
            self.assertEqual(ser.targets, [5])
            self.assertEqual(ser.commands[-1], "log off")

    def test_capture_and_analyze_full_default_range(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"capture.csv")])
            M.validate(a)
            clock = Clock()
            ser = FakeSerial(clock)
            M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
            fit = M.analyze(a.save)
            self.assertAlmostEqual(fit['sin_nm'], .2, delta=.003)
            self.assertAlmostEqual(fit['cos_nm'], .3, delta=.003)
            with Path(a.save).open(encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertTrue(rows)
            self.assertEqual(set(rows[0]), set(M.FIELDS))
            self.assertTrue(all(10 <= t <= 70 for t in ser.targets))
            self.assertTrue(all(abs(a-b) <= 1.001 for a, b in zip(ser.targets, ser.targets[1:])))
            self.assertEqual(ser.commands[-1], "log off")
            self.assertLess(ser.commands.index("log on pitch_down_ident 10"),
                            ser.commands.index("log mode pitch_down_ident on"))
            self.assertTrue(all(int(r['omega_mrad_s']) == 0 for r in rows))
            self.assertTrue(Path(a.save).with_name("capture_fit.json").exists())

    def test_capture_protections_and_cleanup(self):
        for fault in ({"sb": 0}, {"speed": 800}, {"no_data": True}, {"speed": 40},
                      {"ignore_target": True}):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp, \
                    contextlib.redirect_stdout(io.StringIO()):
                a = M.parser().parse_args(["--save", str(Path(tmp)/"bad.csv")])
                clock = Clock()
                ser = FakeSerial(clock, **fault)
                with self.assertRaises(RuntimeError):
                    M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
                self.assertEqual(ser.commands[-1], "log off")
                with Path(a.save).open(encoding="utf-8") as f:
                    rows = list(csv.DictReader(f))
                self.assertEqual(rows, [])

    def test_single_direction_cannot_fit_gravity(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"one.csv"
            with p.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(M.FIELDS)
                w.writerow([0, 1, 20, 100, 20000, 20000, 300, 300, 0, 1])
            with self.assertRaises(ValueError):
                M.analyze(p)


if __name__ == "__main__":
    unittest.main()
