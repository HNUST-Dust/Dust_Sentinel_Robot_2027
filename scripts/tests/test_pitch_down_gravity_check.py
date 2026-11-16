"""模拟串口验证补偿 A/B/A 全链路、恢复及结果计算。"""
import contextlib
import csv
import io
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pitch_down_gravity_check as M


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class FakeSerial:
    def __init__(self, clock, fail_after=None):
        self.clock, self.fail_after = clock, fail_after
        self.queue, self.commands = [], []
        self.target, self.scale, self.on = 10., .4, False
        self.created = False
        self.frames = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.queue.clear()

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if command == "log off":
            self.on = False
            self.queue.append("log off: ok")
        elif command == "var get pitch_down_gscale":
            self.queue.append(f"pitch_down_gscale = {self.scale:.6f}")
        elif command.startswith("log on pitch_down_gravity"):
            self.on, self.created = True, True
            self.queue.append("log on: ok x10")
        elif command == "log mode pitch_down_gravity on":
            self.queue.append("log mode: vofa" if self.created else "log mode: not found")
        elif command.startswith("var set pitch_down_target"):
            self.target = float(command.split()[-1])
            self.queue.append("ok")
        elif command.startswith("var set pitch_down_gscale"):
            self.scale = float(command.split()[-1])
            self.queue.append("ok")
        else:
            raise AssertionError(command)

    def readline(self):
        self.clock.now += .02
        if self.queue:
            return (self.queue.pop(0)+"\r\n").encode()
        if not self.on:
            return b""
        self.frames += 1
        angle = self.target-(1-self.scale)
        gravity = -.581958*math.sin(math.radians(angle))+1.3812*math.cos(math.radians(angle))
        ff = self.scale*gravity
        pid = gravity-ff
        sb = int(self.fail_after is None or self.frames < self.fail_after)
        values = [round(self.clock.now*1000), round(self.target*1000), round(angle*1000),
                  round(gravity*1000), round(gravity*1000), 0, sb,
                  round(pid*1000), round(ff*1000), round(self.scale*1000)]
        return ("PDCHECK,"+",".join(map(str, values))+"\r\n").encode()


class CheckTests(unittest.TestCase):
    def test_full_capture_analysis_and_restore(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"capture.csv")])
            clock = Clock()
            ser = FakeSerial(clock)
            M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
            result = M.analyze(a.save)
            self.assertEqual(len(result), 7)
            self.assertTrue(all(r['comparison_valid'] for r in result))
            self.assertTrue(all(r['compensated_abs_pid_nm'] == 0 for r in result))
            self.assertTrue(all(r['reduction_percent'] is None or abs(r['reduction_percent']-100) < .001 for r in result))
            self.assertTrue(all(abs(r['error_reduction_deg']-1) < .001 for r in result))
            self.assertEqual(ser.scale, .4)
            self.assertEqual(ser.commands[-1], "log off")
            self.assertFalse(any("_kp" in c or "_ki" in c or "_kd" in c for c in ser.commands))
            with open(a.save, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(set(rows[0]), set(M.FIELDS))
            self.assertEqual({int(r['scale_permille']) for r in rows}, {0, 1000})
            self.assertTrue(all(abs(int(r['pid_mnm'])+int(r['gravity_mnm'])-int(r['total_mnm'])) <= 2 for r in rows))

    def test_interrupted_capture_restores_original_scale(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"bad.csv")])
            clock = Clock()
            ser = FakeSerial(clock, fail_after=500)
            with self.assertRaises(RuntimeError):
                M.capture(a, serial_factory=lambda *x, **kw: ser, clock=clock)
            self.assertIn("var set pitch_down_gscale 1", ser.commands)
            self.assertEqual(ser.scale, .4)
            self.assertEqual(ser.commands[-1], "log off")

    def test_baseline_drift_is_reported_inconclusive(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            a = M.parser().parse_args(["--save", str(Path(tmp)/"drift.csv"), "--angles", "30"])
            clock = Clock()
            M.capture(a, serial_factory=lambda *x, **kw: FakeSerial(clock), clock=clock)
            with open(a.save, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            for row in rows:
                if row['stage'] == '2':
                    row['pid_mnm'] = str(int(row['pid_mnm'])+400)
                    row['total_mnm'] = row['pid_mnm']
            M.write_table(a.save, rows)
            result = M.analyze(a.save)
            self.assertFalse(result[0]['repeatable_baseline'])
            self.assertFalse(result[0]['comparison_valid'])


if __name__ == "__main__":
    unittest.main()
