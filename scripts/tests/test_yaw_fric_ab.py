"""摩擦前馈 A/B 脚本：假串口跑完整采集与对比。不会操作硬件。"""
import contextlib
import csv
import importlib.util
import io
import math
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "yaw_fric_ab", Path(__file__).resolve().parents[1] / "yaw_fric_ab.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)

DT = 0.003
TAU_OFF = 0.25          # 关补偿时的一阶时间常数 s
TAU_ON = 0.10           # 开补偿时更快


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class FakeSerial:
    """一阶被控对象：world 追 target；开补偿时间常数更小、静差更小。"""

    def __init__(self, clock):
        self.clock = clock
        self.queue = []
        self.commands = []
        self.target = 0.0
        self.world = 0.0
        self.fric = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def reset_input_buffer(self):
        self.queue.clear()

    def flush(self):
        pass

    def write(self, raw):
        command = raw.decode().strip()
        self.commands.append(command)
        if command == "log off":
            self.queue.append("log off: ok")
        elif command.startswith("var set yaw_small_target"):
            self.target = float(command.split()[-1])
            self.queue.append("ok")
        elif command.startswith("var set yaw_fric_enable"):
            self.fric = int(command.split()[-1])
            self.queue.append("ok")
        elif command.startswith("var set yaw_trace_enable"):
            self.queue.append("ok")
        elif command.startswith("var set yaw_test_"):
            self.queue.append("ok")
        else:
            raise AssertionError(f"unexpected command: {command}")

    def readline(self):
        self.clock.now += DT
        if self.queue:
            return (self.queue.pop(0) + "\r\n").encode()

        tau = TAU_ON if self.fric else TAU_OFF
        self.world += (self.target - self.world) * DT / tau

        line = (f"YSTEP,{int(self.clock.now * 1000)},{int(self.target * 1000)},"
                f"{int(self.world * 1000)},{int(self.world * 1000)},"
                f"{int((self.target - self.world) * 1000)},{self.fric}")
        return (line + "\r\n").encode()


class FricAbTests(unittest.TestCase):
    def run_capture(self, tmp):
        clock = Clock()
        ser = FakeSerial(clock)
        args = M.parser().parse_args(["--save", str(Path(tmp) / "ab.csv"), "--hold", "1.5"])
        M.capture(args, serial_factory=lambda *a, **kw: ser, clock=clock)
        return args, ser

    def test_capture_writes_both_phases(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            args, ser = self.run_capture(tmp)
            with Path(args.save).open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

            self.assertTrue(rows)
            self.assertEqual(list(rows[0]), M.FIELDS)
            self.assertEqual({row["phase"] for row in rows}, {"off", "on"})
            self.assertEqual(ser.commands[-1], "log off")
            self.assertIn("var set yaw_fric_enable 0", ser.commands)
            self.assertIn("var set yaw_fric_enable 1", ser.commands)

    def test_analyze_shows_improvement(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            args, _ = self.run_capture(tmp)
            result = M.analyze(args.save)

            self.assertIn("off", result)
            self.assertIn("on", result)
            self.assertLess(result["on"]["rise"], result["off"]["rise"])
            self.assertLess(result["on"]["steady"], result["off"]["steady"])


if __name__ == "__main__":
    unittest.main()
