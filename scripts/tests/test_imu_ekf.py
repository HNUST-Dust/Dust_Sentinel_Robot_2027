"""Compile/run the production C++ EKF on the host; no board I/O or flashing."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
FRAMEWORK = Path(os.environ.get("IMU_TEST_FRAMEWORK", ROOT.parent.parent / "zephyr_user/framework"))


class EkfRegressionTests(unittest.TestCase):
    def test_production_cpp(self):
        compiler = shutil.which("g++")
        if not compiler:
            self.skipTest("g++ is required for host C++ regression")
        with tempfile.TemporaryDirectory(prefix="imu-ekf-") as directory:
            temp = Path(directory)
            (temp / "imu_device_layer.hpp").write_text(
                "#pragma once\nstruct Sample { float gyro[3]{}; float accel[3]{}; "
                "float temp{}; float dt = 0.001f; };\n", encoding="utf-8")
            executable = temp / "imu_ekf_test.exe"
            subprocess.run([
                compiler, "-std=c++17", "-O2", "-Wall", "-Wextra",
                "-I", str(temp),
                "-I", str(FRAMEWORK / "algorithm/math/eigen"),
                "-I", str(FRAMEWORK / "algorithm/filter/kalman"),
                "-I", str(FRAMEWORK / "algorithm/filter/quaternion"),
                str(Path(__file__).with_name("imu_ekf_regression.cpp")),
                str(FRAMEWORK / "algorithm/filter/quaternion/quaternion.cpp"),
                "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True)


class StationaryTailTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("pitch_imu_log", ROOT / "scripts/pitch_imu_log.py")
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_motion_at_start_excluded(self):
        import numpy as np
        t = np.arange(0.0, 20.0, 0.1)
        motor = np.where(t < 4.0, t, 4.0)
        w = np.where(t < 4.0, 1000.0, 1.0)
        stable = (t >= 4.0).astype(int)
        sel = self.module.stationary_tail(t, np.ones(len(t)), motor, w, stable)
        self.assertGreaterEqual(t[sel][0], 6.0)
        self.assertEqual(t[sel][-1], t[-1])

    def test_slow_motor_motion_not_reported_as_drift(self):
        import numpy as np
        t = np.arange(0.0, 20.0, 0.1)
        sel = self.module.stationary_tail(t, np.ones(len(t)), t * 0.1,
                                          np.ones(len(t)), np.ones(len(t)))
        self.assertFalse(sel.any())


if __name__ == "__main__":
    unittest.main()
