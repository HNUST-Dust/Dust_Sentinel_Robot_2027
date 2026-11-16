"""Host regression of chassis PI helpers extracted unchanged from production."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PID = ROOT.parent.parent / "zephyr_user/framework/algorithm/controller/pid"


class ChassisPiTests(unittest.TestCase):
    def test_controller(self):
        compiler = shutil.which("g++")
        if not compiler:
            self.skipTest("g++ required")
        header = (ROOT / "project/thread/chassis/trd_chassis.hpp").read_text(encoding="utf-8")
        source = (ROOT / "project/thread/chassis/trd_chassis.cpp").read_text(encoding="utf-8")
        constants = header[header.index("// 摩擦前馈"):header.index("inline motor::dji")]
        helpers = source[source.index("static float FrictionFeedforward"):source.index("/**\n * @brief 任务单拍")]
        gains = source[source.index("\t\tconst float b ="):source.index("\n\n\t\tchassis_motor_[i].Init")]
        prefix = r'''
#include "pid.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
constexpr uint8_t N_Wheel = 4;
static alg::pid::Pid pid_[N_Wheel] {};
static float last_target_[N_Wheel] {};
static bool target_valid_[N_Wheel] {};
static void Check(bool ok, const char* msg) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", msg); std::exit(1); }
}
'''
        init = '''
static void Init() {
    for (uint8_t i = 0; i < N_Wheel; ++i) {
        alg::pid::Pid::Config pid_cfg {};
        pid_cfg.outMax = kControlCurrentMax;
        pid_cfg.iOutMax = kIntegralCurrentMax;
''' + gains + '''
        pid_[i].Init(pid_cfg);
        ResetSpeedControl(i);
    }
}
'''
        cases = r'''
int main() {
    Init();
    // With PI itself unsaturated, inertial FF pushes the SUM into saturation.
    SpeedCurrent(0, 6.0f, 6.0f, 0.001f);
    const float previous = pid_[0].GetIntegralError();
    const float saturated = SpeedCurrent(0, 7.0f, 4.0f, 0.001f);
    Check(saturated == kControlCurrentMax, "sum-current clamp");
    Check(pid_[0].GetIntegralError() == previous, "anti-windup includes feedforward");
    Check(SpeedCurrent(0, -100.0f, 0.0f, 0.001f) >= -kControlCurrentMax, "negative current clamp");
    ResetSpeedControl(0);
    Check(pid_[0].GetIntegralError() == 0.0f && !target_valid_[0], "stop clears PI and acceleration history");

    // Equal elapsed times at different sample periods must give equal integrals.
    Init();
    for (int i = 0; i < 100; ++i) SpeedCurrent(0, 1.0f, 0.0f, 0.001f);
    const float fast = pid_[0].GetIntegralError();
    ResetSpeedControl(0);
    for (int i = 0; i < 25; ++i) SpeedCurrent(0, 1.0f, 0.0f, 0.004f);
    Check(std::fabs(fast - pid_[0].GetIntegralError()) < 1e-6f, "actual dt used in integral");

    // Declared motor model only: this is not a hardware response measurement.
    for (int wheel = 0; wheel < N_Wheel; ++wheel) {
        for (float direction : {-1.0f, 1.0f}) {
            Init();
            float omega = 0.0f;
            const float target = direction * 10.0f;
            for (int i = 0; i < 5000; ++i) {
                const float dt = (i % 2) ? 0.001f : 0.003f;
                const float current = SpeedCurrent(wheel, target, omega, dt);
                const float load = (i >= 1500) ? direction * 0.3f : 0.0f;
                const float friction = FrictionFeedforward(wheel, omega, omega);
                omega += (current - friction - load) / kJEq[wheel] * dt;
                Check(std::isfinite(omega) && std::fabs(current) <= kControlCurrentMax,
                      "finite model trajectory and current bound");
            }
            std::printf("wheel%d direction %+.0f model final error %+.6f rad/s\n", wheel, direction, target - omega);
            Check(std::fabs(target - omega) < 0.01f, "model load-disturbance rejection");
        }
    }
    std::puts("All chassis PI regressions passed (model/host only)");
}
'''
        with tempfile.TemporaryDirectory(prefix="chassis-pi-") as directory:
            temp = Path(directory)
            cpp = temp / "test.cpp"
            cpp.write_text(prefix + constants + helpers + init + cases, encoding="utf-8")
            executable = temp / "test.exe"
            subprocess.run([compiler, "-std=c++17", "-O2", "-Wall", "-Wextra",
                            "-I", str(PID), str(cpp), str(PID / "pid.cpp"),
                            "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
