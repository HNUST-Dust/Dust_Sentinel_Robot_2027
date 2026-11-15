// Host regression: compile the production EKF with only the Sample I/O stubbed.
#include "quaternion.hpp"
#include "imu_device_layer.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>

static void Check(bool ok, const char* message)
{
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}

static float AngleDiff(float a, float b) { return std::remainder(a - b, 360.0f); }

static void CheckUnit(const alg::attitude::QuaternionEkf::State& state)
{
    float norm2 = 0.0f;
    for (float q : state.q) norm2 += q * q;
    Check(std::isfinite(norm2) && std::fabs(norm2 - 1.0f) < 1e-6f, "unit quaternion");
}

static void TestStillAfterYawRotation()
{
    alg::attitude::QuaternionEkf ekf;
    alg::attitude::QuaternionEkf::Config cfg;
    cfg.R = 100.0f;
    cfg.alpha = 0.016f;
    ekf.Init(cfg);
    Sample s {};
    s.dt = 0.002f;
    constexpr float pitch = 0.25f;
    s.accel[0] = -9.81f * std::sin(pitch);
    s.accel[2] = -9.81f * std::cos(pitch);
    ekf.Update(s);
    // Rotate about the navigation vertical with constant body gravity.
    for (int i = 0; i < 1000; ++i) {
        for (int j = 0; j < 3; ++j) s.gyro[j] = s.accel[j] / 9.81f * 0.7f;
        ekf.Update(s);
    }
    for (float& w : s.gyro) w = 0.0f;
    const float yaw = ekf.GetState().yaw;
    const float tilt = ekf.GetState().pitch;
    for (int i = 0; i < 5000; ++i) { ekf.Update(s); CheckUnit(ekf.GetState()); }
    const auto& st = ekf.GetState();
    std::printf("still 10s: yaw change %.6f deg, pitch change %.6f deg\n",
                AngleDiff(st.yaw, yaw), st.pitch - tilt);
    Check(std::fabs(AngleDiff(st.yaw, yaw)) < 0.01f, "stationary yaw stability after rotation");
    Check(std::fabs(st.pitch - tilt) < 0.01f, "stationary pitch stability");
    Check(st.chi2 >= 0.0f && st.chi2 < cfg.chi2_th, "stationary innovation accepted");
}

static void TestRejectionAndRecovery()
{
    alg::attitude::QuaternionEkf ekf;
    alg::attitude::QuaternionEkf::Config cfg;
    cfg.R = 100.0f;
    ekf.Init(cfg);
    Sample s {};
    s.dt = 0.002f;
    s.accel[2] = -9.81f;
    ekf.Update(s);
    const auto before = ekf.GetState();
    s.accel[0] = 4.0f;
    s.accel[2] = -8.96f;
    ekf.Update(s);
    const auto rejected = ekf.GetState();
    Check(rejected.chi2 > cfg.chi2_th, "rejected raw chi2 retained");
    Check(rejected.k_scale == 0.0f && rejected.err_cnt == 1, "rejection diagnostics");
    for (int j = 0; j < 4; ++j)
        Check(std::fabs(before.q[j] - rejected.q[j]) < 1e-6f, "rejected correction actually rolled back");
    s.accel[0] = 0.0f;
    s.accel[2] = -9.81f;
    ekf.Update(s);
    Check(ekf.GetState().k_scale > 0.0f && ekf.GetState().err_cnt == 0, "recovery after rejection");
}

static void TestPitchSweep()
{
    alg::attitude::QuaternionEkf ekf;
    alg::attitude::QuaternionEkf::Config cfg;
    cfg.R = 100.0f;
    ekf.Init(cfg);
    Sample s {};
    s.dt = 0.002f;
    s.accel[2] = -9.81f;
    ekf.Update(s);
    float max_error = 0.0f;
    for (int i = 1; i <= 10000; ++i) {
        const float t = i * s.dt;
        const float pitch = 0.4f * std::sin(2.0f * t);
        s.accel[0] = -9.81f * std::sin(pitch);
        s.accel[2] = -9.81f * std::cos(pitch);
        s.gyro[1] = -0.8f * std::cos(2.0f * (t - s.dt * 0.5f));
        ekf.Update(s);
        CheckUnit(ekf.GetState());
        max_error = std::fmax(max_error, std::fabs(ekf.GetState().pitch - pitch * 57.295779513f));
    }
    std::printf("pitch sweep 20s: max error %.6f deg\n", max_error);
    Check(max_error < 0.02f, "pitch sweep tracks gyro and gravity");
}

static void TestEngineFallback()
{
    using Filter = alg::filter::ExtendedKalman<1, 1, 1>;
    Filter f;
    f.SetSystemFunc([](const Filter::State& x, const Filter::Ctrl& u,
                       Filter::State& out, Filter::Cov& F) { out = x + u; F.setIdentity(); });
    f.SetObserveFunc([](const Filter::State& x, Filter::Obs& z, Filter::ObsMat& H) {
        z = x; H.setIdentity();
    });
    f.Init(Filter::Cov::Identity(), Filter::ObsCov::Identity());
    f.SetState(Filter::State::Constant(2.0f));
    f.Predict(Filter::Ctrl::Constant(1.0f));
    const auto predicted_cov = f.PMinus().eval();
    f.Update(Filter::Obs::Constant(10.0f));
    f.SetPosteriorState(Filter::State::Constant(99.0f));
    f.FallbackToPrediction();
    Check(f.GetX()(0) == 3.0f, "posterior write preserves prediction");
    Check(f.GetP().isApprox(predicted_cov), "covariance rollback");
    f.Init(Filter::Cov::Zero(), Filter::ObsCov::Constant(-10.0f));
    f.SetState(Filter::State::Constant(2.0f));
    f.Predict(Filter::Ctrl::Constant(1.0f));
    f.Update(Filter::Obs::Zero());
    Check(f.GetX()(0) == 3.0f && f.GetChi2() == -1.0f, "failed solve retains prediction and flags chi2");
}

static void TestStationaryRecoveryRate()
{
    alg::attitude::QuaternionEkf ekf;
    alg::attitude::QuaternionEkf::Config cfg;
    cfg.R = 100.0f;
    cfg.chi2_th = 1e-6f;
    ekf.Init(cfg);
    Sample s {};
    s.dt = 0.002f;
    s.accel[2] = -9.81f;
    ekf.Update(s);
    // A 4 degree prediction/gravity mismatch must recover without waiting
    // for covariance inflation to open the gate, or jumping on acceptance.
    constexpr float target_deg = 4.0f;
    const float target = target_deg / 57.295779513f;
    s.accel[0] = -9.81f * std::sin(target);
    s.accel[2] = -9.81f * std::cos(target);
    float first_accept = -1.0f;
    float max_rate = 0.0f;
    for (int i = 1; i <= 5000; ++i) {
        const float before = ekf.GetState().pitch;
        ekf.Update(s);
        const auto& st = ekf.GetState();
        CheckUnit(st);
        if (st.k_scale > 0.0f && first_accept < 0.0f) first_accept = i * s.dt;
        max_rate = std::fmax(max_rate, std::fabs(st.pitch - before) / s.dt);
        if (i * s.dt < 0.9f) Check(st.k_scale == 0.0f, "no early stationary bypass");
    }
    std::printf("4 deg static recovery: first correction %.3fs, max rate %.4f deg/s, final error %.6f deg\n",
                first_accept, max_rate, ekf.GetState().pitch - target_deg);
    Check(first_accept >= 0.99f && first_accept < 1.1f, "stationary recovery starts after dwell");
    Check(max_rate <= cfg.correction_rate * 1.01f, "bounded correction rate");
    Check(std::fabs(ekf.GetState().pitch - target_deg) < 0.01f, "stationary gravity convergence");
}

static void TestCurrentFrameGain()
{
    using Filter = alg::filter::ExtendedKalman<1, 1, 1>;
    Filter f;
    f.SetSystemFunc([](const Filter::State& x, const Filter::Ctrl&,
                       Filter::State& out, Filter::Cov& F) { out = x; F.setIdentity(); });
    f.SetObserveFunc([](const Filter::State& x, Filter::Obs& z, Filter::ObsMat& H) {
        z = x; H.setIdentity();
    });
    f.Init(Filter::Cov::Identity(), Filter::ObsCov::Identity());
    f.Predict(Filter::Ctrl::Zero());
    f.SetZ(Filter::Obs::Constant(10.0f));
    Check(!f.UpdateWithGate(1.0f, true), "current-frame outlier rejected before update");
    Check(f.GetX()(0) == 0.0f && f.GetP()(0) == 2.0f, "rejected state and covariance untouched");
    f.SetGainScale(1.0f);
    f.SetZ(Filter::Obs::Constant(std::sqrt(2.7f)));
    Check(f.UpdateWithGate(1.0f, true, 0.01f), "current-frame soft gate accepted");
    Check(std::fabs(f.GetX()(0) - 0.01f) < 1e-6f, "same-frame gain and step cap");
    const float effective_k = 0.01f / std::sqrt(2.7f);
    const float expected_p = 2.0f * (1.0f - effective_k) * (1.0f - effective_k) + effective_k * effective_k;
    Check(std::fabs(f.GetP()(0) - expected_p) < 1e-6f, "covariance uses actual capped gain");
}

int main()
{
    TestEngineFallback();
    TestCurrentFrameGain();
    TestStillAfterYawRotation();
    TestRejectionAndRecovery();
    TestPitchSweep();
    TestStationaryRecoveryRate();
    std::puts("All EKF regressions passed");
}
