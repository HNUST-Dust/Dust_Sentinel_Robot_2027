/**
 * @file trd_imu.cpp
 * @author qingyu
 * @brief IMU 线程 — 数据采集、加热控温、姿态解算
 * @version 0.1
 * @date 2026-11-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Imu"

#include "imu.hpp"
#include "Init_entry.hpp"

namespace thread::imu {

static ::imu::ImuManager imu_ {};

// 本颗 IMU 的静态零偏（gyro rad/s，accel m/s^2）
static constexpr ImuOffsetData kOffset {
    {  0.009880f,  0.011681f, -0.009188f },
    { -0.052499f,  0.056969f,  1.115934f }
};

// 姿态 EKF 参数
static constexpr alg::attitude::QuaternionEkf::Config kEkf 
{
    .Qq      		 = 10.0f,			// 四元数过程噪声
    .R       		 = 100.0f,			// 观测噪声，越大越信陀螺
    .alpha   		 = 0.016f,			// 加速度低通时间常数 s，约 10Hz
    .chi2_th 		 = 1.0e-6f,			// 卡方门控阈值，日志乘 1e9 后为 1000
    .recovery_time   = 1.0f,			// 连续严格静止 1 秒后允许倾角恢复
    .correction_rate = 1.0f,			// 加速度观测的姿态校正限速 deg/s
};

// 加热控温参数
static constexpr heater::Config kHeater 
{
    .target_temp = 40.0f,
    .duty_min    = 0.0f,
    .duty_max    = 0.95f,
	
    .pid  {
        .kp      = 0.19f,
        .ki      = 0.03f,
        .kd      = 0.0005f,
        .iOutMax = 0.5f,
        .outMax  = 0.95f,
        .dt      = 0.001f,
    },
};

/**
 * @brief 每拍回调：用户区自定义处理
 */
static void OnSample()
{
    const topic::imu_to::Message& m = imu_.GetMessage();

    DUST_LOG_DBG("imu_raw", "%f,%f", static_cast<double>(m.pitch), static_cast<double>(m.yaw));
}

bool thread_init()
{
	imu_.SetHook(OnSample);

    if (!imu_.Init(::imu::ImuStartMode::Normal, kOffset, kEkf, kHeater)) {
        return false;
    }
    return true;
}

bool thread_start()
{
    return imu_.Start(ThreadPrio::High);
}

REGISTER_INIT(thread_init, EarlyInit, High, HaltOnFail, nullptr, "imu_init");
REGISTER_THREAD(thread_start, EarlyThread, HaltOnFail, "imu_start");

} // namespace thread::imu
