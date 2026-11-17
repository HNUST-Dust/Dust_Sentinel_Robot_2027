/**
 * @file trd_yaw.cpp
 * @author qingyu
 * @brief 小 yaw 达妙电机初始化与反馈接入
 * @version 0.3
 * @date 2026-11-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cmath>
#include <zephyr/zbus/zbus.h>
#include "trd_yaw.hpp"
#include "gimbal.hpp"
#include "dt_meter.hpp"
#include "stall_watch.hpp"
#include "to_can_tx.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "remote_to.hpp"
#include "imu_to.hpp"
#include "angle.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::yaw {

using namespace instance;
using namespace instance::yaw;
using namespace alg::math;

static Thread<2048> thread_{};

static alg::pid::Pid   small_pid_ {};							// 小 yaw 世界角环
static alg::monitor::DtMeter dt_meter_ {};						// 控制周期测量

static topic::remote_to::Message remote_  {};					// 遥控最新状态，读失败保留旧值
static topic::imu_to::Message    imu_msg_ {};					// IMU 最新状态，读失败保留旧值

static constexpr uint32_t kLostCheckPeriod = 100;				// 掉线检测周期（Task 拍数）

static alg::monitor::StallWatch remote_watch_ {};				// 遥控停发检测（默认 100 拍）
static alg::monitor::StallWatch imu_watch_    {};				// IMU 停发检测（默认 100 拍）
static gimbal::LostWatch  motor_watch_  { kLostCheckPeriod };	// 电机掉线检测

static bool  g_set_zero_small = false;							// 小 yaw 设零请求，执行后自清
static float target_deg_      = 0.0f;							// 小 yaw 世界目标角（度），由 var 设定
static float world_ref_       = 0.0f;							// 遥控积分的世界目标角 rad
static bool  world_ref_ready_ = false;							// 已用当前姿态初始化世界目标

/**
 * @brief 摩擦前馈力矩：库仑摩擦 + 静摩擦，结构同底盘 FrictionFeedforward
 *
 * 方向由误差 tanh 平滑（零位附近自动过渡，不靠死区）；
 * 静摩擦按实际转速平方指数衰减回库仑值，真正停住时才拿满静摩擦。
 *
 * @param error 世界角误差 rad（当前 - 目标）
 * @param omega 关节实际角速度 rad/s
 * @return 前馈力矩 N*m
 */
static float FrictionFeedforward(float error, float omega)
{
    // 方向：误差取反后 tanh 平滑，误差为负表示需要正向运动
    const float s        = std::tanh(-error / (kFricDirSmoothDeg * kDegToRad));
    const bool  forward  = (s >= 0.0f);
    const float a        = forward ? kFricCoulombPos : kFricCoulombNeg;
    const float a_static = forward ? kFricStaticPos  : kFricStaticNeg;

    // 静摩擦随实际转速平方指数衰减回库仑值
    const float r     = omega / kFricSpeedSmooth;
    const float a_eff = a + (a_static - a) * std::exp(-r * r);

    return a_eff * s;
}

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

    const bool remote_alive = !remote_watch_.Timeout();
    const bool imu_alive    = !imu_watch_.Timeout();

    if (!remote_alive || remote_.robot_ctrl == topic::remote_to::StartMode::Off)
	{
        uint8_t frame[8];

        small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);

        world_ref_ready_ = false;		// 重新开启时用当前姿态重建参考
        return true;
    }

    if (!imu_alive) {
        uint8_t frame[8];

        small_yaw_motor_.SetTargetTorque(0.0f);
        small_yaw_motor_.PackCtrlFrame(frame);
        gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);

        return true;
    }

    if (g_set_zero_small)
	{
        g_set_zero_small = false;
        gimbal::SaveZero(small_yaw_motor_, &user_can0_msgq, kSmallYawCanId);
        return true;
    }

    if (!gimbal::Enabled(small_yaw_motor_))
	{
        DUST_LOG_WRN("yaw not enabled: err=%d", static_cast<int>(small_yaw_motor_.ReadAll().err));
        if (gimbal::Enable(small_yaw_motor_, &user_can0_msgq, kSmallYawCanId)) {
            DUST_LOG_INF("yaw re-enabled");
        }
    }

    return true;
}

/**
 * @brief 任务单拍：判断设零标志位和使能状态，异常转 Retry 交 OnRetry 处理
 * @param context 未使用
 * @return Continue=正常；Retry=设零请求或未使能
 */
static TaskResult Task(void* context)
{
    (void)context;

    const float dt = dt_meter_.Update();

    // 直接读话题：每拍取最新状态，不积压历史通知；读失败保留上一帧值
    zbus_chan_read(&pub_remote_to, &remote_,  K_NO_WAIT);
    zbus_chan_read(&pub_imu_to,    &imu_msg_, K_NO_WAIT);

    // 输入停更或整车关闭：处置交给 OnRetry
    const bool remote_lost = remote_watch_.Update(remote_.version);
    const bool imu_lost    = imu_watch_.Update(imu_msg_.version);

    if (remote_lost || imu_lost || remote_.robot_ctrl == topic::remote_to::StartMode::Off) {
        return TaskResult::Retry;
    }

    if (g_set_zero_small || !gimbal::Enabled(small_yaw_motor_)) {
        return TaskResult::Retry;
    }

    if (motor_watch_.Update(small_yaw_motor_)) {
        DUST_LOG_ERR("yaw lost");
        return TaskResult::Retry;
    }

    float world_yaw = 0.0f, world_target = 0.0f, small_torque = 0.0f;

    // 关节角到 ±45° 软限时只停往外推的力矩，不反向拉回
    const DmMotor::Snapshot small = small_yaw_motor_.ReadAll();
    const float small_deg = WrapDeg(small.angle + kSmallAngleOffsetDeg);

    world_yaw = WrapRad(imu_msg_.yaw * kDegToRad); // IMU 发布单位为度，转为 rad

    // 首拍用当前世界角建立参考
    if (!world_ref_ready_) {
        world_ref_ = world_yaw;
        world_ref_ready_ = true;
    }

    // custom1=On 用 var 目标角；Off 用遥控积分世界目标角
    if (remote_.custom1 == topic::remote_to::StartMode::On) {
        world_target = WrapRad(target_deg_ * kDegToRad);
    } else {
        world_ref_   = WrapRad(world_ref_ + remote_.yaw * kYawRateLimitRad * dt);
        world_target = world_ref_;
    }

    // 小轴闭环世界角误差，PID 目标恒为 0
    const float small_err = WrapRad(world_yaw - world_target);

    small_torque  = small_pid_.Calc(world_target, small_err, dt);

    small_torque += FrictionFeedforward(small_err, small.omega);

    if (small_deg >= kSmallSoftLimitDeg) {
        small_torque = std::min(small_torque, 0.0f);
    } else if (small_deg <= -kSmallSoftLimitDeg) {
        small_torque = std::max(small_torque, 0.0f);
    }

    uint8_t frame[8];

    small_yaw_motor_.SetTargetTorque(small_torque);
    small_yaw_motor_.PackCtrlFrame(frame);
    gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);

    DUST_LOG_DBG("yaw_pid", "%f,%f,%f", static_cast<double>(world_target * kRadToDeg), static_cast<double>(world_yaw * kRadToDeg), static_cast<double>(small_deg));

    return TaskResult::Continue;
}

static bool thread_init()
{
    DmMotor::Config cfg{};
    cfg.ctrl_met = ControlMethon::Mit;
    cfg.gearbox_ratio = kGearRatio;			// J4310-2EC 内置减速箱 10:1，同步带 1:1
    cfg.wheel_r = 0.0f;         			// yaw 是旋转关节，无线速度换算

    cfg.kp = 0.0f;
    cfg.kd = 0.0f;

    // 暂按上位机截图配置，电机的实际读回量程需与此一致。
    cfg.PMAX = 12.5f;
    cfg.VMAX = 30.0f;
    cfg.TMAX = 10.0f;

    cfg.can_id    = kSmallYawCanId;
    cfg.master_id = kSmallYawMasterId;
    small_yaw_motor_.Init(cfg);

    // 小 yaw PID：位置环输出力矩
    alg::pid::Pid::Config pid_cfg {};

    pid_cfg.kp     = 1.0f;
    pid_cfg.ki     = 0.0f;
    pid_cfg.kd     = 0.0f;
    pid_cfg.outMax = kYawTorqueMax;

    small_pid_.Init(pid_cfg);

    DUST_LOG_INF("yaw ready: small CAN0 0x03/0x13");

    return true;
}

/**
 * @brief 启动 yaw 线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "yaw", ThreadFaultConfig{ OnRetry, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

CAN_RX_HANDLER(USER_RX_CAN0, kSmallYawMasterId, [](uint8_t* data) { small_yaw_motor_.CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("yaw_zero_small", g_set_zero_small);

REGISTER_SHELL_VAR("yaw_small_target", target_deg_);

REGISTER_SHELL_VAR("yaw_small_kp",     small_pid_.GetKp());
REGISTER_SHELL_VAR("yaw_small_ki",     small_pid_.GetKi());
REGISTER_SHELL_VAR("yaw_small_kd",     small_pid_.GetKd());

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "yaw_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "yaw_start");

} // namespace thread::yaw
