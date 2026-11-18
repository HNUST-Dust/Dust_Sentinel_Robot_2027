/**
 * @file trd_yaw.cpp
 * @author qingyu
 * @brief 大小 yaw 达妙电机初始化与反馈接入
 * @version 0.3
 * @date 2026-11-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <zephyr/zbus/zbus.h>
#include "trd_yaw.hpp"
#include "trd_yaw_coop.hpp"
#include "dm.hpp"
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

static alg::pid::Pid big_pid_ {};										// 大 yaw 世界角环
static alg::pid::Pid small_pid_ {}; 									// 小 yaw IMU 世界角环
static alg::monitor::DtMeter dt_meter_ 	{};								// 控制周期测量

static topic::remote_to::Message remote_  {};							// 遥控最新状态，读失败保留旧值
static topic::imu_to::Message    imu_msg_ {};							// IMU 最新状态，读失败保留旧值

static constexpr uint32_t kLostCheckPeriod = 100;						// 掉线检测周期（Task 拍数）

static alg::monitor::StallWatch remote_watch_ {};						// 遥控停发检测（默认 100 拍）
static alg::monitor::StallWatch imu_watch_    {};						// IMU 停发检测（默认 100 拍）

static gimbal::LostWatch  big_motor_watch_  { kLostCheckPeriod };	// 电机掉线检测

static gimbal::LostWatch small_motor_watch_ { kLostCheckPeriod };
static bool g_set_zero_small = false;

static bool  g_set_zero_big = false;									// 大 yaw 设零请求，执行后自清
static Coop coop_ {};
static bool coop_ready_ = false; // 已用当前姿态初始化协同参考

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

        big_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        gimbal::SendCmd(&user_can1_msgq, kBigYawCanId, frame);
        small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);

        coop_ready_ = false;		// 重新开启时用当前姿态重建参考
        return true;
    }

    if (!imu_alive) {
        uint8_t frame[8];

        big_yaw_motor_.SetTargetTorque(0.0f);
        big_yaw_motor_.PackCtrlFrame(frame);
        gimbal::SendCmd(&user_can1_msgq, kBigYawCanId, frame);
        small_yaw_motor_.SetTargetTorque(0.0f);
        small_yaw_motor_.PackCtrlFrame(frame);
        gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);

        return true;
    }

    if (g_set_zero_big)
	{
        g_set_zero_big = false;
        coop_ready_ = false;
        gimbal::SaveZero(big_yaw_motor_, &user_can1_msgq, kBigYawCanId);
        return true;
    }

    if (g_set_zero_small) {
        g_set_zero_small = false;
        coop_ready_ = false;
        gimbal::SaveZero(small_yaw_motor_, &user_can0_msgq, kSmallYawCanId);
        return true;
    }

    if (!gimbal::Enabled(small_yaw_motor_)) {
        DUST_LOG_WRN("small yaw not enabled: err=%d", static_cast<int>(small_yaw_motor_.ReadAll().err));
        if (gimbal::Enable(small_yaw_motor_, &user_can0_msgq, kSmallYawCanId)) {
            DUST_LOG_INF("small yaw re-enabled");
        }
    }

    if (!gimbal::Enabled(big_yaw_motor_))
	{
        DUST_LOG_WRN("yaw not enabled: err=%d", static_cast<int>(big_yaw_motor_.ReadAll().err));
        if (gimbal::Enable(big_yaw_motor_, &user_can1_msgq, kBigYawCanId)) {
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

    const float big_joint   = WrapRad(-big_yaw_motor_.ReadAll().radian);
    const float small_joint = WrapRad(small_yaw_motor_.ReadAll().radian);
    // 相对偏角只由两轴关节角决定，不依赖 IMU 世界零位。整车关闭时也更新。
    gimbal::SetChassisRelativeYaw(WrapRad(-big_joint - small_joint));

    // 直接读话题：每拍取最新状态，不积压历史通知；读失败保留上一帧值
    zbus_chan_read(&pub_remote_to, &remote_,  K_NO_WAIT);
    zbus_chan_read(&pub_imu_to,    &imu_msg_, K_NO_WAIT);

    // 输入停更或整车关闭：处置交给 OnRetry
    const bool remote_lost = remote_watch_.Update(remote_.version);
    const bool imu_lost    = imu_watch_.Update(imu_msg_.version);

    if (remote_lost || imu_lost || remote_.robot_ctrl == topic::remote_to::StartMode::Off) {
        return TaskResult::Retry;
    }

    if (g_set_zero_big || g_set_zero_small ||
        !gimbal::Enabled(big_yaw_motor_) || !gimbal::Enabled(small_yaw_motor_)) {
        return TaskResult::Retry;
    }

    const bool big_lost   = big_motor_watch_.Update(big_yaw_motor_);
    const bool small_lost = small_motor_watch_.Update(small_yaw_motor_);

    if (big_lost || small_lost) {
        DUST_LOG_ERR("yaw lost");
        return TaskResult::Retry;
    }

    const float world_yaw   = WrapRad(imu_msg_.yaw * kDegToRad);

    if (!coop_ready_) {
        coop_.Init(big_joint, world_yaw);
        coop_ready_ = true;
    }

    // 直接使用遥控 yaw 输入积分世界目标
    float big_target = 0.0f, small_target = 0.0f, world_target = 0.0f;

    coop_.Update(remote_.yaw, world_yaw, big_joint, small_joint, dt, big_target, small_target, world_target);

    uint8_t frame[8];

	{
		// 小轴用 IMU yaw 跟踪世界目标，电机角度仅用于分配和限位
		const float small_err = WrapRad(world_yaw - world_target);
		float small_torque    = small_pid_.Calc(0.0f, small_err, dt);

		if (small_joint >= kSmallHardLimitRad) {
			small_torque = std::min(small_torque, 0.0f);
		} else if (small_joint <= -kSmallHardLimitRad) {
			small_torque = std::max(small_torque, 0.0f);
		}

		small_yaw_motor_.SetTargetTorque(small_torque);
		small_yaw_motor_.PackCtrlFrame(frame);
		gimbal::SendCmd(&user_can0_msgq, kSmallYawCanId, frame);
	}


    // IMU 总角扣除小轴实际角，大轴跟踪世界目标中分配给自己的部分
    const float big_world_yaw    = WrapRad(world_yaw - small_joint);
    const float big_world_target = WrapRad(world_target - small_target);
    const float big_err 		 = WrapRad(big_world_yaw - big_world_target);
    const float big_torque 		 = big_pid_.Calc(0.0f, big_err, dt);

    // 大 yaw 电机方向与 IMU yaw 正方向相反
    big_yaw_motor_.SetTargetTorque(-big_torque);
    big_yaw_motor_.PackCtrlFrame(frame);
    gimbal::SendCmd(&user_can1_msgq, kBigYawCanId, frame);

    DUST_LOG_DBG("yaw_pid", "%f,%f", static_cast<double>(WrapDeg(small_yaw_motor_.ReadAll().angle)), static_cast<double>(WrapDeg(big_yaw_motor_.ReadAll().angle)));

    return TaskResult::Continue;
}

static bool thread_init()
{
    alg::pid::Pid::Config pid_cfg {
        .outMax = kYawTorqueMax,
    };

    DmMotor::Config cfg {
        .ctrl_met      = ControlMethon::Mit,
        .gearbox_ratio = kGearRatio,			// J4310-2EC 内置减速箱 10:1，同步带 1:1
        .wheel_r       = 0.0f,          			// yaw 是旋转关节，无线速度换算
        .kp            = 0.0f,
        .kd            = 0.0f,
        .PMAX          = 12.5f,
        .VMAX          = 30.0f,
        .TMAX          = 10.0f,
    };

	{
		// 小 yaw IMU 世界角环
		pid_cfg.kp = 0.8f;
		pid_cfg.ki = 0.0f;
		pid_cfg.kd = 0.2f;
		small_pid_.Init(pid_cfg);
		cfg.can_id = kSmallYawCanId;
		cfg.master_id = kSmallYawMasterId;
		small_yaw_motor_.Init(cfg);
	}

	{
		// 大 yaw PID：位置环输出力矩
		pid_cfg.kp = 1.2f;
		pid_cfg.ki = 0.0f;
		pid_cfg.kd = 0.45f;
		big_pid_.Init(pid_cfg);

		cfg.can_id    = kBigYawCanId;
		cfg.master_id = kBigYawMasterId;
		big_yaw_motor_.Init(cfg);
	}

    DUST_LOG_INF("yaw ready: big CAN1 0x01/0x11, small CAN0 0x03/0x13");

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

CAN_RX_HANDLER(USER_RX_CAN1, kBigYawMasterId, [](uint8_t* data) { big_yaw_motor_.CanCpltRxCallback(data); });

CAN_RX_HANDLER(USER_RX_CAN0, kSmallYawMasterId, [](uint8_t* data) { small_yaw_motor_.CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("yaw_zero_small", g_set_zero_small);
REGISTER_SHELL_VAR("yaw_small_kp", small_pid_.GetKp());
REGISTER_SHELL_VAR("yaw_small_ki", small_pid_.GetKi());
REGISTER_SHELL_VAR("yaw_small_kd", small_pid_.GetKd());

REGISTER_SHELL_VAR("yaw_zero_big", g_set_zero_big);


REGISTER_SHELL_VAR("yaw_big_kp",   big_pid_.GetKp());
REGISTER_SHELL_VAR("yaw_big_ki",   big_pid_.GetKi());
REGISTER_SHELL_VAR("yaw_big_kd",   big_pid_.GetKd());

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "yaw_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "yaw_start");

} // namespace thread::yaw
