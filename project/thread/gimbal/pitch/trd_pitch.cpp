/**
 * @file trd_pitch.cpp
 * @author qingyu
 * @brief 双 pitch 达妙电机协同控制
 * @version 0.5
 * @date 2026-11-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/zbus/zbus.h>
#include <algorithm>
#include <cmath>
#include "trd_pitch.hpp"
#include "dm.hpp"
#include "gimbal.hpp"
#include "dt_meter.hpp"
#include "stall_watch.hpp"
#include "to_can_tx.hpp"
#include "remote_to.hpp"
#include "imu_to.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "angle.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::pitch {

using namespace instance;
using namespace instance::pitch;
using namespace alg::math;

static Thread<2048> thread_{};

static alg::monitor::DtMeter     dt_meter_ {};										// 控制周期测量

static topic::remote_to::Message remote_   {};										// 遥控最新状态，读失败保留旧值
static topic::imu_to::Message    imu_msg_  {};										// IMU 最新状态，读失败保留旧值

static alg::monitor::StallWatch  remote_watch_ {};									// 遥控停发检测（默认 100 拍）
static alg::monitor::StallWatch  imu_watch_    {};									// IMU 停发检测（默认 100 拍）

static constexpr uint32_t kLostCheckPeriod = 100;

#if defined(CONFIG_TRD_PITCH_UP)

static float up_target_deg_  = 0.0f;												// 上 pitch 目标角（度），通过 var 设定
static bool  g_set_zero_up_  = false;												// 上 pitch 设零请求，执行后自清

static alg::pid::Pid up_pid_     {};												// 上 pitch 位置环
static gimbal::LostWatch up_motor_watch_   { kLostCheckPeriod };				// 上 pitch 电机掉线检测

#endif

#if defined(CONFIG_TRD_PITCH_DOWN)

static float down_target_deg_ = 45.0f;												// 下 pitch 目标角（度），通过 var 设定
static bool  g_set_zero_down_ = false;												// 下 pitch 设零请求，执行后自清

static alg::pid::Pid down_pid_   {};												// 下 pitch 位置环
static gimbal::LostWatch down_motor_watch_ { kLostCheckPeriod };			   // 下 pitch 电机掉线检测

/**
 * @brief 下 pitch 五次多项式目标角曲线状态
 */
struct {
	float position = 0.0f;
	float velocity = 0.0f;
	float acceleration = 0.0f;
	float target = 0.0f;
	float elapsed = 2.0f;
	float coefficient[6] {};
	bool initialized = false;
} down_target_curve_;

/**
 * @brief 以 2 秒五次多项式平滑下 pitch 目标角，重规划时保持参考角及其一、二阶导数连续
 * @param target 目标角，单位 rad
 * @param now 当前电机角，单位 rad，首次运行时作为曲线起点
 * @param dt 本拍时间，单位 s
 * @return 平滑后的参考角，单位 rad
 */
static float SmoothDownTarget(float target, float now, float dt)
{
	constexpr float duration = 2.0f;
	auto& curve = down_target_curve_;
	auto& c = curve.coefficient;
	
	if (!curve.initialized)
	{
		curve.position = now;
		curve.target = now;
		c[0] = now;
		curve.initialized = true;
	}

	if (target != curve.target)
	{
		const float delta        = target - curve.position;
		const float velocity     = curve.velocity * duration;
		const float acceleration = curve.acceleration * duration * duration;
		c[0] = curve.position;
		c[1] = velocity;
		c[2] =  0.5f  * acceleration;
		c[3] =  10.0f * delta - 6.0f * velocity - 1.5f * acceleration;
		c[4] = -15.0f * delta + 8.0f * velocity + 1.5f * acceleration;
		c[5] =  6.0f  * delta - 3.0f * velocity - 0.5f * acceleration;
		curve.target = target;
		curve.elapsed = 0.0f;
	}

	curve.elapsed = std::min(curve.elapsed + dt, duration);
	if (curve.elapsed >= duration)
	{
		curve.position = curve.target;
		curve.velocity = 0.0f;
		curve.acceleration = 0.0f;
		return curve.position;
	}

	const float u      = curve.elapsed / duration;
	curve.position     = c[0]  + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
	curve.velocity     = (c[1] + u * (2.0f * c[2] + u * (3.0f * c[3] + u * (4.0f * c[4] + u * 5.0f * c[5])))) / duration;
	curve.acceleration = (2.0f * c[2] + u * (6.0f * c[3] + u * (12.0f * c[4] + u * 20.0f * c[5]))) / (duration * duration);
	return curve.position;
}

#endif

#if defined(CONFIG_TRD_PITCH_UP)

/**
 * @brief 上 pitch 重力前馈力矩
 * @param pitch_rad 电机反馈角度，单位 rad
 * @return 补偿力矩，单位 N*m
 */
static float GravityTorque(float pitch_rad)
{
    if (!std::isfinite(pitch_rad)) return 0.0f;
    return 0.041619f * std::sin(pitch_rad) + 0.315195f * std::cos(pitch_rad);
}

#endif

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

	#if defined(CONFIG_TRD_PITCH_UP)
	{
		up_pid_.Reset();
	}
	#endif

	#if defined(CONFIG_TRD_PITCH_DOWN)
	{
		down_pid_.Reset();
		down_target_curve_ = {};
	}
	#endif

    uint8_t frame[8];
    // SA 上档或遥控停更时优先失能，禁止进入自动使能分支。
    if (remote_watch_.Timeout() || remote_.robot_ctrl == topic::remote_to::StartMode::Off)
	{
		#if defined(CONFIG_TRD_PITCH_DOWN)
		{
			down_pitch_motor_.SetTargetTorque(0.0f);
			down_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
			gimbal::SendCmd(&user_can0_msgq, kDownPitchCanId, frame);
		}
		#endif

		#if defined(CONFIG_TRD_PITCH_UP)
		{
			up_pitch_motor_.SetTargetTorque(0.0f);
			up_pitch_motor_.PackCmdFrame(frame,   DmMotor::Cmd::Disable);
			gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);
		}
		#endif

		if (remote_.robot_ctrl != topic::remote_to::StartMode::Off) {
            DUST_LOG_WRN("pitch disable: stall=%u ctrl=%d ver=%u", static_cast<unsigned>(remote_watch_.Stall()), static_cast<int>(remote_.robot_ctrl), static_cast<unsigned>(remote_.version));
        }

        return true;
    }

	#if defined(CONFIG_TRD_PITCH_UP)
	{
		// IMU 停更：上 pitch 失去反馈，只发零力矩，下 pitch 不受影响。
		if (imu_watch_.Timeout()) {
			up_pitch_motor_.SetTargetTorque(0.0f);
			up_pitch_motor_.PackCtrlFrame(frame);
			gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);
		}
	}
	#endif

	#if defined(CONFIG_TRD_PITCH_DOWN)
	{
		if (g_set_zero_down_) {
			g_set_zero_down_ = false;
			gimbal::SaveZero(down_pitch_motor_, &user_can0_msgq, kDownPitchCanId);
		}

		if (!gimbal::Enabled(down_pitch_motor_))
		{
			DUST_LOG_WRN("pitch not enabled: err=%d", static_cast<int>(down_pitch_motor_.ReadAll().err));
			if (gimbal::Enable(down_pitch_motor_, &user_can0_msgq, kDownPitchCanId)) {
				DUST_LOG_INF("pitch re-enabled");
			}
		}
	}
	#endif

	#if defined(CONFIG_TRD_PITCH_UP)
	{
		if (g_set_zero_up_) {
			g_set_zero_up_ = false;
			gimbal::SaveZero(up_pitch_motor_, &user_can0_msgq, kUpPitchCanId);
		}

		if (!gimbal::Enabled(up_pitch_motor_))
		{
			DUST_LOG_WRN("pitch not enabled: err=%d", static_cast<int>(up_pitch_motor_.ReadAll().err));
			if (gimbal::Enable(up_pitch_motor_, &user_can0_msgq, kUpPitchCanId)) {
				DUST_LOG_INF("pitch re-enabled");
			}
		}
	}
	#endif

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

    zbus_chan_read(&pub_remote_to, &remote_,  K_NO_WAIT);
	zbus_chan_read(&pub_imu_to,    &imu_msg_, K_NO_WAIT);

    const bool remote_lost = remote_watch_.Update(remote_.version);
	const bool imu_lost    = imu_watch_.Update(imu_msg_.version);

    if (remote_lost || remote_.robot_ctrl == topic::remote_to::StartMode::Off) {
        return TaskResult::Retry;
    }

    uint8_t frame[8];

	#if defined(CONFIG_TRD_PITCH_UP)
	{
		// up pitch
		if (g_set_zero_up_   || !gimbal::Enabled(up_pitch_motor_)) {
			return TaskResult::Retry;
		}

		if (up_motor_watch_.Update(up_pitch_motor_)) {
			DUST_LOG_ERR("up pitch lost");
			return TaskResult::Retry;
		}

		// 上 pitch：IMU 角闭环 + 重力前馈；IMU 停更时只发零力矩，本拍结束。
		if (imu_lost)
		{
			up_pitch_motor_.SetTargetTorque(0.0f);
			up_pitch_motor_.PackCtrlFrame(frame);
			gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);

			return TaskResult::Continue;
		}

		const float up_now_rad	   = imu_msg_.pitch * kDegToRad;					// IMU 发布单位为度，取反统一方向后转 rad
		// custom1=On 用 var 目标角；Off 用遥控 [-1,1] 线性映射到机械限位
		const float up_target_rad  = up_target_deg_  * kDegToRad;
		const float up_err 	   	   = WrapRad(up_now_rad - up_target_rad);

		const float up_pid_torque  = up_pid_.Calc(0.0f, up_err, dt);
		const float gravity_torque = GravityTorque(up_now_rad);
		const float up_torque 	   = std::clamp(up_pid_torque + gravity_torque, -kPitchTorqueMax, kPitchTorqueMax);

		up_pitch_motor_.SetTargetTorque(up_torque);
		up_pitch_motor_.PackCtrlFrame(frame);
		gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);

		DUST_LOG_DBG("pitch_up_track", "%d,%d", static_cast<int>(up_target_deg_ * 1000.0f), static_cast<int>(imu_msg_.pitch * 1000.0f));
	}
	#endif

	#if defined(CONFIG_TRD_PITCH_DOWN)
	{
		// down pitch
		if (g_set_zero_down_ || !gimbal::Enabled(down_pitch_motor_)) {
			return TaskResult::Retry;
		}

		// 窗口滑窗检测掉线：flag_ 与 pre_flag_ 相等即本拍无新反馈，按周期分频检查
		if (down_motor_watch_.Update(down_pitch_motor_)) {
			DUST_LOG_ERR("down pitch lost");
			return TaskResult::Retry;
		}

		// 下 pitch：电机累计角取反后归一到 [-pi, pi)，与取反的力矩输出统一：逆时针为正。
		const DmMotor::Snapshot motor = down_pitch_motor_.ReadAll();
		const float down_now_rad 	  = WrapRad(-motor.radian);
		const float down_now_deg 	  = down_now_rad * kRadToDeg;
		// SB 上档用 var 目标；其余档位用遥控 pitch 映射到机械范围。
		const float down_target_rad   = down_target_deg_ * kDegToRad;

		float pid_torque  = 0.0f;
		float down_torque = 0.0f;

		if (std::isfinite(down_target_rad) && std::isfinite(down_now_rad))
		{
			const float down_reference_rad = SmoothDownTarget(down_target_rad, down_now_rad, dt);
			const float down_err		   = WrapRad(down_now_rad - down_reference_rad);
			pid_torque  = down_pid_.Calc(0.0f, down_err, dt);

			down_torque = std::clamp(pid_torque, -kPitchTorqueMax, kPitchTorqueMax);

			// 逆时针为正；边界及越界时仅允许向范围内施加力矩。
			const bool push_below_min = down_now_rad <= kDownPitchMinDeg * kDegToRad && down_torque < 0.0f;
			const bool push_above_max = down_now_rad >= kDownPitchMaxDeg * kDegToRad && down_torque > 0.0f;

			if (push_below_min || push_above_max) {
				down_torque = 0.0f;
				down_pid_.Reset();
			}
		} 
		else 
		{
			down_pid_.Reset();
			down_target_curve_ = {};
		}

		down_pitch_motor_.SetTargetTorque(-down_torque);
		down_pitch_motor_.PackCtrlFrame(frame);
		gimbal::SendCmd(&user_can0_msgq, kDownPitchCanId, frame);

		DUST_LOG_DBG("pitch_down_track", "%f,%f,%f", static_cast<double>(down_target_deg_), static_cast<double>(down_now_deg), static_cast<double>(down_torque));
	}
	#endif

    return TaskResult::Continue;
}

static bool thread_init()
{
	alg::pid::Pid::Config pid_cfg {
		.outMax = kPitchTorqueMax,
	};

    DmMotor::Config cfg {
		.ctrl_met 	    = ControlMethon::Mit, 
		.gearbox_ratio 	= 10.f,
		.wheel_r 		= 0.f,
		.kp 			= 0.0f,
		.kd 			= 0.0f,
		.PMAX 			= 12.5f,
		.VMAX 			= 30.0f,
		.TMAX			= 10.f,
	};

	#if defined(CONFIG_TRD_PITCH_UP)
	{
		pid_cfg.kp      = 1.2f;
		pid_cfg.ki      = 0.0f;
		pid_cfg.kd      = 0.075f;
		up_pid_.Init(pid_cfg);

		cfg.can_id    = kUpPitchCanId;
		cfg.master_id = kUpPitchMasterId;
		up_pitch_motor_.Init(cfg);
	}
	#endif
    
	#if defined(CONFIG_TRD_PITCH_DOWN)
	{
		pid_cfg.kp     = 100.f;
		pid_cfg.ki     = 0.0f;
		pid_cfg.kd     = 10.0f;
		down_pid_.Init(pid_cfg);

		cfg.can_id    = kDownPitchCanId;
		cfg.master_id = kDownPitchMasterId;
		down_pitch_motor_.Init(cfg);
	}
	#endif

    DUST_LOG_INF("pitch ready: CAN0");

    return true;
}

/**
 * @brief 启动 pitch 线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "pitch", ThreadFaultConfig{ OnRetry, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "pitch_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "pitch_start");

#if defined(CONFIG_TRD_PITCH_DOWN)

CAN_RX_HANDLER(USER_RX_CAN0, kDownPitchMasterId,   [](uint8_t* data) { down_pitch_motor_  .CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("pitch_down_kp",     down_pid_.GetKp());
REGISTER_SHELL_VAR("pitch_down_ki",     down_pid_.GetKi());
REGISTER_SHELL_VAR("pitch_down_kd",     down_pid_.GetKd());

REGISTER_SHELL_VAR("pitch_down_target", down_target_deg_);
REGISTER_SHELL_VAR("pitch_zero_down",   g_set_zero_down_);

#endif

#if defined(CONFIG_TRD_PITCH_UP)

CAN_RX_HANDLER(USER_RX_CAN0, kUpPitchMasterId,     [](uint8_t* data) { up_pitch_motor_    .CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("pitch_up_kp",       up_pid_.GetKp());
REGISTER_SHELL_VAR("pitch_up_ki",       up_pid_.GetKi());
REGISTER_SHELL_VAR("pitch_up_kd",       up_pid_.GetKd());

REGISTER_SHELL_VAR("pitch_up_target",   up_target_deg_);
REGISTER_SHELL_VAR("pitch_zero_up",     g_set_zero_up_);

#endif

} // namespace thread::pitch
