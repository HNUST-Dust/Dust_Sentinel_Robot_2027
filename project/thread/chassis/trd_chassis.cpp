/**
 * @file trd_chassis.cpp
 * @author qingyu
 * @brief 全向轮底盘线程 —— 遥控速度 → 逆运动学 → C620 角速度环
 * @version 0.1
 * @date 2026-11-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Chassis"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <zephyr/zbus/zbus.h>
#include "trd_chassis.hpp"
#include "dt_meter.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "remote_to.hpp"
#include "to_can_tx.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::chassis {

using namespace instance::chassis;

static Thread<2048> thread_{};
static alg::pid::Pid pid_[N_Wheel] {};

static alg::monitor::DtMeter dt_meter_ {};			// 控制周期测量
static float last_target_[N_Wheel] {};				// 惯性前馈上一拍目标
static bool  target_valid_[N_Wheel] {};				// 闭环切入首拍不做目标差分

/**
 * @brief 摩擦前馈电流：按该轮辨识的库仑+黏性+静摩擦模型算出维持该转速所需电流
 *
 * 方向由目标转速决定（几乎阶跃，保证低速也能给满静摩擦），
 * 静摩擦按实际转速衰减（轮子真正停住时才是满值）。
 *
 * @param wheel   轮号
 * @param target  目标转速 rad/s
 * @param omega   实际转速 rad/s
 * @return 前馈电流 A
 */
static float FrictionFeedforward(uint8_t wheel, float target, float omega)
{
    const bool  forward  = (target >= 0.0f);
    const float a        = forward ? kFricA[wheel] : kFricANeg[wheel];
    const float b        = forward ? kFricB[wheel] : kFricBNeg[wheel];
    const float a_static = forward ? kFricStatic[wheel] : kFricStaticNeg[wheel];

    // 方向：用目标转速，平滑宽度小，低速目标也能拿到满静摩擦
    const float s = std::tanh(target / kFricStaticSmooth);

    // 静摩擦随实际转速平方指数衰减回库仑值
    const float r = omega / kFricSmooth;
    const float a_eff = a + (a_static - a) * std::exp(-r * r);

    return a_eff * s + b * target;
}

/**
 * @brief 加速度前馈：按目标转速变化率给出 J·α 对应的电流
 *
 * 目标转速在每拍更新，一阶差分近似 dω/dt；限幅避免阶跃瞬间给出尖峰。
 *
 * @param wheel  轮号
 * @param omega  目标转速 rad/s
 * @param dt     控制周期 s
 * @return 惯性前馈电流 A
 */
static float InertiaFeedforward(uint8_t wheel, float omega, float dt)
{
    float accel = 0.0f;

    if (target_valid_[wheel] && dt > 1.0e-4f) {
        accel = std::clamp((omega - last_target_[wheel]) / dt, -kAccelLimit, kAccelLimit);
    }

    last_target_[wheel] = omega;
    target_valid_[wheel] = true;

    return kJEq[wheel] * accel;
}

/**
 * @brief 每轮速度 PI + 摩擦/惯性前馈，总电流限幅并按总输出做条件积分
 */
static float SpeedCurrent(uint8_t wheel, float target, float omega, float dt)
{
    auto& pi = pid_[wheel];
    const float feedforward 	  = FrictionFeedforward(wheel, target, omega) + InertiaFeedforward(wheel, target, dt);
    const float previous_integral = pi.GetIntegralError();

    float feedback 	= pi.Calc(target, omega, dt);

    const float integral_delta = pi.GetKi() * (pi.GetIntegralError() - previous_integral);
    const float requested 	   = feedback + feedforward;

    // PID 内部只能看到反馈电流；前馈使总输出饱和时，也不能继续向饱和方向积分。
    if ((requested > kControlCurrentMax && integral_delta > 0.0f) || (requested < -kControlCurrentMax && integral_delta < 0.0f)) {
        pi.SetIntegralError(previous_integral);
        feedback -= integral_delta;
    }
    return std::clamp(feedback + feedforward, -kControlCurrentMax, kControlCurrentMax);
}

/**
 * @brief 重置速度环
 */
static void ResetSpeedControl(uint8_t wheel)
{
    pid_[wheel].Reset();
    target_valid_[wheel] = false;
}

/**
 * @brief 任务单拍：读遥控速度 → 逆运动学 → 四轮角速度环 → 组帧进 CAN 发送队列
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
	(void)context;

	static topic::remote_to::Message msg{};
	const zbus_channel* chan = nullptr;

	const float dt = dt_meter_.Update();		// 本拍周期，惯性前馈用

	zbus_sub_wait(&sub_remote_to, &chan, K_NO_WAIT);

	if (chan != nullptr) {
		zbus_chan_read(chan, &msg, K_NO_WAIT);
	}

	// 控制和诊断共用本拍快照，避免同一行里混入不同读取时刻的数据。
	motor::dji::DjiC620::Snapshot feedback[N_Wheel] {};
	for (uint8_t i = 0; i < N_Wheel; ++i) {
		feedback[i] = chassis_motor_[i].ReadAll();
	}

	DUST_LOG_DBG("wheel12", "%.2f,%.2f,%.2f,%.2f", static_cast<double>(feedback[0].omega), static_cast<double>(feedback[1].omega),
	             static_cast<double>(feedback[2].omega), static_cast<double>(feedback[3].omega));

	// 逆运动学反解：轮角速度 → 底盘实际速度 (m/s)
	{
		static constexpr float kInv4Sqrt2 = 0.25f * 0.70710678f;			// 1/(4√2)

		const float w0 = feedback[0].omega;
		const float w1 = feedback[1].omega;
		const float w2 = feedback[2].omega;
		const float w3 = feedback[3].omega;

		const float vx = (-w0 - w1 + w2 + w3) * kInv4Sqrt2 * kWheelRadius;
		const float vy = ( w0 - w1 - w2 + w3) * kInv4Sqrt2 * kWheelRadius;

		const bool enabled = msg.robot_ctrl != topic::remote_to::StartMode::Off;
		const float target_vx = enabled ? msg.chassisx * chassis_v : 0.0f;
		const float target_vy = enabled ? msg.chassisy * chassis_v : 0.0f;

		// 目标 vx、目标 vy、反馈 vx、反馈 vy，单位均为 m/s。
		DUST_LOG_DBG("chassis_v", "%.3f,%.3f,%.3f,%.3f", static_cast<double>(target_vx), static_cast<double>(target_vy), static_cast<double>(vx), static_cast<double>(vy));
	}

	float current[N_Wheel] {};
	float target[N_Wheel] {};

	if (msg.robot_ctrl == topic::remote_to::StartMode::Off)
	{
		for (uint8_t i = 0; i < N_Wheel; ++i)
		{
			ResetSpeedControl(i);
			current[i] = 0.0f;
		}
	}
	else
	{
		static constexpr float kSqrt2 = 1.41421356f;                    		// 对角线 → 边投影

		// 轮角速度 = 底盘线速度 / 轮半径
		const float vx =  msg.chassisx * chassis_v / kWheelRadius;
		const float vy =  msg.chassisy * chassis_v / kWheelRadius;
		const float wz = (msg.chassis_mode == topic::remote_to::ChassisMode::Spin) ? chassis_w : 0.0f;

		target[0] = (-vx + vy) * kSqrt2 + wz;
		target[1] = (-vx - vy) * kSqrt2 + wz;
		target[2] = ( vx - vy) * kSqrt2 + wz;
		target[3] = ( vx + vy) * kSqrt2 + wz;

		for (uint8_t i = 0; i < N_Wheel; ++i)
		{
			const float omega = feedback[i].omega;

			// 摩擦前馈补偿维持该转速所需的电流，惯性前馈补偿加减速，PID 只修残差
			current[i] = SpeedCurrent(i, target[i], omega, dt);
		}
	}

	topic::to_can_tx::Message tx{};

	tx.tx_id = kControlTxId;

	for (uint8_t i = 0; i < N_Wheel; ++i)
	{
		chassis_motor_[i].PackCurrent(tx.data, current[i]);
	}

	const int queue_rc = k_msgq_put(&user_can1_msgq, &tx, K_NO_WAIT);

	return TaskResult::Continue;
}

/**
 * @brief 初始化四个 C620 与角速度环
 * @return true=成功
 */
bool thread_init()
{
	static constexpr float kGearRatio = 3542.0f / 238.0f;

	motor::dji::DjiC620::Config motor_cfg{};

	// 电流域模型 J*dω/dt+b*ω=I-I_friction，按每轮 J/b 配置相同名义闭环极点。
	alg::pid::Pid::Config pid_cfg{};

	pid_cfg.outMax  = kControlCurrentMax;
	pid_cfg.iOutMax = kIntegralCurrentMax;

	for (uint8_t i = 0; i < N_Wheel; ++i)
	{
		motor_cfg.rx_id         = kDriveCanId[i];
		motor_cfg.gearbox_ratio = kGearRatio;
		motor_cfg.wheel_r       = kWheelRadius;

		const float b = 0.5f * (kFricB[i] + kFricBNeg[i]);

		pid_cfg.kp    = 2.0f    * kSpeedLoopZeta * kSpeedLoopWn * kJEq[i] - b;
		pid_cfg.ki    = kJEq[i] * kSpeedLoopWn   * kSpeedLoopWn;
		pid_cfg.kd    = 0.0f;

		chassis_motor_[i].Init(motor_cfg);
		pid_[i].Init(pid_cfg);
		DUST_LOG_INF("wheel%u PI: kp=%.6f ki=%.6f", static_cast<unsigned>(i), static_cast<double>(pid_cfg.kp), static_cast<double>(pid_cfg.ki));
	}

	DUST_LOG_INF("chassis ready");

	return true;
}

/**
 * @brief 启动底盘线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "chassis");
	return true;
}

REGISTER_INIT(thread_init, PreInit, High, HaltOnFail, nullptr, "chassis_init");
REGISTER_THREAD(thread_start, PreThread, ContinueOnFail, "chassis_start");

REGISTER_SHELL_VAR("chassis_v", chassis_v);
REGISTER_SHELL_VAR("chassis_w", chassis_w);

// C620 反馈注册（CAN1 分发）
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[0], [](uint8_t* data) { chassis_motor_[0].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[1], [](uint8_t* data) { chassis_motor_[1].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[2], [](uint8_t* data) { chassis_motor_[2].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[3], [](uint8_t* data) { chassis_motor_[3].CanCpltRxCallback(data); });

} // namespace thread::chassis
