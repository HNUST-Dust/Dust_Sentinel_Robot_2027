/**
 * @file trd_yaw.hpp
 * @author qingyu
 * @brief 小 yaw 达妙电机定义
 * @version 0.3
 * @date 2026-11-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"
#include "angle.hpp"

namespace instance::yaw {

// 小 yaw 接 CAN0；反馈帧使用 Master ID。
constexpr uint16_t kSmallYawCanId 	 	= 0x03;
constexpr uint16_t kSmallYawMasterId 	= 0x13;

constexpr float	   kGearRatio 			= 10.0f;				// J4310-2EC 内置减速箱 10:1，同步带 1:1
constexpr float    kControlDT 			= 0.001f;				// 控制周期 s（Task 1ms）

constexpr uint32_t kCanGapMs  			= 1000;					// 电机命令之间的间隔

constexpr float    kYawRateLimitRad     = alg::math::kPi;		// 遥控打满的世界角速度 rad/s
constexpr float    kYawTorqueMax        = 4.0f;					// yaw 峰值力矩 N*m
constexpr float    kSmallSoftLimitDeg   = 45.0f;				// 小 yaw 关节角软限 ±45°
constexpr float    kSmallAngleOffsetDeg = -8.283081f;			// 小 yaw 反馈角零位偏置 °

// 摩擦前馈（2026-11-17 实测：动摩擦 正 0.31 / 反 0.33，静摩擦 正 0.40 / 反 0.30）
constexpr float    kFricCoulombPos    	= 0.30f;				// 正向运动库仑摩擦 N*m
constexpr float    kFricCoulombNeg    	= 0.32f;				// 反向运动库仑摩擦 N*m
constexpr float    kFricStaticPos     	= 0.45f;				// 正向静摩擦（起转阈值）N*m
constexpr float    kFricStaticNeg     	= 0.30f;				// 反向静摩擦（起转阈值）N*m
constexpr float    kFricDirSmoothDeg  	= 0.5f;					// 方向平滑宽度（度），tanh 饱和区外给满值
constexpr float    kFricSpeedSmooth   	= 0.2f;					// 静摩擦随转速衰减的特征角速度 rad/s

inline 	  DmMotor  small_yaw_motor_{};

} // namespace instance::yaw
