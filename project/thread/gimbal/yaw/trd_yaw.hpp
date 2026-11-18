/**
 * @file trd_yaw.hpp
 * @author qingyu
 * @brief 大小 yaw 达妙电机定义
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

// 大 yaw 接 CAN1；反馈帧使用 Master ID。
constexpr uint16_t kBigYawCanId 	 	= 0x01;
constexpr uint16_t kBigYawMasterId 	= 0x11;

// 小 yaw 接 CAN0；反馈帧使用 Master ID。
constexpr uint16_t kSmallYawCanId = 0x03;
constexpr uint16_t kSmallYawMasterId = 0x13;

constexpr float	   kGearRatio 			= 10.0f;				// J4310-2EC 内置减速箱 10:1，同步带 1:1
constexpr float    kControlDT 			= 0.001f;				// 控制周期 s（Task 1ms）

constexpr uint32_t kCanGapMs  			= 1000;					// 电机命令之间的间隔

constexpr float    kYawRateLimitRad     = alg::math::kPi;		// 遥控打满的世界角速度 rad/s
constexpr float    kYawTorqueMax        = 4.0f;					// yaw 峰值力矩 N*m

inline 	  DmMotor  big_yaw_motor_{};
inline    DmMotor  small_yaw_motor_{};

} // namespace instance::yaw
