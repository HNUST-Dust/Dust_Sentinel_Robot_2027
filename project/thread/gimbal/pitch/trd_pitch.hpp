/**
 * @file trd_pitch.hpp
 * @author qingyu
 * @brief 双 pitch 达妙电机定义
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"

namespace instance::pitch {

constexpr float    kPitchTorqueMax      = 7.0f;			// pitch 峰值力矩 N*m

// 上 pitch 硬限位（度），电机角度逆时针为正
constexpr uint16_t kUpPitchCanId      	= 0x04;
constexpr uint16_t kUpPitchMasterId   	= 0x14;
constexpr float    kUpPitchMinDeg     	= 60.0f;		// 上 pitch 机械下限（度）
constexpr float    kUpPitchMaxDeg     	= -20.0f;		// 上 pitch 机械上限（度）
inline    DmMotor  up_pitch_motor_    	{};

// 下 pitch 硬限位（度），电机角度逆时针为正
constexpr uint16_t kDownPitchCanId      = 0x02;
constexpr uint16_t kDownPitchMasterId   = 0x12;
constexpr float    kDownPitchMinDeg 	= 0.0f;
constexpr float	   kDownPitchMaxDeg 	= 80.0f;
inline    DmMotor  down_pitch_motor_  	{};

} // namespace instance::pitch
