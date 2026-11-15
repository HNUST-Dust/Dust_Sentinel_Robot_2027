/**
 * @file trd_chassis.hpp
 * @author qingyu
 * @brief 四轮全向轮底盘定义
 * @version 0.1
 * @date 2026-11-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "Irq_handlers.h"
#include "dji_c6xx.hpp"

namespace instance::chassis {

#define CHASSIS_RX  USER_RX_CAN1

constexpr uint8_t  N_Wheel = 4;				// 全向轮数量

// C620 反馈 ID
constexpr uint16_t kDriveCanId[N_Wheel] { 0x201, 0x202, 0x203, 0x204 };

// C620 控制帧 ID
constexpr uint16_t kControlTxId = 0x200;

constexpr float	   kWheelRadius = 0.285f;	// 轮半径 m（570mm 直径）

// 目标 v 比例（遥控归一化 [-1,1] 乘以该值）
inline float chassis_v = 2.0f;

// 小陀螺目标旋转速度
inline float chassis_w = 15.0f;

// 摩擦前馈（辨识结果，单位 A）：I = a·tanh(ω/ωs) + b·ω
// a/b 来自 scripts/chassis_friction.py 模型二，正反分开；ωs 为符号函数平滑宽度
constexpr float kFricA[N_Wheel]    = { 0.33983f, 0.27259f, 0.53885f, 0.49975f };	// 正转库仑
constexpr float kFricB[N_Wheel]    = { 0.01966f, 0.02159f, 0.04012f, 0.02587f };	// 正转黏性
constexpr float kFricANeg[N_Wheel] = { 0.31649f, 0.28193f, 0.47052f, 0.50784f };	// 反转库仑
constexpr float kFricBNeg[N_Wheel] = { 0.02408f, 0.02293f, 0.04561f, 0.02563f };	// 反转黏性

constexpr float kFricSmooth = 0.5f;		// 平滑宽度 rad/s，低于此值用 tanh 过渡代替 sgn

inline motor::dji::DjiC620 chassis_motor_[N_Wheel] {};

} // namespace instance::chassis
