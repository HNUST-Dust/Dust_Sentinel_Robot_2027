/**
 * @file trd_yaw_coop.hpp
 * @author qingyu
 * @brief 大小 yaw 协同：世界角分配 + 小轴软限
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "angle.hpp"

namespace instance::yaw {

constexpr float    kYawRateLimitRad     = alg::math::kPi;		// 遥控打满的世界角速度 rad/s

constexpr float kSmallSoftLimitRad = 45.0f * alg::math::kDegToRad;		// 小 yaw 软限 ±45°
constexpr float kSmallHardLimitRad = 50.0f * alg::math::kDegToRad;		// 小 yaw 硬限 ±50°
/**
 * @brief 世界角分配器
 *
 * 世界目标角速度积分成世界目标角；底盘朝向由 IMU 世界角减两轴关节角推出；
 * 目标需求超小轴软限的部分推给大轴参考，限位内大轴参考保持不动。
 */
class Coop final
{
public:
    void Init(float big_joint, float world_yaw);
    void Update(float remote_yaw, float world_yaw, float big_joint, float small_joint, float dt, float& big_target, float& small_target, float& world_target);

private:
    float world_ref_ = 0.0f;		// 世界目标角 rad
    float big_ref_   = 0.0f;		// 大 yaw 参考关节角 rad
};

} // namespace instance::yaw
