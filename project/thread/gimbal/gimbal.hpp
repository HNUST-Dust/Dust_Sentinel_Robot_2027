/**
 * @file gimbal.hpp
 * @author qingyu
 * @brief 云台达妙电机公共命令与掉线检测
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <zephyr/kernel.h>
#include "dm.hpp"

namespace instance::gimbal {

// 底盘相对云台的偏角：theta_chassis - theta_gimbal，rad，[-pi, pi]。
// 从上往下看逆时针为正；两个 yaw 电机零位对应底盘与云台朝向一致。
// 读写均由同一个互斥锁保护，仅在线程上下文调用。
// 云台系速度转底盘系：v_chassis = R(-chassis_relative_to_gimbal_yaw_rad) * v_gimbal。
// 直接访问变量时必须持有 chassis_yaw_mutex_；常规读写使用下面的接口。
extern k_mutex chassis_yaw_mutex_;
extern float chassis_relative_to_gimbal_yaw_rad;

void  SetChassisRelativeYaw(float angle_rad);

float GetChassisRelativeYaw();

/**
 * @brief 电机掉线检测：按拍数分频调用 PwrLossCheck
 *
 * 分频是为了让检测窗口覆盖若干拍，单拍无反馈不立即判掉线。
 */
class LostWatch
{
public:
    /**
     * @brief 分频周期
     * @param period 每多少拍检测一次
     */
    explicit LostWatch(uint32_t period) : period_(period == 0 ? 1 : period) {}

    /**
     * @brief 本拍是否检测到掉线
     * @param motor 目标电机
     * @return true=掉线
     */
    bool Update(DmMotor& motor)
    {
        if (++count_ < period_) return false;

        count_ = 0;

        return motor.PwrLossCheck();
    }

private:
    uint32_t period_;			// 检测分频拍数
    uint32_t count_ = 0;		// 当前累计拍数
};

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @param data   8 字节命令帧
 */
void SendCmd(k_msgq* msgq, uint16_t can_id, const uint8_t (&data)[8]);

/**
 * @brief 电机是否处于使能态
 * @param motor 目标电机
 * @return true=已使能
 */
bool Enabled(const DmMotor& motor);

/**
 * @brief 使能电机：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @return true=已使能；false=清错未生效或使能失败
 */
bool Enable(DmMotor& motor, k_msgq* msgq, uint16_t can_id);

/**
 * @brief 下发设零并等 1s 停稳
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 */
void SaveZero(DmMotor& motor, k_msgq* msgq, uint16_t can_id);

} // namespace instance::gimbal
