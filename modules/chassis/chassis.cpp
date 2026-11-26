/**
 * @file chassis.cpp
 * @author qingyu
 * @brief 底盘四电机（C620）CAN 接收装配与反馈快照
 * @version 0.1
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstdint>
#include <zephyr/kernel.h>

#include "chassis.hpp"
#include "Irq_handlers.h"
#include "Init_entry.hpp"
#include "log.hpp"

namespace instance::chassis {

namespace {

constexpr float kGearRatio   = 3591.0f / 187.0f;    // C620 减速比
constexpr float kWheelRadius = 0.285f;              // 轮半径 m

} // namespace

/**
 * @brief 当前 MCU 本地时刻（µs，32 位）
 * @return µs 计数
 */
uint32_t Chassis::NowUs()
{
    return static_cast<uint32_t>(k_cyc_to_us_floor64(k_cycle_get_64()));
}

/**
 * @brief 配置四个 C620；CAN1 的初始化与发送在 can_tx 线程
 * @return true=成功
 */
bool Chassis::Init()
{
    for (uint8_t i = 0; i < kWheelCount; ++i)
	{
        motor::dji::DjiC620::Config cfg {};

        cfg.rx_id         = static_cast<uint16_t>(kRxIdBase + i);
        cfg.gearbox_ratio = kGearRatio;
        cfg.wheel_r       = kWheelRadius;

        motor_[i].Init(cfg);
    }

    ready_ = true;
    DUST_LOG_INF("chassis: motors ready");

    return true;
}

/**
 * @brief CAN 接收分发：更新电机快照并记录接收时刻
 * @param slot 槽位 0–3
 * @param data 8 字节反馈数据
 */
void Chassis::OnRx(uint8_t slot, uint8_t* data)
{
    if (slot >= kWheelCount || !ready_) {
        return;
    }

    motor_[slot].CanCpltRxCallback(data);
    last_rx_us_[slot] = NowUs();
    ++rx_seq_[slot];
}

/**
 * @brief 读取指定槽的反馈快照
 * @param slot 槽位 0–3
 * @return 反馈；槽位非法或未初始化时全零、不在线
 */
Chassis::Feedback Chassis::Read(uint8_t slot) const
{
    if (slot >= kWheelCount || !ready_) {
        return {};
    }

    const auto snap = motor_[slot].ReadAll();

    Feedback fb {};

    fb.position = snap.radian;
    fb.omega    = snap.omega;
    fb.effort   = snap.current;

    if (last_rx_us_[slot] == 0) {
        fb.age_ms = UINT16_MAX;
        return fb;
    }

    const uint32_t age_ms = (NowUs() - last_rx_us_[slot]) / 1000u;

    fb.age_ms = static_cast<uint16_t>(age_ms > UINT16_MAX ? UINT16_MAX : age_ms);
    fb.flags  = (fb.age_ms < kOnlineAgeMs) ? 0x0001u : 0u;

    return fb;
}

} // namespace instance::chassis

namespace {

/**
 * @brief 初始化底盘电机装配
 * @return true=成功
 */
bool chassis_init()
{
    return instance::chassis::chassis_.Init();
}

} // namespace

CAN_RX_HANDLER(USER_RX_CAN1, 0x201, [](uint8_t* data) { instance::chassis::chassis_.OnRx(0, data); });
CAN_RX_HANDLER(USER_RX_CAN1, 0x202, [](uint8_t* data) { instance::chassis::chassis_.OnRx(1, data); });
CAN_RX_HANDLER(USER_RX_CAN1, 0x203, [](uint8_t* data) { instance::chassis::chassis_.OnRx(2, data); });
CAN_RX_HANDLER(USER_RX_CAN1, 0x204, [](uint8_t* data) { instance::chassis::chassis_.OnRx(3, data); });

REGISTER_INIT(chassis_init, PreInit, High, HaltOnFail, nullptr, "chassis_init");
