/**
 * @file chassis.hpp
 * @author qingyu
 * @brief 底盘四电机（C620）装配与反馈快照
 * @version 0.1
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dji_c6xx.hpp"

namespace instance::chassis {

using namespace motor::dji;

/**
 * @brief 底盘四电机装配：CAN1 上 C620 的接收分发与反馈快照
 *
 * 只做电机报文的接收与快照，不做任何控制计算。
 */
class Chassis final
{
public:
    /**
     * @brief 单槽反馈快照
     */
    struct Feedback
    {
        float    position;      // 输出轴角度 rad
        float    omega;         // 实测轮角速度 rad/s
        float    effort;        // 实测电流 A
        uint16_t age_ms;        // 距最近一次接收的时间 ms，饱和 65535
        uint16_t flags;         // bit0 在线；其余保留
    };

    static constexpr uint8_t  kWheelCount  = 4;         // 底盘轮数
    static constexpr uint16_t kRxIdBase    = 0x201;     // C620 反馈 ID 基准
    static constexpr uint32_t kOnlineAgeMs = 100;       // 在线判定阈值 ms

    bool Init();                                        // 配置四个 C620（CAN1 由 can_tx 线程初始化）
    Feedback Read(uint8_t slot) const;                  // 读取指定槽反馈快照
    void OnRx(uint8_t slot, uint8_t* data);             // CAN 接收分发（ISR 上下文）

private:
    static uint32_t NowUs();                            // 当前 MCU 本地时刻 µs

    DjiC620  motor_[kWheelCount] {};         			// 四个 C620
    uint32_t last_rx_us_[kWheelCount] {};    			// 每槽最近接收时刻
    uint32_t rx_seq_[kWheelCount] {};        			// 每槽接收计数
    bool     ready_ = false;                 			// 初始化完成
};

inline Chassis chassis_;                                // 模块唯一实例

static_assert(sizeof(Chassis::Feedback) == 16, "Feedback must match PDO motor slot");

} // namespace instance::chassis
