/**
 * @file dt_meter.hpp
 * @author qingyu
 * @brief 控制周期测量 — 相邻两拍 cycle 差值换算成秒
 * @version 0.1
 * @date 2026-11-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#pragma message "Compiling Algorithm/Monitor/DtMeter"

#include <algorithm>
#include <cstdint>
#include <zephyr/kernel.h>

namespace alg::monitor {

/**
 * @brief 控制周期测量：相邻两拍 cycle 差值换算成秒
 *
 * cycle 只用于求差，不回绕成绝对时间；Retry 路径会阻塞 1s，结果钳位避免积分突跳。
 */
class DtMeter
{
public:
    /**
     * @brief 本拍控制周期
     * @return 周期秒数，首拍或异常时返回 1ms
     */
    float Update()
    {
        const uint32_t now_cycle = k_cycle_get_32();
        float dt = 0.001f;

        if (last_cycle_ != 0) {
            const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle_);
            if (delta_ns != 0) {
                dt = std::clamp(static_cast<float>(delta_ns) * 1.0e-9f, 0.0001f, 0.01f);
            }
        }
        last_cycle_ = now_cycle;

        return dt;
    }

private:
    uint32_t last_cycle_ = 0;	// 上一拍 cycle 计数
};

} // namespace alg::monitor
