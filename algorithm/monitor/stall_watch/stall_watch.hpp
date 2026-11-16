/**
 * @file stall_watch.hpp
 * @author qingyu
 * @brief 帧序号停更检测 — 序号连续不变即判对端停发
 * @version 0.1
 * @date 2026-11-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#pragma message "Compiling Algorithm/Monitor/StallWatch"

#include <cstdint>

namespace alg::monitor {

/**
 * @brief 帧序号停更检测：序号连续不变即判对端停发
 *
 * 调用方每拍把话题里最新帧的 version 传进来，内部统计连续无新帧拍数。
 */
class StallWatch
{
public:
    /**
     * @brief 更新一拍的帧序号
     * @param version 本拍读到的最新帧序号
     * @return true=已停更达到上限
     */
    bool Update(uint32_t version)
    {
        if (version == seen_) {
            ++stall_;
        } else {
            seen_  = version;
            stall_ = 0;
        }

        return stall_ >= limit_;
    }

    /**
     * @brief 停更拍数上限
     * @param limit 连续无新帧拍数达到该值即判停更
     */
    void SetLimit(uint32_t limit) { limit_ = limit; }

    /**
     * @brief 是否已停更，不改变内部计数
     * @return true=当前已判停更
     */
    bool Timeout() const { return stall_ >= limit_; }

    /**
     * @brief 当前连续无新帧拍数
     * @return 拍数
     */
    uint32_t Stall() const { return stall_; }

private:
    uint32_t seen_  = 0;		// 已处理的帧序号
    uint32_t stall_ = 0;		// 连续无新帧拍数
    uint32_t limit_ = 100;		// 停更判定拍数上限
};

} // namespace alg::monitor
