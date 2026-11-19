/**
 * @file navigation_packet.hpp
 * @author qingyu
 * @brief 导航通信协议：底盘速度命令、裁判系统状态与 CRC16 校验
 * @version 0.1
 * @date 2026-11-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace instance::ecat {

/**
 * @brief 导航下发的底盘速度命令，小端字节序
 */
struct CmdVelPacket
{
    static constexpr uint32_t kSize   = 15;
    static constexpr uint8_t  kHeader = 0x6B;

    uint8_t  header    = kHeader;    // 帧头 0x6B
    float    linear_x  = 0.0f;       // 底盘系 m/s，向前为正
    float    linear_y  = 0.0f;       // 底盘系 m/s，向左为正
    float    angular_z = 0.0f;       // rad/s，固定朝向联调时为零
    uint16_t checksum  = 0;          // CRC16，覆盖字节 0–12
} __attribute__((packed));

/**
 * @brief 下位机回传的裁判系统聚合状态，业务字段暂时保持零
 */
struct StatusPacket
{
    static constexpr std::size_t kSize   = 20;
    static constexpr uint8_t     kHeader = 0x6D;

    uint8_t  header                = kHeader;    // 帧头 0x6D
    uint8_t  sequence              = 0;          // 聚合帧序号，255 后回绕
    uint8_t  game_type             = 0;          // 0x0001 低四位，RMUL 3V3 为 4
    uint16_t stage_remain_s        = 0;          // 0x0001 当前阶段剩余秒数
    uint8_t  game_progress         = 0;          // 0x0001 高四位，比赛中为 4
    uint16_t current_hp            = 0;          // 0x0201 当前血量
    uint16_t maximum_hp            = 0;          // 0x0201 血量上限
    uint8_t  robot_id              = 0;          // 0x0201 本车 ID
    uint8_t  chassis_power_enabled = 0;          // 0x0201 底盘电源输出位
    uint16_t zone_state_and_rfid   = 0;          // 低 12 位控制区状态，高 4 位 RFID
    uint16_t ally_control_degree   = 0;          // 0x0003 己方控制度
    uint16_t enemy_control_degree  = 0;          // 0x0003 对方控制度
    uint16_t checksum              = 0;          // CRC16，覆盖字节 0–17
} __attribute__((packed));

static_assert(sizeof(float) == 4);
static_assert(std::numeric_limits<float>::is_iec559);
static_assert(std::is_trivially_copyable_v<CmdVelPacket>);
static_assert(std::is_trivially_copyable_v<StatusPacket>);
static_assert(sizeof(CmdVelPacket) == CmdVelPacket::kSize);
static_assert(sizeof(StatusPacket) == StatusPacket::kSize);
static_assert(offsetof(CmdVelPacket, checksum) == 13);
static_assert(offsetof(StatusPacket, checksum) == 18);

/**
 * @brief 计算 RoboMaster CRC16：初值 0xFFFF，反射多项式 0x8408，无最终异或
 * @param data 待校验字节
 * @param size 字节数，不含帧尾 CRC
 * @return CRC16，小端存入帧尾
 */
inline uint16_t NavigationCrc16(const void* data, std::size_t size)
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    uint16_t    crc   = 0xFFFF;

    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];

        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? (crc >> 1U) ^ 0x8408U : crc >> 1U;
        }
    }
    return crc;
}

/**
 * @brief 校验速度命令的帧头、CRC 与浮点数有效性
 * @param packet 接收到的速度命令
 * @return true=校验通过
 */
inline bool ValidCommand(const CmdVelPacket& packet)
{
    const bool header_valid   = packet.header == CmdVelPacket::kHeader;
    const bool checksum_valid = packet.checksum == NavigationCrc16(&packet, offsetof(CmdVelPacket, checksum));
    const bool velocity_valid = std::isfinite(packet.linear_x) && std::isfinite(packet.linear_y) && std::isfinite(packet.angular_z);

    return header_valid && checksum_valid && velocity_valid;
}

} // namespace instance::ecat
