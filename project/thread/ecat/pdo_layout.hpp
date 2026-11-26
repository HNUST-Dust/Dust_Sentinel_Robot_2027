/**
 * @file pdo_layout.hpp
 * @author qingyu
 * @brief EtherCAT PDO 布局：双向 256 B，本版不含电机
 * @version 1.1
 * @date 2026-11-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace instance::ecat {

// 协议常量
static constexpr uint32_t kFeedbackMagic  		 = 0x31424450;     // "PDB1"
static constexpr uint32_t kCommandMagic   		 = 0x31434D43;     // "CMC1"
static constexpr uint16_t kProtoMajor     		 = 1;
static constexpr uint16_t kProtoMinor     		 = 0;
static constexpr uint16_t kSlotMapVersion 		 = 1;
static constexpr uint16_t kPdoBytes       		 = 256;

// 帧内偏移
static constexpr uint16_t kRemoteOffset          = 34;
static constexpr uint16_t kImuOffset             = 62;
static constexpr uint16_t kMotorCommandOffset    = 48;
static constexpr uint16_t kDeviceOperationOffset = 176;
static constexpr uint16_t kDiagnosticOffset      = 208;

#pragma pack(push, 1)

/**
 * @brief TxPDO 头：下位机回传状态
 */
struct FeedbackHeader
{
    uint32_t magic;                    // 固定 0x31424450（ASCII "PDB1"）
    uint16_t proto_major;              // 协议主版本，本版 1
    uint16_t proto_minor;              // 协议次版本，本版 0
    uint32_t mcu_boot_id;              // 每次上电重新生成，主站据此判断从站重启
    uint32_t feedback_seq;             // 每 1 ms 自增，溢出回绕
    uint32_t mcu_tick_us;              // 本帧组帧时刻，MCU 本地 µs
    uint32_t applied_command_seq;      // 已执行到的 command_seq；本版无执行器，恒 0
    uint16_t accepted_host_session;    // 当前被接纳的主机会话号；0 = 无有效会话
    uint16_t operation_ack_seq;        // 设备管理操作应答序号；本版恒 0
    uint16_t status_flags;             // bit0 = 已接纳会话；其余保留为 0
    uint16_t fault_code;               // 故障码；本版恒 0
    uint8_t  armed_mask;               // 八槽使能位；本版恒 0
    uint8_t  can_health;               // CAN 总线健康摘要位；本版恒 0
};

/**
 * @brief TxPDO 遥控反馈
 */
struct RemoteFeedback
{
    uint32_t source_seq;               // 遥控消息帧序号，每解析一帧自增
    uint32_t sample_tick_us;           // 该帧采样时刻，MCU 本地 µs（非 PDO 组帧时刻）
    float    axis[4];                  // 归一化 [-1, 1]：chassisx / chassisy / yaw / pitch
    uint16_t controls;                 // 开关位：bit0 整车总开关、bit1 底盘 Spin、bit2 射击、bit3 装填、bit4 自瞄、bit5 超级电容、bit6~10 自定义
    uint8_t  protocol;                 // 遥控协议；0 = 未报告（本版恒 0）
    uint8_t  status;                   // bit0 = 收到过有效遥控帧；bit1 保留
};

/**
 * @brief TxPDO IMU 反馈
 */
struct ImuFeedback
{
    uint32_t source_seq;               // IMU 姿态帧序号，每解算一帧自增
    uint32_t sample_tick_us;           // 该帧采样时刻，MCU 本地 µs
    float    quaternion[4];            // 姿态四元数 w, x, y, z
    float    gyro[3];                  // 角速度，rad/s
    float    accel[3];                 // 加速度，m/s²
    uint16_t flags;                    // bit0 = 已产生过姿态；其余保留为 0
};

/**
 * @brief RxPDO 头：上位机命令
 */
struct CommandHeader
{
    uint32_t magic;                    // 必须 0x31434D43（ASCII "CMC1"）
    uint16_t proto_major;              // 协议主版本，必须 1
    uint16_t proto_minor;              // 协议次版本，必须 0
    uint16_t byte_length;              // 整帧长度，必须 256
    uint16_t slot_map_version;         // 槽位映射版本，必须 1
    uint16_t host_session;             // 主机会话号，同一会话生命周期内保持不变
    uint32_t command_seq;              // 仅产生新决策时自增，重复下发同一命令不得自增
    uint32_t mcu_boot_id_echo;         // 回显最近一次 TxPDO 读到的 mcu_boot_id
    uint32_t feedback_seq_echo;        // 本命令依据的 feedback_seq
    uint32_t lease_us;                 // 租约上限，µs；从站侧再夹一层
    uint32_t compute_time_us;          // 主站控制计算耗时，诊断用
    uint16_t flags;                    // bit0 请求 arm；bit1 请求 disarm
    uint16_t enable_mask;              // 使能掩码；本版忽略
};

/**
 * @brief RxPDO 诊断请求
 */
struct DiagnosticRequest
{
    uint32_t request_seq;              // 诊断请求序号
    uint16_t opcode;                   // 诊断操作码
    uint16_t flags;                    // 诊断标志位
    uint8_t  payload[8];               // 请求参数载荷
};

/**
 * @brief 心跳/状态回传帧（从站 → 主站）
 */
struct PdoInputs
{
    FeedbackHeader header;
    RemoteFeedback remote;
    ImuFeedback    imu;
    uint8_t        padding[144];           // 补齐到 256 B
};

/**
 * @brief 命令接收帧（主站 → 从站）：256 B 平铺，与上位机一致
 */
struct PdoOutputs
{
    uint8_t data[kPdoBytes];
};

#pragma pack(pop)

static_assert(sizeof(float) == 4);
static_assert(std::is_trivially_copyable_v<FeedbackHeader>);
static_assert(std::is_trivially_copyable_v<RemoteFeedback>);
static_assert(std::is_trivially_copyable_v<ImuFeedback>);
static_assert(std::is_trivially_copyable_v<CommandHeader>);
static_assert(std::is_trivially_copyable_v<DiagnosticRequest>);

static_assert(sizeof(FeedbackHeader) == 34);
static_assert(sizeof(RemoteFeedback) == 28);
static_assert(sizeof(ImuFeedback) == 50);
static_assert(sizeof(CommandHeader) == 38);
static_assert(sizeof(DiagnosticRequest) == 16);

static_assert(sizeof(PdoInputs) == kPdoBytes);
static_assert(sizeof(PdoOutputs) == kPdoBytes);

static_assert(offsetof(PdoInputs, remote) == kRemoteOffset);
static_assert(offsetof(PdoInputs, imu) == kImuOffset);

} // namespace instance::ecat
