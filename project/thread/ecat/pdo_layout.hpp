/**
 * @file pdo_layout.hpp
 * @author qingyu
 * @brief EtherCAT PDO 布局：双向 256 B，本版不含电机
 * @version 1.0
 * @date 2026-11-25
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
static constexpr uint16_t kRemoteOffset          = 40;
static constexpr uint16_t kImuOffset             = 72;
static constexpr uint16_t kMotorFeedbackOffset   = 128;
static constexpr uint16_t kMotorCommandOffset    = 48;
static constexpr uint16_t kDeviceOperationOffset = 176;
static constexpr uint16_t kDiagnosticOffset      = 208;

/**
 * @brief TxPDO 头：下位机回传状态
 */
struct FeedbackHeader
{
    uint32_t magic;
    uint16_t proto_major;
    uint16_t proto_minor;
    uint32_t mcu_boot_id;
    uint32_t feedback_seq;
    uint32_t mcu_tick_us;
    uint32_t applied_command_seq;
    uint16_t accepted_host_session;
    uint16_t operation_ack_seq;
    uint16_t status_flags;
    uint16_t fault_code;
    uint8_t  armed_mask;
    uint8_t  can_health;
    uint8_t  reserved[6];
} __attribute__((packed));

/**
 * @brief TxPDO 遥控反馈
 */
struct RemoteFeedback
{
    uint32_t source_seq;
    uint32_t sample_tick_us;
    float    axis[4];
    uint16_t controls;
    uint8_t  protocol;
    uint8_t  status;
    uint8_t  reserved[4];
} __attribute__((packed));

/**
 * @brief TxPDO IMU 反馈
 */
struct ImuFeedback
{
    uint32_t source_seq;
    uint32_t sample_tick_us;
    float    quaternion[4];
    float    gyro[3];
    float    accel[3];
    float    temperature;
    uint16_t flags;
    uint8_t  reserved[2];
} __attribute__((packed));

/**
 * @brief RxPDO 头：上位机命令
 */
struct CommandHeader
{
    uint32_t magic;
    uint16_t proto_major;
    uint16_t proto_minor;
    uint16_t byte_length;
    uint16_t slot_map_version;
    uint16_t host_session;
    uint16_t reserved0;
    uint32_t command_seq;
    uint32_t mcu_boot_id_echo;
    uint32_t feedback_seq_echo;
    uint32_t lease_us;
    uint32_t compute_time_us;
    uint16_t flags;
    uint16_t enable_mask;
    uint8_t  reserved[8];
} __attribute__((packed));

/**
 * @brief RxPDO 诊断请求
 */
struct DiagnosticRequest
{
    uint32_t request_seq;
    uint16_t opcode;
    uint16_t flags;
    uint8_t  payload[8];
} __attribute__((packed));

/**
 * @brief 心跳/状态回传帧（从站 → 主站）
 */
struct PdoInputs
{
    FeedbackHeader header;
    RemoteFeedback remote;
    ImuFeedback    imu;
    uint8_t        motor_feedback[128];    // 8 槽 × 16 B，本版预留全零
} __attribute__((packed));

/**
 * @brief 命令接收帧（主站 → 从站）
 */
struct PdoOutputs
{
    CommandHeader     header;
    uint8_t           motor_command[128];    // 8 槽 × 16 B，本版预留
    uint8_t           device_operation[32];  // 本版预留
    DiagnosticRequest diagnostic;
    uint8_t           reserved[32];
} __attribute__((packed));

static_assert(sizeof(float) == 4);
static_assert(std::is_trivially_copyable_v<FeedbackHeader>);
static_assert(std::is_trivially_copyable_v<RemoteFeedback>);
static_assert(std::is_trivially_copyable_v<ImuFeedback>);
static_assert(std::is_trivially_copyable_v<CommandHeader>);
static_assert(std::is_trivially_copyable_v<DiagnosticRequest>);

static_assert(sizeof(FeedbackHeader) == 40);
static_assert(sizeof(RemoteFeedback) == 32);
static_assert(sizeof(ImuFeedback) == 56);
static_assert(sizeof(CommandHeader) == 48);
static_assert(sizeof(DiagnosticRequest) == 16);

static_assert(sizeof(PdoInputs) == kPdoBytes);
static_assert(sizeof(PdoOutputs) == kPdoBytes);

static_assert(offsetof(PdoInputs, remote) == kRemoteOffset);
static_assert(offsetof(PdoInputs, imu) == kImuOffset);
static_assert(offsetof(PdoInputs, motor_feedback) == kMotorFeedbackOffset);
static_assert(offsetof(PdoOutputs, motor_command) == kMotorCommandOffset);
static_assert(offsetof(PdoOutputs, device_operation) == kDeviceOperationOffset);
static_assert(offsetof(PdoOutputs, diagnostic) == kDiagnosticOffset);

} // namespace instance::ecat
