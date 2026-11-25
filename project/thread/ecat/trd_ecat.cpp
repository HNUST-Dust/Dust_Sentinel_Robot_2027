/**
 * @file trd_ecat.cpp
 * @author qingyu
 * @brief HPM5E31 EtherCAT PDO 组帧与命令接收
 * @version 0.2
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstring>
#include <zephyr/kernel.h>

#include "trd_ecat.hpp"
#include "Init_entry.hpp"
#include "ecat_hal_hpm.hpp"
#include "ethercat.hpp"
#include "rtl8201f/phy_rtl8201f.hpp"
#include "log.hpp"
#include "shell_cmd.hpp"
#include "token.hpp"
#include "thread.hpp"
#include "remote_to.hpp"
#include "imu_to.hpp"

namespace instance::ecat {

namespace {

K_MUTEX_DEFINE(pdo_mutex_);
PdoOutputs outputs_ {};
PdoInputs  inputs_  {};

} // namespace

/**
 * @brief 读取最近一次主站下发的 PDO 快照
 * @param outputs 接收快照，不代表命令有效性
 */
void GetOutputs(PdoOutputs& outputs)
{
    k_mutex_lock(&pdo_mutex_, K_FOREVER);
    outputs = outputs_;
    k_mutex_unlock(&pdo_mutex_);
}

/**
 * @brief 读取最近一次下位机生成的 PDO 快照
 * @param inputs 回传快照
 */
void GetInputs(PdoInputs& inputs)
{
    k_mutex_lock(&pdo_mutex_, K_FOREVER);
    inputs = inputs_;
    k_mutex_unlock(&pdo_mutex_);
}

} // namespace instance::ecat

namespace thread::ecat {

using namespace instance::ecat;
using Slave = ::ecat::EthercatSlave<PdoOutputs, PdoInputs>;

static_assert(Slave::Profile::kRxBytes == 256);
static_assert(Slave::Profile::kTxBytes == 256);

static Thread<4096> thread_  {};
static Slave        slave_   {};

static constexpr uint32_t kCommandTimeoutMs = 100;    // 命令停更超时 ms

static uint32_t boot_id_           = 0;               // 本次上电标识
static uint32_t feedback_seq_      = 0;               // 回传帧序号
static uint32_t last_sm2_count_    = 0;               // 最近一次 SM2 收帧计数
static uint32_t last_command_ms_   = 0;               // 最近一次有效命令时间 ms
static uint32_t last_command_seq_  = 0;               // 最近一次有效命令序号
static uint16_t accepted_session_  = 0;               // 当前接纳的主机会话

static topic::remote_to::Message remote_msg_ {};      // 最近一帧遥控数据
static topic::imu_to::Message    imu_msg_    {};      // 最近一帧 IMU 数据

/**
 * @brief 读取当前 MCU 本地时刻（µs，32 位）
 * @return µs 计数
 */
static uint32_t NowUs()
{
    return static_cast<uint32_t>(k_cyc_to_us_floor64(k_cycle_get_64()));
}

/**
 * @brief 从 zbus 取遥控与 IMU 的最新消息
 */
static void ReadTopics()
{
    topic::remote_to::Message remote {};

    if (zbus_chan_read(&pub_remote_to, &remote, K_NO_WAIT) == 0) {
        remote_msg_ = remote;
    }

    topic::imu_to::Message imu {};

    if (zbus_chan_read(&pub_imu_to, &imu, K_NO_WAIT) == 0) {
        imu_msg_ = imu;
    }
}

/**
 * @brief 把遥控模式与开关打包成 controls 位
 * @param msg 遥控消息
 * @return 位域
 */
static uint16_t PackControls(const topic::remote_to::Message& msg)
{
    uint16_t bits = 0;

    bits |= (msg.robot_ctrl    == topic::remote_to::StartMode::On) ? (1u << 0) : 0u;
    bits |= (msg.chassis_mode  == topic::remote_to::ChassisMode::Spin) ? (1u << 1) : 0u;
    bits |= (msg.shoot_ctrl    == topic::remote_to::StartMode::On) ? (1u << 2) : 0u;
    bits |= (msg.reload_ctrl   == topic::remote_to::StartMode::On) ? (1u << 3) : 0u;
    bits |= (msg.autoaim_ctrl  == topic::remote_to::StartMode::On) ? (1u << 4) : 0u;
    bits |= (msg.supercap_ctrl == topic::remote_to::StartMode::On) ? (1u << 5) : 0u;
    bits |= (msg.custom1 	   == topic::remote_to::StartMode::On) ? (1u << 6) : 0u;
    bits |= (msg.custom2 	   == topic::remote_to::StartMode::On) ? (1u << 7) : 0u;
    bits |= (msg.custom3 	   == topic::remote_to::StartMode::On) ? (1u << 8) : 0u;
    bits |= (msg.custom4 	   == topic::remote_to::StartMode::On) ? (1u << 9) : 0u;
    bits |= (msg.custom5 	   == topic::remote_to::StartMode::On) ? (1u << 10) : 0u;

    return bits;
}

/**
 * @brief 组装本拍 TxPDO：头 + 遥控 + IMU，电机区保持零
 * @param inputs 待发送的 PDO
 */
static void FillInputs(PdoInputs& inputs)
{
    inputs = {};

    inputs.header.magic                 = kFeedbackMagic;
    inputs.header.proto_major           = kProtoMajor;
    inputs.header.proto_minor           = kProtoMinor;
    inputs.header.mcu_boot_id           = boot_id_;
    inputs.header.feedback_seq          = ++feedback_seq_;
    inputs.header.mcu_tick_us           = NowUs();
    inputs.header.accepted_host_session = accepted_session_;
    inputs.header.status_flags          = (accepted_session_ != 0) ? 1u : 0u;

    inputs.remote.source_seq     = remote_msg_.version;
    inputs.remote.sample_tick_us = remote_msg_.sample_us;
    inputs.remote.axis[0]        = remote_msg_.chassisx;
    inputs.remote.axis[1]        = remote_msg_.chassisy;
    inputs.remote.axis[2]        = remote_msg_.yaw;
    inputs.remote.axis[3]        = remote_msg_.pitch;
    inputs.remote.controls       = PackControls(remote_msg_);
    inputs.remote.status         = (remote_msg_.version != 0) ? 1u : 0u;

    inputs.imu.source_seq     = imu_msg_.version;
    inputs.imu.sample_tick_us = imu_msg_.sample_us;

    for (uint8_t i = 0; i < 4; ++i) {
        inputs.imu.quaternion[i] = imu_msg_.quaternion[i];
    }

    for (uint8_t i = 0; i < 3; ++i) {
        inputs.imu.gyro[i]  = imu_msg_.gyro[i];
        inputs.imu.accel[i] = imu_msg_.accel[i];
    }

    inputs.imu.temperature = imu_msg_.temperature;
    inputs.imu.flags       = (imu_msg_.version != 0) ? 1u : 0u;
}

/**
 * @brief 校验 RxPDO 头并记录会话与序号，本版不驱动任何输出
 * @param outputs     主站下发的 PDO
 * @param sm2_count   本拍 SM2 收帧计数，同值命令重复下发也计为更新
 * @param operational 当前是否处于 OP 状态
 */
static void DecodeCommand(const PdoOutputs& outputs, uint32_t sm2_count, bool operational)
{
    const CommandHeader& header = outputs.header;

    if (sm2_count != last_sm2_count_)
    {
        last_sm2_count_ = sm2_count;

        const bool header_ok = header.magic == kCommandMagic && header.byte_length == kPdoBytes && 
							   header.slot_map_version == kSlotMapVersion && header.mcu_boot_id_echo == boot_id_;

        if (header_ok && operational) {
            accepted_session_ = header.host_session;
            last_command_seq_ = header.command_seq;
            last_command_ms_  = k_uptime_get_32();
        }
    }

    if (!operational || k_uptime_get_32() - last_command_ms_ >= kCommandTimeoutMs) {
        accepted_session_ = 0;
    }

    DUST_LOG_DBG("ecat_cmd", "%d,%d,%d,%d", static_cast<int>(accepted_session_), static_cast<int>(last_command_seq_), static_cast<int>(feedback_seq_), static_cast<int>(accepted_session_ != 0));
}

/**
 * @brief 任务单拍：组帧、交换 PDO、校验命令
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
    (void)context;

    PdoInputs  inputs  {};
    PdoOutputs outputs {};

    ReadTopics();
    FillInputs(inputs);
    slave_.Poll();

    // 直接复用 PHY 链路检查，拔线后不等待 100 ms 命令停更超时。
    const bool link_up = slave_.LinkUp(0);

    // PDO 缓冲与 PDI ISR 共用，仅在复制期间屏蔽中断。
    const unsigned int key = irq_lock();

    slave_.SetInputs(inputs);
    outputs = slave_.Outputs();

    const uint32_t sm2_count  = slave_.Sm2Count();
    const bool    operational = link_up && slave_.State() == AlState::Op;

    irq_unlock(key);

    DecodeCommand(outputs, sm2_count, operational);

    k_mutex_lock(&pdo_mutex_, K_FOREVER);
    outputs_ = outputs;
    inputs_  = inputs;
    k_mutex_unlock(&pdo_mutex_);

    return TaskResult::Continue;
}

/**
 * @brief EtherCAT 故障报告命令
 * @param args 子命令及参数
 */
static void EcatCommand(uint8_t* args)
{
    const char* command = nullptr;
    uint8_t*    rest    = shell::NextToken(args, command);

    if (std::strcmp(command, "fault") != 0) {
        DUST_LOG_RAW("usage: ecat fault [clear]");
    } else if (*rest == '\0') {
        DUST_LOG_RAW("%s", slave_.RequestFaultReport() ? "ecat: saved fault report requested" : "ecat: no saved fault");
    } else if (std::strcmp(reinterpret_cast<const char*>(rest), "clear") == 0) {
        slave_.ClearFaultReport();
        DUST_LOG_RAW("ecat: fault clear requested");
    } else {
        DUST_LOG_RAW("usage: ecat fault [clear]");
    }
}

/**
 * @brief 初始化 ESC 与 Port0 的 RTL8201F
 * @return true=成功
 */
static bool thread_init()
{
    static ::ecat::EcatEscHpm esc_     {};
    static ::PhyRtl8201f      phy0_    {};

    // 按真实 ESC 端口排列；MDIO 地址由本项目 overlay 的 <1> 提供。
    static ::EcatPhy* const phys[] { &phy0_, nullptr };

    boot_id_ = k_cycle_get_32();

    if (!slave_.Init(&esc_, phys)) {
        DUST_LOG_ERR("ecat: init failed");
        return false;
    }

    return true;
}

/**
 * @brief 启动 EtherCAT 线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "ecat");
    return true;
}

REGISTER_SHELL_CMD("ecat", "ecat fault [clear]", EcatCommand);
REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "ecat_init");
REGISTER_THREAD(thread_start, LateThread, ContinueOnFail, "ecat_start");

} // namespace thread::ecat
