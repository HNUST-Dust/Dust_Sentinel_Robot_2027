/**
 * @file trd_ecat.cpp
 * @author qingyu
 * @brief HPM5E31 EtherCAT 导航收发与底盘命令更新
 * @version 0.1
 * @date 2026-11-18
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

namespace instance::ecat {

K_MUTEX_DEFINE(chassis_command_mutex_);
ChassisCommand chassis_command_ {};

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

static constexpr uint32_t kCommandTimeoutMs = 100;    // 导航命令停更超时 ms
static uint8_t  status_sequence_ = 0;                 // 状态聚合帧序号
static uint32_t last_sm2_count_  = 0;                 // 最近一次 SM2 收帧计数
static uint32_t last_command_ms_ = 0;                 // 最近一次有效命令时间 ms

/**
 * @brief 生成回传状态包，业务字段及 PDO 剩余字节保持零
 * @param inputs 待发送的 PDO
 */
static void FillInputs(PdoInputs& inputs)
{
    StatusPacket status {};

    status.sequence = status_sequence_++;
    status.checksum = NavigationCrc16(&status, offsetof(StatusPacket, checksum));

    inputs = {};
    std::memcpy(&inputs, &status, sizeof(status));
}

/**
 * @brief 校验新收到的速度命令，失效或停更时清零，并更新底盘共享快照
 * @param outputs     主站下发的 PDO
 * @param sm2_count   本拍 SM2 收帧计数，同值命令重复下发也计为更新
 * @param operational 当前是否处于 OP 状态
 * @return true=命令有效，false=清零接收快照
 */
static bool UpdateCommand(const PdoOutputs& outputs, uint32_t sm2_count, bool operational)
{
	static ChassisCommand command_ {};

    const uint32_t now = k_uptime_get_32();

    if (sm2_count != last_sm2_count_)
    {
        last_sm2_count_ = sm2_count;

        CmdVelPacket packet {};
        std::memcpy(static_cast<void*>(&packet), &outputs, sizeof(packet));

        command_.valid = operational && ValidCommand(packet);

        if (command_.valid) {
            command_.velocity = packet;
            last_command_ms_  = now;
        }
    }

    if (!operational || now - last_command_ms_ >= kCommandTimeoutMs) {
        command_.valid = false;
    }

    if (!command_.valid) {
        command_.velocity = {};
    }

    k_mutex_lock(&chassis_command_mutex_, K_FOREVER);
    chassis_command_ = command_;
    k_mutex_unlock(&chassis_command_mutex_);

    // 打印本拍交给底盘的命令，断联或校验失败后立即显示零值。
    DUST_LOG_DBG("ecat_cmd", "%.3f,%.3f,%.3f,%d", static_cast<double>(command_.velocity.linear_x), static_cast<double>(command_.velocity.linear_y),
                 static_cast<double>(command_.velocity.angular_z), static_cast<int>(command_.valid));

    return command_.valid;
}

/**
 * @brief 任务单拍：聚合回传状态、交换 PDO、更新底盘命令
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
    (void)context;

    PdoInputs  inputs  {};
    PdoOutputs outputs {};

    FillInputs(inputs);
    slave_.Poll();

    // 直接复用 PHY 链路检查，拔线后不等待 100 ms 命令停更超时。
    const bool link_up = slave_.LinkUp(0);

    // PDO 缓冲与 PDI ISR 共用，仅在复制期间屏蔽中断。
    const unsigned int key = irq_lock();

    slave_.SetInputs(inputs);
    outputs = slave_.Outputs();

    const uint32_t sm2_count   = slave_.Sm2Count();
    const bool    operational = link_up && slave_.State() == AlState::Op;

    irq_unlock(key);

    if (!UpdateCommand(outputs, sm2_count, operational)) {
        outputs = {};
    }

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
