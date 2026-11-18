/**
 * @file trd_ecat.cpp
 * @author qingyu
 * @brief HPM5E31 EtherCAT 基础收发：RTL8201F
 * @date 2026-11-18
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

namespace {

	K_MUTEX_DEFINE(pdo_mutex_);
	PdoOutputs outputs_ {};
	PdoInputs inputs_ {};

} // namespace

void GetOutputs(PdoOutputs& outputs)
{
    k_mutex_lock(&pdo_mutex_, K_FOREVER);
    outputs = outputs_;
    k_mutex_unlock(&pdo_mutex_);
}

void GetInputs(PdoInputs& inputs)
{
    k_mutex_lock(&pdo_mutex_, K_FOREVER);
    inputs = inputs_;
    k_mutex_unlock(&pdo_mutex_);
}

} // namespace instance::ecat

namespace thread::ecat {

using instance::ecat::PdoOutputs;
using instance::ecat::PdoInputs;
using Slave = ::ecat::EthercatSlave<PdoOutputs, PdoInputs>;

static_assert(Slave::Profile::kRxBytes == 256);
static_assert(Slave::Profile::kTxBytes == 256);

static Thread<4096> thread_ {};
static Slave slave_ {};
static ::ecat::EcatEscHpm esc_ {};
static ::PhyRtl8201f phy0_ {};
static uint32_t input_counter_ = 0;

static TaskResult Task(void* context)
{
    (void)context;

    // 与参考一致：TxPDO 每个字填入本拍计数，便于主站检查完整帧。
    PdoInputs inputs {};
    for (auto& word : inputs.counter) {
        word = input_counter_;
    }

    PdoOutputs outputs {};
    // 门面缓冲由 PDI ISR 同时访问：只在复制完整 PDO 时屏蔽中断。
    // Poll、互斥锁和日志均放在临界区之外，避免长时间占用中断。
    const unsigned int key = irq_lock();
    slave_.SetInputs(inputs);
    outputs = slave_.Outputs();
    irq_unlock(key);

    k_mutex_lock(&instance::ecat::pdo_mutex_, K_FOREVER);
    instance::ecat::outputs_ = outputs;
    instance::ecat::inputs_ = inputs;
    k_mutex_unlock(&instance::ecat::pdo_mutex_);

    slave_.Poll();
    ++input_counter_;
    return TaskResult::Continue;
}

static void EcatCommand(uint8_t* args)
{
    const char* command = nullptr;
    uint8_t* rest = shell::NextToken(args, command);

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

static bool thread_init()
{
    // 按真实 ESC 端口排列；MDIO 地址由本项目 overlay 的 <1> 提供。
    static ::EcatPhy* const phys[] {
		 &phy0_, nullptr
	};

    if (!slave_.Init(&esc_, phys)) {
        DUST_LOG_ERR("ecat: init failed");
        return false;
    }
	
    return true;
}

static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "ecat");
    return true;
}

REGISTER_SHELL_CMD("ecat", "ecat fault [clear]", EcatCommand);
REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "ecat_init");
REGISTER_THREAD(thread_start, LateThread, ContinueOnFail, "ecat_start");

} // namespace thread::ecat
