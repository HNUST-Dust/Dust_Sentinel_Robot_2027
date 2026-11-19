/**
 * @file trd_ecat.hpp
 * @author qingyu
 * @brief EtherCAT 导航通信：PDO 快照与底盘共享命令
 * @version 0.1
 * @date 2026-11-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <zephyr/kernel.h>
#include "navigation_packet.hpp"

namespace instance::ecat {

// 从站视角：主站写入 RxPDO，从站通过 TxPDO 回复主站。
struct PdoOutputs
{
    uint32_t counter[64];
};

struct PdoInputs
{
    uint32_t counter[64];
};

static_assert(sizeof(PdoOutputs) == 256, "RxPDO must be 256 bytes");
static_assert(sizeof(PdoInputs) == 256, "TxPDO must be 256 bytes");

/**
 * @brief 底盘共享命令：EtherCAT 线程写入，chassis 线程读取
 */
struct ChassisCommand
{
    CmdVelPacket velocity {};
    bool         valid = false;     // OP 下通过校验，且最近 100 ms 内收到有效 PDO
};

// 直接访问共享命令时，读写均须持有 chassis_command_mutex_。
extern k_mutex        chassis_command_mutex_;
extern ChassisCommand chassis_command_;

// 获取最近一次线程快照；只供线程调用，内部使用互斥锁。
// 未收到有效命令、断链、退出 OP 或命令停更超时时，outputs 清零。
void GetOutputs(PdoOutputs& outputs);
void GetInputs(PdoInputs& inputs);

} // namespace instance::ecat
