/**
 * @file trd_ecat.hpp
 * @author qingyu
 * @brief EtherCAT 基础 PDO：双向各 64 个 uint32_t
 * @date 2026-11-18
 */

#pragma once

#include <cstdint>

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

// 获取最近一次线程快照；只供线程调用，内部使用互斥锁。
// 未收到主站数据时 outputs 为零；停更时保留最后一帧，不代表实时有效命令。
void GetOutputs(PdoOutputs& outputs);
void GetInputs(PdoInputs& inputs);

} // namespace instance::ecat
