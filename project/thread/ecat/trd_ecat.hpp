/**
 * @file trd_ecat.hpp
 * @author qingyu
 * @brief EtherCAT 通信：PDO 快照读写
 * @version 0.2
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <zephyr/kernel.h>
#include "pdo_layout.hpp"

namespace instance::ecat {

// 从站视角：主站写入 RxPDO（PdoOutputs），从站通过 TxPDO（PdoInputs）回复主站。
static_assert(sizeof(PdoOutputs) == 256, "RxPDO must be 256 bytes");
static_assert(sizeof(PdoInputs) == 256, "TxPDO must be 256 bytes");

// 获取最近一次线程快照；只供线程调用，内部使用互斥锁。
void GetOutputs(PdoOutputs& outputs);
void GetInputs(PdoInputs& inputs);

} // namespace instance::ecat
