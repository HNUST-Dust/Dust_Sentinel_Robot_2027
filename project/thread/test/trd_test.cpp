/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief IMU 姿态诊断：高速晃动与机械限位下的漂移观测
 * @version 0.1
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cmath>
#include <zephyr/zbus/zbus.h>
#include "thread.hpp"
#include "Init_entry.hpp"
#include "log.hpp"

namespace thread::test {

static Thread<1024> thread_ {};

/**
 * @brief 任务单拍：按分频把 IMU 原始量与解算姿态打成一条
 * @param context 未使用
 * @return Continue
 */
static TaskResult Task(void* context)
{
    (void)context;

    return TaskResult::Continue;
}

static bool thread_init()
{
    DUST_LOG_INF("imu diag ready");
    return true;
}

/**
 * @brief 启动 IMU 诊断线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::Normal, nullptr, "test", ThreadFaultConfig{ nullptr, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "test_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "test_start");

} // namespace thread::test
