/**
 * @file trd_can_tx.cpp
 * @author qingyu
 * @brief CAN TX 线程——阻塞等待两路队列并按就绪事件发送
 * @version 0.3
 * @date 2026-11-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstdint>
#include <cstring>
#include <cstddef>
#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_can_tx.hpp"
#include "can.hpp"
#include "Irq_handlers.h"
#include "log.hpp"

#pragma message "Compiling Thread/Can"

namespace thread::can {

// k_msgq vs zbus：zbus 多个发布者共用一个 channel 会互相覆盖；
// k_msgq 内部拷贝数据，多 put 一 get 天然支持多发布者，且满时丢帧不阻塞。
static Thread<> thread_{};
static Can user_can0{};
static Can user_can1{};

struct CanTxChannel
{
	Can&        can;
	k_msgq*     msgq;
	const char* name;
};

static CanTxChannel channels_[] {
	{ user_can0, &user_can0_msgq, "can0" },
	{ user_can1, &user_can1_msgq, "can1" },
};

static constexpr std::size_t kCanCount = sizeof(channels_) / sizeof(channels_[0]);
static k_poll_event events_[kCanCount] {};

/**
 * @brief 处理已就绪通道中的待发送消息
 * @param channel 对应的 CAN 实例、发送队列和日志名称
 */
static void SendPending(CanTxChannel& channel)
{
	topic::to_can_tx::Message msg{};

	while (k_msgq_get(channel.msgq, &msg, K_NO_WAIT) == 0)
	{
		can_frame tx {
			.id = msg.tx_id,
			.dlc = 8,
		};

		std::memcpy(tx.data, msg.data, 8);

		if (!channel.can.Send(&tx, K_MSEC(1))) {
			DUST_LOG_ERR("%s tx fail id=0x%x", channel.name, static_cast<unsigned>(tx.id));
		}
	}
}

/**
 * @brief 阻塞等待两路发送队列，仅处理本次就绪的消息
 * @param context 未使用
 * @return Continue=本轮就绪队列处理结束
 */
static TaskResult Task(void* context)
{
	(void)context;

	for (auto& event : events_) {
		event.state = K_POLL_STATE_NOT_READY;
	}

	const int ret = k_poll(events_, static_cast<int>(kCanCount), K_FOREVER);
	if (ret != 0) {
		DUST_LOG_ERR("can poll fail ret=%d", ret);
		return TaskResult::Continue;
	}

	for (std::size_t i = 0; i < kCanCount; ++i)
	{
		if (events_[i].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
			SendPending(channels_[i]);
		}
	}

	return TaskResult::Continue;
}

/**
 * @brief 初始化 user_can0 / user_can1 并挂接收分发回调
 * @return true=成功
 */
bool thread_init()
{
	for (uint8_t i = 0; i < kCanCount; i++) {
		k_poll_event_init(&events_[i], K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, channels_[i].msgq);
	}

	{
		const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can0));

		if (!device_is_ready(dev)) {
			DUST_LOG_ERR("user_can0 not ready");
			return false;
		}

		const can_filter filter { .id = 0, .mask = 0, .flags = 0 };

		user_can0.Init(dev, filter);
		user_can0.SetRxCallback(user_can0_rx_callback);
		DUST_LOG_INF("user_can0 ready");
	}

	{
		const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can1));

		if (!device_is_ready(dev)) {
			DUST_LOG_ERR("user_can1 not ready");
			return false;
		}

		const can_filter filter { .id = 0, .mask = 0, .flags = 0 };

		user_can1.Init(dev, filter);
		user_can1.SetRxCallback(user_can1_rx_callback);
		DUST_LOG_INF("user_can1 ready");
	}

	return true;
}

/**
 * @brief 启动 CAN TX 线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_NO_WAIT, ThreadPrio::High, nullptr, "can");
	return true;
}

REGISTER_INIT(thread_init, PreInit, High, HaltOnFail, nullptr, "can_init");
REGISTER_THREAD(thread_start, PreThread, ContinueOnFail, "can_start");

} // namespace thread::can
