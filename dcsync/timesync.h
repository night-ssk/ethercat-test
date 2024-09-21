/**
 * @file timesync.h
 * @author EtherCAT Developers
 * @brief EtherCAT 分布式时钟
 * @version 1.0
 * @date 2024-03-12
 *
 * @copyright Copyright 2024 (c), Precision Equipment and Robotics Team
 *
 */

#pragma once

#include <ecrt.h>

// 1 秒 = 1000000000 ns
#define NSEC_PER_SEC 1000000000L

// 任务循环频率（单位 `Hz`）
#define FREQUENCY 250

// 一个周期所需要的 `ns` 数
#define PERIOD_NS  NSEC_PER_SEC / FREQUENCY

/**
 * @brief 计算两个时间以 `ns` 为单位的差值
 *
 * @param[in] lhs 左操作数
 * @param[in] rhs 右操作数
 */
// inline int64_t operator-(const timespec &lhs, const timespec &rhs)
// {
//     return (lhs.tv_sec - rhs.tv_sec) * NSEC_PER_SEC + lhs.tv_nsec - rhs.tv_nsec;
// }

/**
 * @brief 获取通过 `system_time_base` 调整当前系统时间，单位为 `ns`
 *
 * @return 以 `ns` 为单位的当前系统时间
 */
uint64_t getSysTime();

/**
 * @brief 系统时间转换为计数器时间
 * 
 * @param[in] time 系统时间
 * @return 计数器时间
 */
uint64_t system2count(uint64_t time);

/**
 * @brief 同步分布式时钟 DC，获取时钟偏移
 * 
 * @param[in] master EtherCAT 主站
 * @param[out] dc_time 系统参考时钟
 * @return 时钟偏移
 */
int32_t syncDC(ec_master_t *master, uint64_t *dc_time);

/**
 * @brief 基于参考从站时间差更新主站时钟
 *
 * @param[in] dc_diff 时钟偏移
 * @note 在发送 EtherCAT 帧后调用，以避免在 `syncDC()` 中出现时间抖动
 */
void updateMasterClock(int32_t dc_diff);

/**
 * @brief 等待至当前的唤醒时间，并设置下一个唤醒时间
 *
 * @param[in] master EtherCAT 主站
 * @param[in] current_wt 当前唤醒时间
 * @param[out] next_wt 下一个唤醒时间
 */
void sleepUntil(ec_master_t *master, uint64_t current_wt, uint64_t *next_wt);
