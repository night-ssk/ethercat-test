/**
 * @file timesync.c
 * @author EtherCAT Developers
 * @brief 
 * @version 1.0
 * @date 2024-03-14
 * 
 * @copyright Copyright 2024 (c), Precision Equipment and Robotics Team
 * 
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include "timesync.h"
#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
// EtherCAT DC variables
#define DC_FILTER_CNT 1024

static int64_t system_time_base;

/**
 * @brief 获取通过 `system_time_base` 调整当前系统时间，单位为 `ns`
 *
 * @return 以 `ns` 为单位的当前系统时间
 */
uint64_t getSysTime()
{
    struct timespec time_spc;
    clock_gettime(CLOCK_MONOTONIC, &time_spc);
    uint64_t t = (uint64_t)(time_spc.tv_sec * NSEC_PER_SEC) + (time_spc).tv_nsec;

    if ((uint64_t)system_time_base > t)
        return t;
    else
        return t - system_time_base;
}
/**
 * @brief 系统时间转换为计数器时间
 *
 * @param[in] time 系统时间
 * @return 计数器时间
 */
uint64_t system2count(uint64_t time)
{
    uint64_t ret;
    if (system_time_base < 0 && (-system_time_base) > time)
        ret = time;
    else
        ret = time + system_time_base;
    return ret;
}
/**
 * @brief 同步分布式时钟 DC，获取时钟偏移
 *
 * @param[in] master EtherCAT 主站
 * @param[out] dc_time 系统参考时钟
 * @return 时钟偏移
 */
int32_t syncDC(ec_master_t *master, uint64_t *dc_time)
{
    uint32_t ref_time = 0;
    uint64_t prev_app_time = *dc_time;

    *dc_time = getSysTime();

    // 获取参考时钟用于同步主站周期
    ecrt_master_reference_clock_time(master, &ref_time);
    int32_t dc_diff = (prev_app_time) - ref_time;
    // 调用同步从站到参考从站
    ecrt_master_sync_slave_clocks(master);
    return dc_diff;
}

void updateMasterClock(int32_t dc_diff)
{
    // 定义变量
    static int32_t prev_dc_diff_ns;
    static int64_t dc_diff_total_ns;
    static int64_t dc_delta_total_ns;
    static bool dc_started;
    static int dc_filter_idx;
    static int64_t dc_adjust_ns;

    // 计算漂移（通过非标准化时间差）
    int32_t delta = dc_diff - prev_dc_diff_ns;
    prev_dc_diff_ns = dc_diff;

    // normalise the time diff
    dc_diff = ((dc_diff + (PERIOD_NS / 2)) % PERIOD_NS) - (PERIOD_NS / 2);

    // only update if primary master
    if (dc_started)
    {
        // add to totals
        dc_diff_total_ns += dc_diff;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT)
        {
            // add rounded delta average
            dc_adjust_ns += ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sgn(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
            if (dc_adjust_ns < -(0.001f * PERIOD_NS))
                dc_adjust_ns = -0.001f * PERIOD_NS;
            if (dc_adjust_ns > (0.001f * PERIOD_NS))
                dc_adjust_ns = 0.001f * PERIOD_NS;

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sgn(dc_diff);
    }
    else
        dc_started = (dc_diff != 0);
}

void sleepUntil(ec_master_t *master, uint64_t current_wt, uint64_t *next_wt)
{
    // 在原来的唤醒时间上增加补偿
    uint64_t wt_after = system2count(current_wt);
    struct timespec ts = {(wt_after / NSEC_PER_SEC), (wt_after % NSEC_PER_SEC)};
    int s = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

    if (s != 0)
    {
        if (s == EINTR)
            printf("Interrupted by signal handler");
        else
            printf("clock_nanosleep: errno = %d", s);
    }

    // 在 ns 级别上设置主时钟时间
    *next_wt = current_wt;
    ecrt_master_application_time(master, *next_wt);

    // 在系统时间中计算下一个唤醒时间
    *next_wt += PERIOD_NS;
}