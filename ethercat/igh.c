#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "timesync.h"

#include "igh.h"
/* Application parameters */
#define CLOCK_TO_USE CLOCK_REALTIME
#define MEASURE_TIMING

/* 周期时间 */

/**
 * 检查过程数据状态
 */
void check_domain_state(struct _Domain *domain)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain->domain, &ds);

    if (ds.working_counter != domain->domain_state.working_counter) {
        //printf("--> check_domain_state: Domain: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain->domain_state.wc_state) {
        //printf("--> check_domain_state: Domain: State %u.\n", ds.wc_state);
    }

    domain->domain_state = ds;
}

/**
 * 计算两个timespec结构体相加的结果
 */
struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/**
 * 检查主站状态
 */
void check_master_state(ec_master_t* master, ec_master_state_t master_state)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
    //    printf("--> check_master_state: %u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
    //    printf("--> check_master_state: AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
     //   printf("--> check_master_state: Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

/**
 * 检查从站配置状态
 */
void check_slave_config_state(struct _SlaveConfig *slave_config, int SLAVE_NUM)
{
    ec_slave_config_state_t s;
    int i;

    for (i = 0; i < SLAVE_NUM; i++) {
        memset(&s, 0, sizeof(s));

        ecrt_slave_config_state(slave_config[i].sc, &s);

        if (s.al_state != slave_config[i].sc_state.al_state) {
            printf("--> check_slave_config_state: slave[%d]: State 0x%02X.\n", i, s.al_state);
        }

        if (s.online != slave_config[i].sc_state.online) {
            printf("--> check_slave_config_state: slave[%d]: %s.\n", i, s.online ? "online" : "offline");
        }

        if (s.operational != slave_config[i].sc_state.operational) {
            printf("--> check_slave_config_state: slave[%d]: %soperational.\n", i, s.operational ? "" : "Not ");
        }

        slave_config[i].sc_state = s;
    }
}