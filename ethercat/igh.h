#include "ecrt.h"
#include "timesync.h"
/****************************************************************************/

#define CLOCK_TO_USE CLOCK_REALTIME // 使用的时钟类型为实时时钟
#define MEASURE_TIMING // 启用计时测量

/****************************************************************************/


#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
    (B).tv_nsec - (A).tv_nsec) // 计算两个timespec结构体之间的纳秒差异

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec) // 将timespec结构体转换为纳秒

/****************************************************************************/

/*****************************从站信息表***************************************/
struct _SlaveInfo {
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;
};
/*****************************从站信息表***************************************/

//电机配置结构体
struct _SlaveConfig {
    ec_slave_config_t *sc; // 从站配置
    ec_slave_config_state_t sc_state; // 从站状态
};
/* Domain结构体 */
struct _Domain {
    ec_domain_t *domain; // Domain
    ec_domain_state_t domain_state; // Domain状态

    uint8_t *domain_pd; // Domain的过程数据
};


void check_domain_state(struct _Domain *domain);
struct timespec timespec_add(struct timespec time1, struct timespec time2);
void check_master_state(ec_master_t* master, ec_master_state_t master_state);
void check_slave_config_state(struct _SlaveConfig *slave_config, int SLAVE_NUM);
void* ethercatMaster(void* arg);