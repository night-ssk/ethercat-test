#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include "igh.h"
#include "motor.h"
#include "timesync.h"
#include "term.h"
/**************************命令行信息*****************************/
// 全局静态变量
extern MotorCommand currentCommand[5];
// 互斥锁
extern pthread_mutex_t cmd_lock;
int moveStartTime = 0;
/**************************命令行信息*****************************/

/************************设备信息*************************/

enum device_info
{
    motor1_alias = 0, motor1_position = 0, motor1_vid = 0x00000168, motor1_pid = 0x0000000a,
};
const static struct _SlaveInfo slave_info[] = {
    {motor1_alias, motor1_position, motor1_vid, motor1_pid}, //直线电机驱动器1
};
/************************设备信息*************************/

/*************************域空间*************************/
const static ec_pdo_entry_reg_t domain_regs[] = {
//直线电机1
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6060,0x00,&motor_parm[0].operation_mode},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6040,0x00,&motor_parm[0].ctrl_word}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6041,0x00,&motor_parm[0].status_word}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x607A,0x00,&motor_parm[0].target_pos}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6064,0x00,&motor_parm[0].current_pos}, 
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6081,0x00,&motor_parm[0].profile_velocity},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6083,0x00,&motor_parm[0].profile_acc},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6084,0x00,&motor_parm[0].profile_dec},
    // {motor1_alias, motor1_position, motor1_vid, motor1_pid,
    //     0x6092,0x01,&motor_parm[0].step_div},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6098,0x00,&motor_parm[0].home_way},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6099,0x01,&motor_parm[0].home_spd_high},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x6099,0x02,&motor_parm[0].home_spd_low},
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
        0x609A,0x00,&motor_parm[0].home_acc},   
    {motor1_alias, motor1_position, motor1_vid, motor1_pid,
    0x6061,0x00,&motor_parm[0].operation_mode_display},
    {},
};
struct _Domain _domain;
/*************************域空间*************************/
volatile bool g_quit = false;
const struct timespec cycletime = {0, PERIOD_NS};

/* EtherCAT 主站 */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static uint64_t dc_time_ns;
/**
 * 循环任务
 */
void cyclic_task(struct _SlaveConfig *slave_config, struct _Domain *domain)
{
    /* 用于确定状态字的值 */
    struct timespec wakeupTime;
    uint32_t count = 0;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    /* 获取当前时间 */
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    uint64_t wakeup_time = getSysTime() + 10 * PERIOD_NS;
    while (!g_quit) {
        sleepUntil(master, wakeup_time, &wakeup_time);
        /* 周期时间：1ms */
        // wakeupTime = timespec_add(wakeupTime, cycletime);
        // clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif
        /* 接收过程数据 */
        ecrt_master_receive(master);
        ecrt_domain_process(domain->domain);

        /* 检查过程数据状态（可选） */
        check_domain_state(domain);

        /* 检查主站状态（可选） */
        check_master_state(master,master_state);

        /* 检查从站配置状态（可选） */
        check_slave_config_state(slave_config,1);

#ifdef MEASURE_TIMING
        /* 不要频繁打印，否则会影响实时性能 */
        count++;
        if (count == FREQUENCY) {
            // 输出计时统计信息
            // printf("period     min: %10u(ns) ... max: %10u(ns)\n",
            //         period_min_ns, period_max_ns);
            // printf("exec       min: %10u(ns) ... max: %10u(ns)\n",
            //         exec_min_ns, exec_max_ns);
            // printf("latency    min: %10u(ns) ... max: %10u(ns)\n",
            //         latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;

            count = 0;
        }
#endif
    /* 电机控制 */
    motor_main(domain);

    ecrt_domain_queue(domain->domain);
    // 在 ecrt_master_send() 之前同步分布式时钟
    int32_t dc_diff_ns = syncDC(master, &dc_time_ns);
    // 发送所有队列中的数据包
    ecrt_master_send(master);
    // 更新主站时钟
    updateMasterClock(dc_diff_ns);

#ifdef MEASURE_TIMING
    clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}
/**
 * 主函数
 */
void* ethercatMaster(void* arg)
{
    int ret = -1;
    int SLAVE_NUM = sizeof(slave_info) / sizeof(struct _SlaveInfo);

    struct _SlaveConfig slave_config[SLAVE_NUM];
    memset(&slave_config, 0, sizeof(slave_config));
    memset(&_domain, 0, sizeof(_domain));

    /* 请求EtherCAT主站 */
    master = ecrt_request_master(0);
    if (!master) {
        printf("--> main: Request master failed.\n");
        return (void*)-1;
    }
    printf("--> main: Request master success.\n");


    /* 创建Domain */
    _domain.domain = ecrt_master_create_domain(master);

    if (!_domain.domain) {
        printf("--> main: Create domain failed.\n");
        goto err_leave;
    }
    printf("--> main: Create domain success.\n");

    /* 获取从站配置 ，slave_info从站信息*/
    /*  alias：从站别名、position：从站的位置、vendor_id：从站的供应商ID*/
    for (int i = 0; i < SLAVE_NUM; i++) {
        slave_config[i].sc = ecrt_master_slave_config(master, slave_info[i].alias,
                    slave_info[i].position, slave_info[i].vendor_id,
                    slave_info[i].product_code);
        if (!slave_config[i].sc) {
            printf("--> main: Get slave configuration failed.\n");
            goto err_leave;
        }
    }
    printf("--> main: Get slave configuration success.\n");

    /* 配置PDO *///配置mapping
    for (int i = 0; i < SLAVE_NUM; i++) {
        ret = ecrt_slave_config_pdos(slave_config[i].sc, EC_END, slave_motor_syncs);
        if (ret != 0) {
            printf("--> main: Configuration PDO failed.\n");
            goto err_leave;
        }
    }
    /* 注册PDO条目到Domain */
    ret = ecrt_domain_reg_pdo_entry_list(_domain.domain, domain_regs);
    if (ret != 0) {
        
        printf("--> main: Failed to register bunch of PDO entries for domain.\n");
        goto err_leave;
    }
    printf("--> main: Success to register bunch of PDO entries for domain.\n");
    
    dc_time_ns = getSysTime();
    // 为每个从站配置分布式时钟 DC
    for (uint32_t i = 0; i < SLAVE_NUM; i++)
    {
        ecrt_slave_config_sdo16(slave_config[i].sc, 0x1c32, 1, 2);
        ecrt_slave_config_sdo16(slave_config[i].sc, 0x1c33, 1, 2);
        ecrt_slave_config_dc(slave_config[i].sc, 0x0300, PERIOD_NS, PERIOD_NS / 4, 0, 0);
    }
    printf("Configuration DC success.");

    // 选择参考时钟（第一个从站）
    ret = ecrt_master_select_reference_clock(master, slave_config[0].sc);
    if (ret < 0)
    {
        printf("Failed to select reference clock: %s\n", strerror(-ret));
        goto err_leave;
    }

    /* 激活EtherCAT主站 */
    ret = ecrt_master_activate(master);
    if (ret < 0) {
        
        printf("--> main: Activate master failed.\n");
        goto err_leave;
    }
    printf("--> main: Activate master success.\n");

    /* 获取过程数据内存的指针 */
    _domain.domain_pd = ecrt_domain_data(_domain.domain);
    if (!_domain.domain_pd) {
        printf("--> main: Get pointer to the process data memory failed.\n");
        goto err_leave;
    }
    printf("--> main: Get pointer to the process data memory success.\n");

    /* 设置进程优先级 */
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19)) {
        printf("--> main: Warning: Failed to set priority: %s\n", strerror(errno));
    }
    /**/
    /* 开始循环任务 */
    printf("--> main: Enter cycle task now...\n");
    cyclic_task(slave_config, &_domain);
err_leave:
    /* 释放EtherCAT主站 */
    ecrt_release_master(master);
    printf("--> main: Release master.\n");
    return (void*)0;
}