#include "igh.h"
#include "motor.h"
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include "term.h"
// 全局静态变量
MotorCommand currentCommand[MOTOR_NUM];

// 互斥锁
pthread_mutex_t cmd_lock;

/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_motor_pdo_entries[] = {
    {0x6040, 0x00, 16}, //控制字 U16
    {0x6060, 0x00, 8},  //操作模式 I8
    {0x607a, 0x00, 32}, //目标位置 S32
    // {0x6092, 0x01, 32}, //细分数 U32
    {0x6098, 0x00, 8},  //回零方式 I8
    {0x6099, 0x01, 32}, //回零速度-快 U32
    {0x6099, 0x02, 32}, //回零速度-慢 U32
    {0x609A, 0x00, 32}, //回零加速度 U32
    {0x6081, 0x00, 32}, //轮廓速度
    {0x6083, 0x00, 32}, //轮廓加速度
    {0x6084, 0x00, 32}, //轮廓减速度

    {0x6041, 0x00, 16}, //状态字 U16
    {0x6061, 0x00, 8},  //操作模式显示 I8
    {0x6064, 0x00, 32}, //当前位置 S32ßß
};

ec_pdo_info_t slave_motor_pdos[] = {
    {0x1600, 10, slave_motor_pdo_entries + 0}, //RXPDO
    {0x1a00, 3, slave_motor_pdo_entries + 10}, //TXPDO
};

ec_sync_info_t slave_motor_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_motor_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_motor_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
/**********************CStruct**********************************/


/*
    * 电机回零
*/
void homeing(struct _Domain* domain, int* motorMode) {
    for (int i = 0; i < MOTOR_NUM; i++) {
        int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[i].operation_mode_display);// 读取状态字
        uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[i].status_word);// 读取状态字
        if(operation_mode_display == MODEL_HOME && (status & 0xFF) == 0x37)
        {
            if((status & 0xFF00) == 0x0600)
            {
                EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x1F); // 启动运行
            }
            else if((status & 0xFF00) == 0x1600)// 到位
            {
                printf("电机%d回零完成\n", i);
                pthread_mutex_lock(&cmd_lock); // 加锁
                *motorMode = MODEL_NONE;
                pthread_mutex_unlock(&cmd_lock); // 解锁
                EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 重置
                return;
            }
        }
        else
        {
            EC_WRITE_S8(domain->domain_pd + motor_parm[i].operation_mode, MODEL_HOME); // 设置操作模式
            EC_WRITE_S8(domain->domain_pd + motor_parm[i].home_way, 24); // 设置回零方式
            EC_WRITE_U32(domain->domain_pd + motor_parm[i].home_spd_high, 20000); // 设置回零速度-快
            EC_WRITE_U32(domain->domain_pd + motor_parm[i].home_spd_low, 2000); // 设置回零速度-慢
            EC_WRITE_U32(domain->domain_pd + motor_parm[i].home_acc, 200000); // 设置回零加速度

            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机得电
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 重置
        }
    }
}
/*
    * 电机位置控制pp模式
*/
void pp(struct _Domain* domain, int target_pos) {
    static int times = 0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[i].operation_mode_display);// 读取状态字
        uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[i].status_word);// 读取状态字
        if(operation_mode_display == MODEL_PP && (status & 0xFF) == 0x37)
        {
            if (status & 0x0400) // 到位 0x0400 
            {
                EC_WRITE_S32(domain->domain_pd + motor_parm[i].target_pos, target_pos); // 设置目标位置
                EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x3f); // 重置
                if(times++ % 500 == 0)
                printf("电机%d到位\n", i);
            }
            else if (status & 0x1000) {
                EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x2f); // 重置
            }
        }
        else
        {
            EC_WRITE_S8(domain->domain_pd + motor_parm[i].operation_mode, MODEL_PP); // 设置操作模式

            EC_WRITE_U32(domain->domain_pd + motor_parm[i].profile_velocity, 2000); // 设置轮廓速度
            EC_WRITE_U32(domain->domain_pd + motor_parm[i].profile_acc, 200000); // 设置轮廓加速度
            EC_WRITE_U32(domain->domain_pd + motor_parm[i].profile_dec, 200000); // 设置轮廓减速度

            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机得电
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 重置
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x2f); // 重置
        }
    }
}

/*
    * 电机位置控制CSP模式
*/
void csp(struct _Domain* domain, int target_pos) {
  //  static int times = 0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[i].operation_mode_display);// 读取状态字
        uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[i].status_word);// 读取状态字
        if(operation_mode_display == MODEL_CSP && (status & 0xFF) == 0x37){
            EC_WRITE_S32(domain->domain_pd + motor_parm[i].target_pos, target_pos); // 设置目标位置
        }else if(operation_mode_display == MODEL_CSP && (status & 0xFF) == 0x31) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机得电
        }else if(operation_mode_display == MODEL_CSP && (status & 0xFF) == 0x33) {
                    EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 重置
        }else {
            EC_WRITE_S8(domain->domain_pd + motor_parm[i].operation_mode, MODEL_CSP); // 设置操作模式
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
        }
        // if (times == 0) {
        //     EC_WRITE_S8(domain->domain_pd + motor_parm[i].operation_mode, MODEL_CSP); // 设置操作模式
        //     //延时delay
        // }else if (times == 500) {
        //     EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
        // }else if(times == 1000) {
        //     EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机得电
        // }else if(times == 1500) {
        //     EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 电机得电
        // }else if(times >= 1501) {
        //     EC_WRITE_S32(domain->domain_pd + motor_parm[i].target_pos, target_pos); // 设置目标位置
        //     // //根据系统时间计算发送频率
        //     // //获取时间  
        //     // static struct timespec ts_start, ts_end;
        //     // if (times == 4) {
        //     //     clock_gettime(CLOCK_REALTIME, &ts_start);
        //     // } if(times % 1000 == 0 && times > 4) {
        //     //     clock_gettime(CLOCK_REALTIME, &ts_end);
        //     //     long time = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 + (ts_end.tv_nsec - ts_start.tv_nsec) / 1e6;
        //     //     //频率计算
        //     //     int frequency = 1000 / (time / 1000000);
        //     //     printf("发送频率：%d\n", frequency);
        //     //     printf("发送时间：%ld\n", time);
        //     //     clock_gettime(CLOCK_REALTIME, &ts_start);
        //     // }
        // }
        // times++;
    }
}
void motor_main(struct _Domain* domain) {
    MotorCommand cmd[MOTOR_NUM];
    static int times = 0;
    times++;
    pthread_mutex_lock(&cmd_lock); // 加锁
    for (int i = 0; i < MOTOR_NUM; i++) {
        cmd[i] = currentCommand[i];
    }
    pthread_mutex_unlock(&cmd_lock); // 解锁
    for (int i = 0; i < MOTOR_NUM; i++) //初始化电机流程
    {
        uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[i].status_word);// 读取状态字
        if ((status & 0xFF) == 0x37) { // 电机初始化完成
            if (cmd[i].mode == MODEL_HOME) {
                homeing(domain, &currentCommand[i].mode);
            } else if (cmd[i].mode == MODEL_CSP) {
                csp(domain, cmd[i].target_pos);
            }else if (cmd[i].mode == MODEL_PP) {
                pp(domain, cmd[i].target_pos);
            }
            //打印电机位置
            if(times % 1000 == 0)
            {
                printf("电机%d当前位置：%d\n", i, EC_READ_S32(domain->domain_pd + motor_parm[i].current_pos));
            }
        }
        else if ((status & 0xFF) == 0x50){ // 初始状态设置操作模式、加减速
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
        } else if ((status & 0xFF) == 0x31) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机使能
        } else if ((status & 0xFF) == 0x33) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 电机启动
        }
    }
}

