// motor.c

#include "igh.h"
#include "motor.h"
#include "term.h"
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

// 引入命令队列
extern CommandQueue commandQueue;

// 定义当前命令集
MotorCommandSet currentCommandSet;


/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_motor_pdo_entries[] = {
    {0x6040, 0x00, 16}, // 控制字 U16
    {0x6060, 0x00, 8},  // 操作模式 I8
    {0x607a, 0x00, 32}, // 目标位置 S32
    // {0x6092, 0x01, 32}, // 细分数 U32
    {0x6098, 0x00, 8},  // 回零方式 I8
    {0x6099, 0x01, 32}, // 回零速度-快 U32
    {0x6099, 0x02, 32}, // 回零速度-慢 U32
    {0x609A, 0x00, 32}, // 回零加速度 U32
    {0x6081, 0x00, 32}, // 轮廓速度
    {0x6083, 0x00, 32}, // 轮廓加速度
    {0x6084, 0x00, 32}, // 轮廓减速度

    {0x6041, 0x00, 16}, // 状态字 U16
    {0x6061, 0x00, 8},  // 操作模式显示 I8
    {0x6064, 0x00, 32}, // 当前位置 S32
};

ec_pdo_info_t slave_motor_pdos[] = {
    {0x1600, 10, slave_motor_pdo_entries + 0},  // RXPDO
    {0x1a00, 3, slave_motor_pdo_entries + 10}, // TXPDO
};

ec_sync_info_t slave_motor_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_motor_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_motor_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
/**********************CStruct**********************************/

// 电机回零
void homeing(struct _Domain* domain, int* motorMode, int motorID) {
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字
    if (operation_mode_display == MODEL_HOME && (status & 0xFF) == 0x37) {
        if ((status & 0xFF00) == 0x0600) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x1F); // 启动运行
        }
        else if ((status & 0xFF00) == 0x1600) { // 到位
            printf("电机%d回零完成\n", motorID);
            *motorMode = MODEL_NONE;
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 重置
            return;
        }
    }
    else {
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_HOME); // 设置操作模式
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].home_way, 24);                // 设置回零方式
        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].home_spd_high, 20000);      // 设置回零速度-快
        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].home_spd_low, 2000);        // 设置回零速度-慢
        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].home_acc, 200000);         // 设置回零加速度

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
                printf("电机%d开始回零运动\n", motorID);

    }
}

// 电机位置控制pp模式
void pp(struct _Domain* domain, int target_pos, int motorID) {
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字
    if (operation_mode_display == MODEL_PP && (status & 0xFF) == 0x37) {
        if ((status & 0x1000) == 0) { // 到位 0x0400
            EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x3f);       // 重置
        }
        else if (status & 0x1000) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x2f); // 重置
        }
        // EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
        // EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x3f);       // 重置
        // EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x2f); // 重置
    }
    else {
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_PP); // 设置操作模式

        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].profile_velocity, 80000); // 设置轮廓速度
        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].profile_acc, 1000000);   // 设置轮廓加速度
        EC_WRITE_U32(domain->domain_pd + motor_parm[motorID].profile_dec, 1000000);   // 设置轮廓减速度

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x2f); // 重置
        printf("电机%d开始位置控制\n", motorID);
    }
}

// 电机位置控制CSP模式
void csp(struct _Domain* domain, int target_pos, int motorID) {
    static int times = 0;
    int8_t operation_mode_display = EC_READ_U16(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字

    EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置

    if (operation_mode_display != MODEL_CSP) {
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_CSP); // 设置操作模式
    }

    if (times == 1000) {
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        printf("电机%d得电\n", motorID);
    }
    if (times == 2000) {
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机使能
        printf("电机%d使能\n", motorID);
    }
    if (times == 3000) {
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
        printf("电机%d重置\n", motorID);
    }
    if (times > 4000) {
        EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
    }
    times++;
}

// 主电机控制函数
void motor_main(struct _Domain* domain) {
    static MotorCommandSet cmdSet;
    static int times = 0;
    times++;

    // 执行命令集
    for (int i = 0; i < MOTOR_NUM; i++) {
        uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[i].status_word); // 读取状态字
        if ((status & 0xFF) == 0x37) { // 电机初始化完成
            // 从队列中获取一个命令集（非阻塞）
            dequeue_non_blocking(&commandQueue, &cmdSet);
            // 应用命令集到当前命令集
            currentCommandSet = cmdSet;
            if (currentCommandSet.commands[i].mode == MODEL_HOME) {
                homeing(domain, &cmdSet.commands[i].mode, i);
            } 
            else if (currentCommandSet.commands[i].mode == MODEL_CSP) {
                csp(domain, currentCommandSet.commands[i].target_pos, i);
            }
            else if (currentCommandSet.commands[i].mode == MODEL_PP) {
                pp(domain, currentCommandSet.commands[i].target_pos, i);
            }

            printf("time:%d ms target_pos:%d actual_pos:%d \n", times * 4,  currentCommandSet.commands[i].target_pos, EC_READ_S32(domain->domain_pd + motor_parm[i].current_pos)); // 读取当前位置
        }
        else if ((status & 0xFF) == 0x50) { // 初始状态设置操作模式、加减速
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机得电
        } 
        else if ((status & 0xFF) == 0x31) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x07); // 电机使能
        } 
        else if ((status & 0xFF) == 0x33) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x0f); // 电机启动
        }
    }
}
