// motor.c

#include "ecrt.h"
#include "igh.h"
#include "motor.h"
#include "term.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "mqtt_comm.h"
#include "front.h"

static const int NORMAL_THETA1_MIN = 138720;
static const int NORMAL_THETA1_MAX = 261784;
// 引入命令队列
extern pthread_mutex_t cmdSet_mutex;
extern MotorSet cmdSet;

/* 转换比例 */
static const double POSITION_PULSE_PER_M         = 20000;       // 位移：1脉冲对应50nm
static const double ROTATIONAL_PULSE_PER_DEG_DD    = 1.0 / 0.001;       // 力矩电机：1脉冲对应0.001°
static const double ROTATIONAL_PULSE_PER_DEG_ZOLIX = 1.0 / 0.00055277122; // 卓立汉光电机：1脉冲对应0.00055277122°
/* 各轴零点脉冲数 */
static const int ZERO_PULSE_THETA6 = 0;   // 旋转关节6
static const int ZERO_PULSE_THETA5 = 71158;  // 旋转关节5
static const int ZERO_PULSE_D4     = 795823;  // 位移关节4
static const int ZERO_PULSE_D3     = 703125;  // 位移关节3
static const int ZERO_PULSE_D2     = 705678;  // 位移关节2
static const int ZERO_PULSE_THETA1 = 181148;   // 旋转关节1
/**********************CStruct**********************************/
ec_pdo_entry_info_t slave_motor_pdo_entries[] = {
    {0x6040, 0x00, 16}, // 控制字 U16
    {0x6060, 0x00, 8},  // 操作模式 I8
    {0x607a, 0x00, 32}, // 目标位置 S32

    {0x6041, 0x00, 16}, // 状态字 U16
    {0x6061, 0x00, 8},  // 操作模式显示 I8
    {0x6064, 0x00, 32}, // 当前位置 S32
};

ec_pdo_info_t slave_motor_pdos[] = {
    {0x1600, 3, slave_motor_pdo_entries + 0},  // RXPDO
    {0x1a00, 3, slave_motor_pdo_entries + 3}, // TXPDO
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
void homeing(struct _SlaveConfig *slave_config, struct _Domain* domain, int* motorMode, int motorID, int* state) {
    int8_t operation_mode_display = EC_READ_S8(domain->domain_pd + motor_parm[motorID].operation_mode_display); // 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);             // 读取状态字
    if (operation_mode_display == MODEL_HOME && (status & 0xFF) == 0x37) {
        if ((status & 0xFF00) == 0x0600) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x1F); // 启动运行
        }
        else if ((status & 0xFF00) == 0x1600) { // 到位
            printf("电机%d回零完成\n", motorID);
            *motorMode = MODEL_CSP;
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 重置
            return;
        }
    }
    else { //标记：没有根据state启动回零
        EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_HOME); // 设置操作模式

        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
        EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 重置
        printf("电机%d开始回零运动\n", motorID);
    }
}


// 电机位置控制CSP模式
void csp(struct _SlaveConfig *slave_config, struct _Domain* domain, int target_pos, int motorID, int* state) {
    int8_t operation_mode_display = EC_READ_S8(domain->domain_pd + motor_parm[motorID].operation_mode_display);// 读取状态字
    uint16_t status = EC_READ_U16(domain->domain_pd + motor_parm[motorID].status_word);// 读取状态字
    if (*state < 10) {
        if (*state == 0) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x06); // 重置电机
            if((status & 0xFF) == 0x31) {
                *state = 1;
            }
        } else if(*state == 1) {
            EC_WRITE_S8(domain->domain_pd + motor_parm[motorID].operation_mode, MODEL_CSP); // 设置操作模式
            if (operation_mode_display == MODEL_CSP) {
                *state = 2;
            }
        }  else if (*state == 2) { //标记：模式切换位置的影响
            int actual_motor = EC_READ_S32(domain->domain_pd + motor_parm[motorID].current_pos);
            int target_motor = EC_READ_S32(domain->domain_pd + motor_parm[motorID].target_pos);
            EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, actual_motor); // 设置目标位置
            int diff1 = target_motor - actual_motor;
            int diff2 = target_pos - actual_motor;
            if(-100 < diff1 && diff1 < 100 && -3000 < diff2 && diff2 < 3000) {
                *state = 3;
            }
        } else if (*state == 3) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x07); // 电机得电
            if((status & 0xFF) == 0x33) {
                *state = 4;
            }
        } else if (*state == 4) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[motorID].ctrl_word, 0x0f); // 使能电机
            if((status & 0xFF) == 0x37) {
                *state = 10;
            }
        }
    } else {
        EC_WRITE_S32(domain->domain_pd + motor_parm[motorID].target_pos, target_pos); // 设置目标位置
    }
}
// 主电机控制函数
void motor_main(struct _SlaveConfig *slave_config, struct _Domain* domain) {
    // 上锁
    pthread_mutex_lock(&cmdSet_mutex);

    /************************** 读取电机状态 **************************/ 
    for (int i = 0; i < MOTOR_NUM; i++) {
        cmdSet.actual_pos[i] = EC_READ_S32(domain->domain_pd + motor_parm[i].current_pos);
    }

    g_robotStateData.PositionAxis[5] = (cmdSet.actual_pos[0] - ZERO_PULSE_THETA6) / ROTATIONAL_PULSE_PER_DEG_ZOLIX;
    g_robotStateData.PositionAxis[4] = (cmdSet.actual_pos[1] - ZERO_PULSE_THETA5) / ROTATIONAL_PULSE_PER_DEG_DD;
    g_robotStateData.PositionAxis[3] = (cmdSet.actual_pos[2] - ZERO_PULSE_D4) / POSITION_PULSE_PER_M;
    g_robotStateData.PositionAxis[2] = (cmdSet.actual_pos[3] - ZERO_PULSE_D3) / POSITION_PULSE_PER_M;
    g_robotStateData.PositionAxis[1] = (cmdSet.actual_pos[4] - ZERO_PULSE_D2) / POSITION_PULSE_PER_M;
    g_robotStateData.PositionAxis[0] = (cmdSet.actual_pos[5] - ZERO_PULSE_THETA1) / ROTATIONAL_PULSE_PER_DEG_DD;

    forward_kinematics_from_pulses(
        cmdSet.actual_pos[5], cmdSet.actual_pos[4], cmdSet.actual_pos[3],\
cmdSet.actual_pos[2], cmdSet.actual_pos[1], cmdSet.actual_pos[0], \
    &g_robotStateData.PositionXYZ[0], &g_robotStateData.PositionXYZ[1], &g_robotStateData.PositionXYZ[2], \
        &g_robotStateData.PositionXYZ[3], &g_robotStateData.PositionXYZ[4], &g_robotStateData.PositionXYZ[5]);

    /************************** 初始化电机 **************************/ 
    static int init_flag = 0;
    if (init_flag < 10) {
        if (init_flag == 0) {
            int8_t operation_mode_display = 0;
            bool motor_start_flag = true;
            for(int i = 1; i < MOTOR_NUM; i++) {
                operation_mode_display = EC_READ_S8(domain->domain_pd + motor_parm[i].operation_mode_display);// 读取状态字
                //printf("operation_mode_display: %d,id: %d\n", operation_mode_display, i);
                if (operation_mode_display != 1 && operation_mode_display != 8) {
                    motor_start_flag = false; // 只要有一个不是 1，就设为 false
                    break; // 直接退出循环，提高效率
                }
            }
            if (motor_start_flag) {
                init_flag = 1;
            }
        }
        if (init_flag == 1) { // 设置目标位置
            int theta1_init = EC_READ_S32(domain->domain_pd + motor_parm[5].current_pos);
            if (theta1_init < NORMAL_THETA1_MIN || theta1_init > NORMAL_THETA1_MAX) {  /////标记
                g_robotStateData.taskState = 203; // 范围错误
            }

            for (int i = 0; i < MOTOR_NUM; i++) {
                cmdSet.plan_pos[i] = EC_READ_S32(domain->domain_pd + motor_parm[i].current_pos);
            }
            init_flag = 2;
        } else if (init_flag == 2) { // 执行第一次初始化
            for (int i = 0; i < MOTOR_NUM; i++) {
                csp(slave_config, domain, cmdSet.plan_pos[i], i, &cmdSet.state[i]);
            }
            if (cmdSet.state[0] == 10 && cmdSet.state[1] == 10 && cmdSet.state[2] == 10 && cmdSet.state[3] == 10 && cmdSet.state[4] == 10 && cmdSet.state[5] == 10) {
                init_flag = 10;
                g_robotStateData.taskState = 1;//初始化完成,可以接收任务
                memcpy(cmdSet.plan_pos, cmdSet.actual_pos, sizeof(cmdSet.plan_pos));
            }
       }
    }

    // /************************** 电机控制 **************************/ 
    // 电机失能
    if(g_robotStateData.taskState > 200) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            EC_WRITE_U16(domain->domain_pd + motor_parm[i].ctrl_word, 0x06); // 电机失能
        }
        return;
    }
    updateStatues(&cmdSet);// 切换模式时，只重置卓立汉光电机
    // 电机控制
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (i != 0) {   // 大族电机
            csp(slave_config, domain, cmdSet.plan_pos[i], i, &cmdSet.state[i]);
        } else  { // 卓立汉光
            if (cmdSet.mode == MODEL_HOME) {
                homeing(slave_config, domain, &cmdSet.mode, i, &cmdSet.state[i]);
            } else if (cmdSet.mode == MODEL_CSP) {
                csp(slave_config, domain, cmdSet.plan_pos[i], i, &cmdSet.state[i]);
            }
        }
    }

    //解锁
    pthread_mutex_unlock(&cmdSet_mutex);
}
