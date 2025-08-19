// term.c

#include "term.h"
#include "motor.h"
#include "front.h"
#include "back.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "mqtt_comm.h"
#include "myprint.h"

/* 各轴零点脉冲数 */
static const int ZERO_PULSE_THETA6 = 0;   // 旋转关节6
static const int ZERO_PULSE_THETA5 = 71158;  // 旋转关节5
static const int ZERO_PULSE_D4     = 795823;  // 位移关节4
static const int ZERO_PULSE_D3     = 703125;  // 位移关节3
static const int ZERO_PULSE_D2     = 705678;  // 位移关节2
static const int ZERO_PULSE_THETA1 = 181148;   // 旋转关节1
/* 电机脉冲限位 (顺序：theta6, theta5, d4, d3, d2, theta1) */
static const int LIMIT_THETA6_MIN = -11500; // theta6限位范围需要调整
static const int LIMIT_THETA6_MAX = 26000;

static const int LIMIT_THETA5_MIN = -18000;
static const int LIMIT_THETA5_MAX = 160000;

static const int LIMIT_D4_MIN = 140000;
static const int LIMIT_D4_MAX = 1560000;

static const int LIMIT_D3_MIN = 30000;
static const int LIMIT_D3_MAX = 1370000;

static const int LIMIT_D2_MIN = 30000;
static const int LIMIT_D2_MAX = 1370000;

static const int LIMIT_THETA1_MIN = 22853;
static const int LIMIT_THETA1_MAX = 261784;

/* 转换比例 */
static const double POSITION_PULSE_PER_M         = 20000;       // 位移：1脉冲对应50nm
static const double ROTATIONAL_PULSE_PER_DEG_DD    = 1.0 / 0.001;       // 力矩电机：1脉冲对应0.001°
static const double ROTATIONAL_PULSE_PER_DEG_ZOLIX = 1.0 / 0.00055277122; // 卓立汉光电机：1脉冲对应0.00055277122°
// 全局控制命令
MotorSet cmdSet = {
        {0, 0, 0, 0, 0, 0},  
        {0, 0, 0, 0, 0, 0},  
        {0, 0, 0, 0, 0, 0},
        MODEL_CSP, 
        MODEL_CSP,
        {0, 0, 0, 0, 0, 0}
};
pthread_mutex_t cmdSet_mutex = PTHREAD_MUTEX_INITIALIZER;
// mqtt数据
extern RobotControlData g_robotControlData;
extern RobotStateData   g_robotStateData;
// 模式切换时更新状态
void updateStatues(MotorSet *cmd) {
    if (cmd->mode != cmd->previous_mode) {
        cmd->state[0] = 0;
        cmd->previous_mode = cmd->mode; // 更新 previous_mode
    }
}
//正运动学转换
int posconvert(const RobotControlData *recv, int *outpos) {
    int ret = 0;
    // 逆运动学求解
    Pose p = {recv->AbsolutePose[0], recv->AbsolutePose[1], recv->AbsolutePose[2], \
             recv->AbsolutePose[3], recv->AbsolutePose[4], recv->AbsolutePose[5]};

    outpos[0] = recv->Delta[0] * ROTATIONAL_PULSE_PER_DEG_DD + ZERO_PULSE_THETA1;
    outpos[1] = recv->Delta[1] * POSITION_PULSE_PER_M + ZERO_PULSE_D2;
    outpos[2] = recv->Delta[2] * POSITION_PULSE_PER_M + ZERO_PULSE_D3;
    outpos[3] = recv->Delta[3] * POSITION_PULSE_PER_M + ZERO_PULSE_D4;
    outpos[4] = recv->Delta[4] * ROTATIONAL_PULSE_PER_DEG_DD + ZERO_PULSE_THETA5;
    outpos[5] = recv->Delta[5] * ROTATIONAL_PULSE_PER_DEG_ZOLIX + ZERO_PULSE_THETA6;
    if(recv->taskType == 2) { // 末端位置控制
        inverse_kinematics(p.x, p.y, p.z, p.Rx, p.Ry, p.Rz,
                &outpos[0], &outpos[1], &outpos[2], &outpos[3], &outpos[4], &outpos[5]);
    }
    /************* 交换顺序 *************/
    int temp;
    for (int i = 0; i < 3; i++) { 
        temp = outpos[i];
        outpos[i] = outpos[5 - i];
        outpos[5 - i] = temp;
    }

    /************* 限位判断 *************/
    if (outpos[0] < LIMIT_THETA6_MIN || outpos[0] > LIMIT_THETA6_MAX) {
        ret = 1;
    } else if (outpos[1] < LIMIT_THETA5_MIN || outpos[1] > LIMIT_THETA5_MAX) {
        ret = 1;
    } else if (outpos[2] < LIMIT_D4_MIN || outpos[2] > LIMIT_D4_MAX) {
        ret = 1;
    } else if (outpos[3] < LIMIT_D3_MIN || outpos[3] > LIMIT_D3_MAX) {
        ret = 1;
    } else if (outpos[4] < LIMIT_D2_MIN || outpos[4] > LIMIT_D2_MAX) {
        ret = 1;
    } else if (outpos[5] < LIMIT_THETA1_MIN || outpos[5] > LIMIT_THETA1_MAX) {
        ret = 1;
    }

    return ret;
}

// 根据状态设置电机
int SetHome(int *target_pos) {
    target_pos[0] = ZERO_PULSE_THETA6;
    target_pos[1] = ZERO_PULSE_THETA5;
    target_pos[2] = ZERO_PULSE_D4;
    target_pos[3] = ZERO_PULSE_D3;
    target_pos[4] = ZERO_PULSE_D2;
    target_pos[5] = ZERO_PULSE_THETA1;

    return *target_pos;
}
// 设置新目标状态

// 斜坡函数平滑控制
int updateSetPos(int setpos, int target_pos, float max_speed) {
    int diff = target_pos - setpos;         // 计算目标与当前位置的差
    // 如果差距小于步长，直接到达目标
    if (abs(diff) <= max_speed)
        return target_pos;
    
    // 向目标方向移动一个步长
    if (diff > 0)
        return setpos + max_speed;
    else
        return setpos - max_speed;
}

int SetMotor(RobotControlData *recv) {
    int ret = 0;
    // 上锁
    if (recv->taskType == 3) { // home模式
        SetHome(cmdSet.target_pos); // 设置目标位置为0
        cmdSet.plan_pos[0] = ZERO_PULSE_THETA6;// 标记，这里是为了让回零后不再运动
        cmdSet.mode = MODEL_HOME;
    } else if (recv->taskType == 2 || recv->taskType == 1) { // 末端位置模式
        ret = posconvert(recv, cmdSet.target_pos); // 逆运动学求解
        cmdSet.mode = MODEL_CSP;
    } else;
    return ret;
    // 解锁
}
void driveMotor(MotorSet *cmd) {
    // cmdSet.plan_pos[0] = updateSetPos(cmdSet.plan_pos[0], cmdSet.target_pos[0], 22); // 电机速度 3度/s
    // cmdSet.plan_pos[1] = updateSetPos(cmdSet.plan_pos[1], cmdSet.target_pos[1], 12); // 电机速度 3度/s
    // cmdSet.plan_pos[2] = updateSetPos(cmdSet.plan_pos[2], cmdSet.target_pos[2], 400);// 电机速度 5mm/s
    // cmdSet.plan_pos[3] = updateSetPos(cmdSet.plan_pos[3], cmdSet.target_pos[3], 400);// 电机速度 5mm/s
    // cmdSet.plan_pos[4] = updateSetPos(cmdSet.plan_pos[4], cmdSet.target_pos[4], 400);// 电机速度 5mm/s
    // cmdSet.plan_pos[5] = updateSetPos(cmdSet.plan_pos[5], cmdSet.target_pos[5], 12); // 电机速度 3度/s
    if(g_robotControlData.step > 50) {
        g_robotControlData.step = 50;
    }else if (g_robotControlData.step < 10) {
        g_robotControlData.step = 10;
    }
    cmdSet.plan_pos[0] = updateSetPos(cmdSet.plan_pos[0], cmdSet.target_pos[0], 2 * 0.1f * g_robotControlData.step); // 电机速度 3度/s
    cmdSet.plan_pos[1] = updateSetPos(cmdSet.plan_pos[1], cmdSet.target_pos[1], 1 * 0.1f * g_robotControlData.step); // 电机速度 3度/s
    cmdSet.plan_pos[2] = updateSetPos(cmdSet.plan_pos[2], cmdSet.target_pos[2], 40 * 0.1f * g_robotControlData.step);// 电机速度 5mm/s
    cmdSet.plan_pos[3] = updateSetPos(cmdSet.plan_pos[3], cmdSet.target_pos[3], 40 * 0.1f * g_robotControlData.step);// 电机速度 5mm/s
    cmdSet.plan_pos[4] = updateSetPos(cmdSet.plan_pos[4], cmdSet.target_pos[4], 40 * 0.1f * g_robotControlData.step);// 电机速度 5mm/s
    cmdSet.plan_pos[5] = updateSetPos(cmdSet.plan_pos[5], cmdSet.target_pos[5], 2 * 0.1f * g_robotControlData.step); // 电机速度 3度/s

}
void judge_task(RobotStateData *state) {//
    if (cmdSet.mode == MODEL_CSP && (memcmp(cmdSet.plan_pos, cmdSet.target_pos, sizeof(cmdSet.plan_pos)) == 0)) { // 模式切换到了CSP且到达零点
        state->taskState = 1;
    }
}

void* process_main(void* arg){
    while(1){
    //print_robot_control_data(&g_robotControlData);
    //printMotorSet(&cmdSet);
    //printf("taskState: %d\n", g_robotStateData.taskState);
/********************* 接收任务 ********************/
        if (g_robotControlData.taskType == 4) { // home模式
            g_robotStateData.taskState = 201; // 任务急停
        }
        if(g_robotStateData.taskState == 1 || g_robotStateData.taskState == 101) { // 任务执行完成可以接收任务、命令警告也可以重新获取任务
            if(g_robotControlData.taskNum != g_robotStateData.taskNum) { // 新任务
                // 任务接收
                g_robotStateData.taskState = 0; //开始执行新任务
                g_robotStateData.taskNum = g_robotControlData.taskNum; // 更新任务编号
                pthread_mutex_lock(&cmdSet_mutex);
                int ret = SetMotor(&g_robotControlData); // 配置电机
                pthread_mutex_unlock(&cmdSet_mutex);
                if(ret != 0) { // 逆运动学求解超出范围
                    g_robotStateData.taskState = 101; // 指令位置超过了量程
                }
            }
        } else if(g_robotStateData.taskState == 0) { //任务执行中
            // 任务执行
            pthread_mutex_lock(&cmdSet_mutex);
            driveMotor(&cmdSet);
            pthread_mutex_unlock(&cmdSet_mutex);
            // 任务完成判断
            judge_task(&g_robotStateData);
        } else if (g_robotStateData.taskState == 201 || g_robotStateData.taskState == 202) { // 任务急停
            break; // 错误退出
        }else;
        usleep(4000);
    }
    return NULL;
}

int term_init() {
    pthread_t process_thread;
    pthread_create(&process_thread, NULL, process_main, NULL);
	pthread_detach(process_thread);
    return 0;
}
