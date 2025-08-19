// term.h

#ifndef TERM_H
#define TERM_H

#include <pthread.h>
#include "motor.h"


/**************************** 电机命令 ****************************/
typedef struct {
    int target_pos[MOTOR_NUM];
    int actual_pos[MOTOR_NUM];
    int plan_pos[MOTOR_NUM];
    int previous_mode;
    int mode;
    int state[MOTOR_NUM];
} MotorSet;

typedef struct {
    double x;
    double y;
    double z;
    double Rx;
    double Ry;
    double Rz;
} Pose;
// 队列接口
void updateStatues(MotorSet *cmd);
int term_init(void);
#endif // TERM_H
