// term.h

#ifndef TERM_H
#define TERM_H

#include <pthread.h>
#include "motor.h"
typedef struct {
    int target_pos;
    int mode;
} MotorCommand;

// 定义一个命令集，包含所有电机的命令
typedef struct {
    MotorCommand commands[MOTOR_NUM];
} MotorCommandSet;

// 命令队列节点
typedef struct CommandNode {
    MotorCommandSet commandSet;
    struct CommandNode* next;
} CommandNode;

// 命令队列结构体
typedef struct {
    CommandNode* front;
    CommandNode* rear;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} CommandQueue;

// 队列接口
void initQueue(CommandQueue* queue);
void enqueue(CommandQueue* queue, MotorCommandSet* commandSet);
int dequeue_non_blocking(CommandQueue* queue, MotorCommandSet* commandSet);

void* readCommands(void* arg);

#endif // TERM_H
