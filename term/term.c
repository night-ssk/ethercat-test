// term.c

#include "term.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// 全局命令队列
CommandQueue commandQueue;

// 初始化队列
void initQueue(CommandQueue* queue) {
    queue->front = queue->rear = NULL;
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->cond, NULL);
}

// 入队
void enqueue(CommandQueue* queue, MotorCommandSet* commandSet) {
    CommandNode* newNode = (CommandNode*)malloc(sizeof(CommandNode));
    if (!newNode) {
        fprintf(stderr, "内存分配失败\n");
        return;
    }
    newNode->commandSet = *commandSet;
    newNode->next = NULL;

    pthread_mutex_lock(&queue->mutex);
    if (queue->rear == NULL) {
        queue->front = queue->rear = newNode;
    } else {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    pthread_cond_signal(&queue->cond); // 通知等待的线程
    pthread_mutex_unlock(&queue->mutex);
}

// 非阻塞出队
int dequeue_non_blocking(CommandQueue* queue, MotorCommandSet* commandSet) {
    int ret = 0;
    pthread_mutex_lock(&queue->mutex);
    if (queue->front != NULL) {
        CommandNode* temp = queue->front;
        *commandSet = temp->commandSet;
        queue->front = queue->front->next;
        if (queue->front == NULL) {
            queue->rear = NULL;
        }
        free(temp);
        ret = 1;
    }
    pthread_mutex_unlock(&queue->mutex);
    return ret;
}

// 读取命令的线程函数，从文件中读取命令
void* readCommands(void* arg) {
    char* filename = (char*)arg;
    printf("readCommands 接收到的文件名: %s\n", filename); // 调试输出

    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "无法打开文件: %s\n", filename);
        return NULL;
    }

    char input[1024];
    // 初始化队列
    initQueue(&commandQueue);

    // 按行读取文件
    while (fgets(input, sizeof(input), file) != NULL) {
        // 去除换行符
        input[strcspn(input, "\n")] = 0;

        // 处理位置命令，例如: pp 0 -14 -27 ...
        if (strncmp(input, "pp", 2) == 0) {
            MotorCommandSet cmdSet;
            memset(&cmdSet, 0, sizeof(MotorCommandSet));

            // 解析命令
            char* token = strtok(input, " ");
            if (token == NULL || strcmp(token, "pp") != 0) {
                fprintf(stderr, "无效的pos命令: %s\n", input);
                continue;
            }

            // 设置所有电机的模式
            for(int i = 0; i < MOTOR_NUM; i++) {
                cmdSet.commands[i].mode = MODEL_PP;
            }
            // 依次解析所有目标位置
            int sufficientParams = 1;
            for (int i = 0; i < MOTOR_NUM; i++) {
                token = strtok(NULL, " ");
                if (token == NULL) {
                    fprintf(stderr, "位置命令参数不足，电机数量为 %d\n", MOTOR_NUM);
                    sufficientParams = 0;
                    break;
                }
                cmdSet.commands[i].target_pos = atoi(token);
                // 保持模式不变，或根据需要设置
            }

            if (sufficientParams) {
                enqueue(&commandQueue, &cmdSet);

                // 保持250Hz频率，每4毫秒设置一次位置
                usleep(4000); // 4000微秒 = 4毫秒
            }
        }
        // 处理模式命令，例如: mode home
        else if (strncmp(input, "home", 4) == 0) {
            MotorCommandSet cmdSet;
            memset(&cmdSet, 0, sizeof(MotorCommandSet));
            // 设置所有电机的模式
            for(int i = 0; i < MOTOR_NUM; i++) {
                cmdSet.commands[i].mode = MODEL_HOME;
            }
            enqueue(&commandQueue, &cmdSet);
            //延时10s
            sleep(20);
        }
    }
    fclose(file);
    printf("命令文件读取完成。\n");
    return NULL;
}
