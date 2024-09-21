#include "term.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "motor.h"
#include <unistd.h>

extern MotorCommand currentCommand[MOTOR_NUM];
extern pthread_mutex_t cmd_lock;

void setPos(int motorID, int distance) {
    if(motorID < 0 || motorID >= 6) 
    {
        printf("电机ID错误%d\n", motorID);
        return;
    }

    pthread_mutex_lock(&cmd_lock); // 加锁
    currentCommand[motorID].target_pos = distance;
    pthread_mutex_unlock(&cmd_lock); // 解锁
}
void setMode(int motorID, int mode) {
    if(motorID < 0 || motorID >= 6) 
    {
        printf("电机ID错误%d\n", motorID);
        return;
    }

    pthread_mutex_lock(&cmd_lock); // 加锁
    currentCommand[motorID].mode = mode;
    pthread_mutex_unlock(&cmd_lock); // 解锁
}

// void* readCommands(void* arg)
// {
//     char input[100];
//     for(int i = 0; i < MOTOR_NUM; i++) {
//         currentCommand[i].target_pos = 0;
//         currentCommand[i].mode = MODEL_NONE;
//     }
//     while (1) {
//         if (fgets(input, sizeof(input), stdin) == NULL) {
//             fprintf(stderr, "读取输入失败\n");
//             continue;
//         }
//         // 设置电机位置, pos id distance
//         if (strncmp(input, "pos", 3) == 0) {
//             int motorID, distance;
//             if (sscanf(input, "pos %d %d", &motorID, &distance) == 2) {
//                 setPos(motorID, distance);
//             } else {
//                 fprintf(stderr, "设置位置失败\n");
//             }
//         }
//         // 设置电机模式, mode  home/csp
//         else if (strncmp(input, "mode", 4) == 0) {
//             char mode[10];
//             if (sscanf(input, "mode %s", mode) == 1) {
//                 if (strcmp(mode, "home") == 0) {
//                     setMode(0, MODEL_HOME);
//                 } else if (strcmp(mode, "csp") == 0) {
//                     setMode(0, MODEL_CSP);
//                 }else if (strcmp(mode, "pp") == 0) {
//                     setMode(0, MODEL_PP);
//                 }else {
//                     fprintf(stderr, "设置模式失败\n");
//                     printf("mode %s\n", mode);
//                 }
//             } else {
//                 fprintf(stderr, "设置模式失败\n");
//                 printf("mode %s\n", mode);
//             }
//         }

//     }
    
//     return NULL;
// }

// 读取命令的线程函数，从文件中读取命令
void* readCommands(void* arg) {
    char* filename = (char*)arg;
    printf("readCommands 接收到的文件名: %s\n", filename); // 调试输出

    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "无法打开文件: %s\n", filename);
        return NULL;
    }

    char input[100];
    // 初始化 currentCommand
    pthread_mutex_lock(&cmd_lock);
    for(int i = 0; i < MOTOR_NUM; i++) {
        currentCommand[i].target_pos = 0;
        currentCommand[i].mode = MODEL_NONE;
    }
    pthread_mutex_unlock(&cmd_lock);

    // 按行读取文件
    while (fgets(input, sizeof(input), file) != NULL) {
        // 去除换行符
        input[strcspn(input, "\n")] = 0;

        // 处理位置命令，例如: pos 0 -14
        if (strncmp(input, "pos", 3) == 0) {
            int motorID, distance;
            if (sscanf(input, "pos %d %d", &motorID, &distance) == 2) {
                setPos(motorID, distance);
                printf("设置电机 %d 位置为 %d\n", motorID, distance);

                // 实时打印所有电机的目标位置和模式
                pthread_mutex_lock(&cmd_lock);
                for(int i = 0; i < MOTOR_NUM; i++) {
                    printf("电机 %d: 位置 = %d, 模式 = %d\n", i, currentCommand[i].target_pos, currentCommand[i].mode);
                }
                pthread_mutex_unlock(&cmd_lock);

                // 保持250Hz频率，每4毫秒设置一次位置
                usleep(4000); // 4000微秒 = 4毫秒
            } else {
                fprintf(stderr, "设置位置失败，命令格式错误: %s\n", input);
            }
        }
        // 处理模式命令，例如: mode home
        else if (strncmp(input, "mode", 4) == 0) {
            char modeStr[10];
            if (sscanf(input, "mode %s", modeStr) == 1) {
                int mode;
                // 假设所有模式都应用于电机ID 0，如果需要应用于不同电机，可以修改此处
                if (strcmp(modeStr, "home") == 0) {
                    mode = MODEL_HOME;
                } else if (strcmp(modeStr, "csp") == 0) {
                    mode = MODEL_CSP;
                } else if (strcmp(modeStr, "pp") == 0) {
                    mode = MODEL_PP;
                } else {
                    fprintf(stderr, "未知的模式: %s\n", modeStr);
                    continue;
                }
                setMode(0, mode);
                printf("设置电机 0 模式为 %s\n", modeStr);

                // 实时打印所有电机的目标位置和模式
                pthread_mutex_lock(&cmd_lock);
                for(int i = 0; i < MOTOR_NUM; i++) {
                    printf("电机 %d: 位置 = %d, 模式 = %d\n", i, currentCommand[i].target_pos, currentCommand[i].mode);
                }
                pthread_mutex_unlock(&cmd_lock);

                if (strcmp(modeStr, "home") == 0) {
                    printf("等待10秒以完成回零...\n");
                    sleep(10); // 等待10秒
                }
            } else {
                fprintf(stderr, "设置模式失败，命令格式错误: %s\n", input);
            }
        }
        // 处理其他可能的命令
        else {
            fprintf(stderr, "未知的命令: %s\n", input);
        }
    }

    fclose(file);
    printf("命令文件读取完成。\n");
    return NULL;
}
