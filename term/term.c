#include "term.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "motor.h"

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

void* readCommands(void* arg)
{
    char input[100];
    for(int i = 0; i < MOTOR_NUM; i++) {
        currentCommand[i].target_pos = 0;
        currentCommand[i].mode = MODEL_NONE;
    }
    while (1) {
        if (fgets(input, sizeof(input), stdin) == NULL) {
            fprintf(stderr, "读取输入失败\n");
            continue;
        }
        // 设置电机位置, pos id distance
        if (strncmp(input, "pos", 3) == 0) {
            int motorID, distance;
            if (sscanf(input, "pos %d %d", &motorID, &distance) == 2) {
                setPos(motorID, distance);
            } else {
                fprintf(stderr, "设置位置失败\n");
            }
        }
        // 设置电机模式, mode  home/csp
        else if (strncmp(input, "mode", 4) == 0) {
            char mode[10];
            if (sscanf(input, "mode %s", mode) == 1) {
                if (strcmp(mode, "home") == 0) {
                    setMode(0, MODEL_HOME);
                } else if (strcmp(mode, "csp") == 0) {
                    setMode(0, MODEL_CSP);
                }else if (strcmp(mode, "pp") == 0) {
                    setMode(0, MODEL_PP);
                }else {
                    fprintf(stderr, "设置模式失败\n");
                    printf("mode %s\n", mode);
                }
            } else {
                fprintf(stderr, "设置模式失败\n");
                printf("mode %s\n", mode);
            }
        }

    }
    
    return NULL;
}