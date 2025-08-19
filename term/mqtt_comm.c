#include "mqtt_comm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <mosquitto.h>
#include <cJSON.h>

/* 定义全局变量：接收数据与发送数据 */
RobotControlData g_robotControlData = { {0}, {0}, 0, 0, 0 };
RobotStateData   g_robotStateData   = { {0}, {0}, 0, -1, 0 };//初始化不能操作

/* MQTT 相关参数 */
static struct mosquitto *mosq = NULL;
static const char *sub_topic = "SeriesControl";  // 接收话题
static const char *pub_topic = "SeriesState";      // 发送话题

/* 发送线程句柄 */
static pthread_t send_thread;

/* 内部函数声明 */
static void on_connect(struct mosquitto *mosq, void *obj, int rc);
static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);
static void *send_thread_func(void *arg);

//打印cjson
void print_robot_control_data(const RobotControlData *data)
{
    /* 创建 JSON 根对象 */
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        fprintf(stderr, "创建 JSON 对象失败\n");
        return;
    }
    
    /* 添加绝对位置 AbsolutePose 数组 */
    cJSON *absPose = cJSON_CreateDoubleArray(data->AbsolutePose, 6);
    if (!absPose) {
        fprintf(stderr, "创建 AbsolutePose 数组失败\n");
        cJSON_Delete(root);
        return;
    }
    cJSON_AddItemToObject(root, "AbsolutePose", absPose);
    
    /* 添加增量位置信息 IncPosition 对象 */
    cJSON *incPosition = cJSON_CreateObject();
    if (!incPosition) {
        fprintf(stderr, "创建 IncPosition 对象失败\n");
        cJSON_Delete(root);
        return;
    }
    for (int i = 0; i < 6; i++) {
        char key[16];
        snprintf(key, sizeof(key), "Delta%d", i + 1);
        cJSON_AddNumberToObject(incPosition, key, data->Delta[i]);
    }
    cJSON_AddNumberToObject(incPosition, "Step", data->step);
    cJSON_AddItemToObject(root, "IncPosition", incPosition);
    
    /* 添加任务信息 TaskInfo 对象 */
    cJSON *taskInfo = cJSON_CreateObject();
    if (!taskInfo) {
        fprintf(stderr, "创建 TaskInfo 对象失败\n");
        cJSON_Delete(root);
        return;
    }
    cJSON_AddNumberToObject(taskInfo, "TaskNum", data->taskNum);
    cJSON_AddNumberToObject(taskInfo, "TaskType", data->taskType);
    cJSON_AddItemToObject(root, "TaskInfo", taskInfo);
    
    /* 打印生成的 JSON 字符串 */
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        printf("MQTT JSON 消息格式:\n%s\n", json_str);
        free(json_str);
    }
    
    cJSON_Delete(root);
}

/**
 * @brief 初始化 MQTT 客户端、建立连接并启动后台线程
 */
int mqtt_comm_init(const char *broker_address, int broker_port)
{
    int ret;
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "创建 mosquitto 客户端失败\n");
        return -1;
    }
    /* 设置回调函数 */
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);
    
    ret = mosquitto_connect(mosq, broker_address, broker_port, 60);
    if(ret) {
        fprintf(stderr, "连接失败, 错误码：%d\n", ret);
        return ret;
    }
    /* 启动网络处理线程（内部会创建一个线程） */
    ret = mosquitto_loop_start(mosq);
    if(ret) {
        fprintf(stderr, "启动 mosquitto 网络循环失败：%d\n", ret);
        return ret;
    }
    /* 创建发送线程，每秒发布一次状态数据 */ 
    ret = pthread_create(&send_thread, NULL, send_thread_func, NULL);
    if(ret) {
        fprintf(stderr, "创建发送线程失败\n");
        return ret;
    }
    return 0;
}

/**
 * @brief 清理 MQTT 客户端资源
 */
void mqtt_comm_cleanup(void)
{
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_loop_stop(mosq, true);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
}

/**
 * @brief 阻塞等待发送线程结束（示例中发送线程无限循环，可根据需要修改退出机制）
 */
void mqtt_comm_start(void)
{
    pthread_detach(send_thread);
}

/**
 * @brief MQTT 连接成功后的回调函数，订阅 SeriesControl 话题
 */
static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    if (rc == 0) {
        printf("成功连接到 MQTT 服务器。\n");
        mosquitto_subscribe(mosq, NULL, sub_topic, 0);
    } else {
        printf("连接失败，错误码：%d\n", rc);
    }
}

/**
 * @brief 收到消息后的回调函数
 *
 * 当接收到 SeriesControl 话题的消息后，将 JSON 数据解析并更新全局的 g_robotControlData。
 */
static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
    (void)obj;
    if(msg->payloadlen && strcmp(msg->topic, sub_topic) == 0) {
        cJSON *root = cJSON_Parse(msg->payload);
        if(root == NULL) {
            fprintf(stderr, "JSON解析失败\n");
            return;
        }
        /* 解析绝对位置 AbsolutePose 数组，要求至少6个元素 */
        cJSON *absPose = cJSON_GetObjectItem(root, "AbsolutePose");
        if(cJSON_IsArray(absPose) && cJSON_GetArraySize(absPose) >= 6) {
            for (int i = 0; i < 6; i++) {
                g_robotControlData.AbsolutePose[i] = cJSON_GetArrayItem(absPose, i)->valuedouble;
            }
        }
        /* 解析相对位置 IncPosition 对象 */
        cJSON *incPos = cJSON_GetObjectItem(root, "IncPosition");
        if(incPos && cJSON_IsObject(incPos)) {
            for (int i = 0; i < 6; i++) {
                char key[16];
                snprintf(key, sizeof(key), "Delta%d", i+1);
                cJSON *delta = cJSON_GetObjectItem(incPos, key);
                if(cJSON_IsNumber(delta)) {
                    g_robotControlData.Delta[i] = delta->valuedouble;
                }
            }
            cJSON *step = cJSON_GetObjectItem(incPos, "Step");
            if(cJSON_IsNumber(step)) {
                g_robotControlData.step = step->valueint;
            }
        }
        /* 解析任务信息 TaskInfo */
        cJSON *taskInfo = cJSON_GetObjectItem(root, "TaskInfo");
        if(taskInfo && cJSON_IsObject(taskInfo)) {
            cJSON *taskNum = cJSON_GetObjectItem(taskInfo, "TaskNum");
            cJSON *taskType = cJSON_GetObjectItem(taskInfo, "TaskType");
            if(cJSON_IsNumber(taskNum))  g_robotControlData.taskNum  = taskNum->valueint;
            if(cJSON_IsNumber(taskType)) g_robotControlData.taskType = taskType->valueint;
        }
        cJSON_Delete(root);
    }
}

/**
 * @brief 发送线程函数
 *
 * 每隔1秒构造一条 JSON 数据并通过 MQTT 发布到 SeriesState 话题。
 * JSON 格式示例：
 * {
 *    "PositionAxis": [Delta1, Delta2, ..., Delta6],
 *    "PositionXYZ": [x, y, z, Rx, Ry, Rz],
 *    "TaskInfo": { "TaskNum": ..., "TaskState": ..., "TaskType": ... }
 * }
 */
static void *send_thread_func(void *arg)
{
    (void)arg;
    while(1) {
        cJSON *root = cJSON_CreateObject();
        if (!root) {
            fprintf(stderr, "创建 JSON 对象失败\n");
            sleep(1);
            continue;
        }
        /* 创建 PositionAxis 数组 */
        cJSON *positionAxis = cJSON_CreateDoubleArray(g_robotStateData.PositionAxis, 6);
        cJSON_AddItemToObject(root, "PositionAxis", positionAxis);
        
        /* 创建 PositionXYZ 数组 */
        cJSON *positionXYZ = cJSON_CreateDoubleArray(g_robotStateData.PositionXYZ, 6);
        cJSON_AddItemToObject(root, "PositionXYZ", positionXYZ);
        
        /* 创建 TaskInfo 对象 */
        cJSON *taskInfo = cJSON_CreateObject();
        cJSON_AddNumberToObject(taskInfo, "TaskNum",  g_robotStateData.taskNum);
        cJSON_AddNumberToObject(taskInfo, "TaskState",g_robotStateData.taskState);
        cJSON_AddNumberToObject(taskInfo, "TaskType", g_robotStateData.taskType);
        cJSON_AddItemToObject(root, "TaskInfo", taskInfo);
        
        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            int ret = mosquitto_publish(mosq, NULL, pub_topic, (int)strlen(json_str), json_str, 0, false);
            if(ret != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "发布消息失败: %s\n", mosquitto_strerror(ret));
            }
            free(json_str);
        }
        cJSON_Delete(root);
        sleep(1);
    }
    return NULL;
}
