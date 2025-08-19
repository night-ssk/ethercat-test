#ifndef MQTT_COMM_H
#define MQTT_COMM_H



/**
 * @brief 接收数据结构
 *
 * 用于保存从上位机接收到的控制数据，字段对应 JSON 中的 AbsolutePose、IncPosition 和 TaskInfo（不含 TaskState）。
 */
typedef struct {
    double AbsolutePose[6];  // x, y, z, Rx, Ry, Rz
    double Delta[6];            // Delta1 ~ Delta6
    int step;                // IncPosition 中的 Step
    int taskNum;
    int taskType;
} RobotControlData;

/**
 * @brief 发送数据结构
 *
 * 用于构造需要发送给上位机的状态数据，字段对应 JSON 中的 PositionXYZ、PositionAxis 和 TaskInfo（包含 TaskState）。
 */
typedef struct {
    double PositionXYZ[6];   // x, y, z, Rx, Ry, Rz
    double PositionAxis[6];     // 对应于接收数据中的 Delta1 ~ Delta6（可以根据需要转换）
    int taskNum;
    int taskState;           // 发送时用到的任务状态
    int taskType;
} RobotStateData;

/* 全局变量：分别存储接收和发送数据 */
extern RobotControlData g_robotControlData;
extern RobotStateData   g_robotStateData;

/**
 * @brief 初始化 MQTT 通讯，包括创建 mosquitto 客户端、连接、启动网络循环线程和发送线程
 * @param broker_address MQTT 服务器地址，例如 "192.168.1.100"
 * @param broker_port MQTT 服务器端口，例如 1883
 * @return 0 表示成功，否则返回错误码
 */
int mqtt_comm_init(const char *broker_address, int broker_port);

/**
 * @brief 清理 MQTT 相关资源
 */
void mqtt_comm_cleanup(void);

/**
 * @brief 启动通讯任务（本例中初始化后网络循环和发送线程已启动，该函数用于阻塞等待发送线程退出）
 */
void mqtt_comm_start(void);

void print_robot_control_data(const RobotControlData *data);

#endif // MQTT_COMM_H
