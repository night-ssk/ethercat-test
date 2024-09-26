// motor.h

#ifndef MOTOR_H
#define MOTOR_H

#include <ecrt.h> 

#define MOTOR_NUM 6 // 根据需要设置电机数量

extern ec_sync_info_t slave_motor_syncs[];

// 电机参数结构体
struct _motorParm {
    unsigned int ctrl_word;         //6040-00h
    unsigned int operation_mode;    //6060-00h
    unsigned int target_pos;        //607A-00h
    // unsigned int step_div;        //6092-01h
    // 回零相关
    unsigned int home_way;          //6098-00h
    unsigned int home_spd_high;     //6099-01h
    unsigned int home_spd_low;      //6099-02h
    unsigned int home_acc;          //609A-00h
    // PP模式相关
    unsigned int profile_velocity;  //6081-00h
    unsigned int profile_acc;       //6083-00h
    unsigned int profile_dec;       //6084-00h

    unsigned int status_word;       //6041-00h
    unsigned int operation_mode_display; //6061-00h
    unsigned int current_pos;       //6064-00h
} motor_parm[MOTOR_NUM];

enum motor_info {
    MODEL_PP = 1, MODEL_PV = 3, MODEL_HOME = 6, MODEL_CSP = 8, MODEL_NONE = 0,
};

void motor_main(struct _Domain* domain);

#endif // MOTOR_H
