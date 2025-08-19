#include "back.h"
#include <math.h>

/* 结构参数（单位：m） */
static const double L1 = 148e-3;
static const double L2 = 42e-3;
static const double L3 = 132e-3;
static const double L4 = 122e-3;
static const double L5 = 108e-3;
static const double L6 = 92.5e-3;

/* 转换比例 */
static const double POSITION_PULSE_PER_M         = 1.0 / 50e-9;       // 位移：1脉冲对应50nm
static const double ROTATIONAL_PULSE_PER_DEG_DD    = 1.0 / 0.001;       // 力矩电机：1脉冲对应0.001°
static const double ROTATIONAL_PULSE_PER_DEG_ZOLIX = 1.0 / 0.00055277122; // 卓立汉光电机：1脉冲对应0.00055277122°

/* 各轴零点脉冲数 */
static const int ZERO_PULSE_THETA6 = 0;   // 旋转关节6
static const int ZERO_PULSE_THETA5 = 71158;  // 旋转关节5
static const int ZERO_PULSE_D4     = 795823;  // 位移关节4
static const int ZERO_PULSE_D3     = 703125;  // 位移关节3
static const int ZERO_PULSE_D2     = 705678;  // 位移关节2
static const int ZERO_PULSE_THETA1 = 181148;   // 旋转关节1


/**
 * 内部函数：根据理想DH模型计算逆运动学
 * 输入参数：
 *   rx, ry, rz 为末端姿态（单位：°，其中 rz 在计算中取反）
 *   x, y, z 为末端位置（单位：m）
 * 输出：
 *   d2, d3, d4 为直线运动量（单位：m）
 *   theta1, theta5 为旋转关节角（单位：°，直接使用 rx 和 ry）
 *   theta6_rad 为内部计算用的旋转角（单位：弧度，计算时使用 -rz 的弧度值，用于 zolix 电机转换）
 */
static void ideal_DH_IK(double rx, double ry, double rz,
                        double x, double y, double z,
                        double* d2, double* d3, double* d4,
                        double* theta6_rad)
{
    /* 将旋转角转换为弧度用于三角函数计算 */
    double theta1_rad = rx * (M_PI / 180.0);
    double theta5_rad = ry * (M_PI / 180.0);
    *theta6_rad = -rz * (M_PI / 180.0);  // 注意：内部使用 -rz

    /* 计算各直线关节的运动量 */
    *d2 = y * cos(theta1_rad) - x * sin(theta1_rad) - L6 * cos(*theta6_rad) * sin(theta5_rad);
    *d3 = -L4 - L5 + x * cos(theta1_rad) + y * sin(theta1_rad) - L6 * sin(*theta6_rad); // -230
    *d4 = -L1 - L2 - L3 + z + L6 * cos(theta5_rad) * cos(*theta6_rad); //229.5
}

void inverse_kinematics(double x, double y, double z,
                       double rx, double ry, double rz,
                       int* pulse_theta1, int* pulse_d2, int* pulse_d3,
                       int* pulse_d4, int* pulse_theta5, int* pulse_theta6)
{
    x = 0.001 * x; // mm -> m
    y = 0.001 * y; // mm -> m
    z = 0.001 * z; // mm -> m
    double d2, d3, d4;
    double theta6_rad;
    
    /* 计算直线运动量和theta6（弧度） */
    ideal_DH_IK(rx, ry, rz, x, y, z, &d2, &d3, &d4, &theta6_rad);

    /* 将各关节的物理运动量转换为电机脉冲数
     * 注意：rx 和 ry 直接作为角度输入，rz 按照 -rz 计算
     */
    double pulse_theta1_d = ZERO_PULSE_THETA1 + rx * ROTATIONAL_PULSE_PER_DEG_DD;
    double pulse_d2_d     = ZERO_PULSE_D2     + d2 * POSITION_PULSE_PER_M;
    double pulse_d3_d     = ZERO_PULSE_D3     + d3 * POSITION_PULSE_PER_M;
    double pulse_d4_d     = ZERO_PULSE_D4     + d4 * POSITION_PULSE_PER_M;
    double pulse_theta5_d = ZERO_PULSE_THETA5 + ry * ROTATIONAL_PULSE_PER_DEG_DD;
    double pulse_theta6_d = ZERO_PULSE_THETA6 + (-rz) * ROTATIONAL_PULSE_PER_DEG_ZOLIX;

    /* 取整 */
    int pt1 = (int)round(pulse_theta1_d);
    int pd2 = (int)round(pulse_d2_d);
    int pd3 = (int)round(pulse_d3_d);
    int pd4 = (int)round(pulse_d4_d);
    int pt5 = (int)round(pulse_theta5_d);
    int pt6 = (int)round(pulse_theta6_d);

    /* 检查各轴是否超出行程限位
     * 顺序：theta6, theta5, d4, d3, d2, theta1
     */
    // if (pt6 < LIMIT_THETA6_MIN || pt6 > LIMIT_THETA6_MAX)
    //     return IK_ERR_OUT_OF_RANGE;
    // if (pt5 < LIMIT_THETA5_MIN || pt5 > LIMIT_THETA5_MAX)
    //     return IK_ERR_OUT_OF_RANGE;
    // if (pd4 < LIMIT_D4_MIN     || pd4 > LIMIT_D4_MAX)
    //     return IK_ERR_OUT_OF_RANGE;
    // if (pd3 < LIMIT_D3_MIN     || pd3 > LIMIT_D3_MAX)
    //     return IK_ERR_OUT_OF_RANGE;
    // if (pd2 < LIMIT_D2_MIN     || pd2 > LIMIT_D2_MAX)
    //     return IK_ERR_OUT_OF_RANGE;
    // if (pt1 < LIMIT_THETA1_MIN || pt1 > LIMIT_THETA1_MAX)
    //     return IK_ERR_OUT_OF_RANGE;

    /* 返回计算结果 */
    *pulse_theta1 = pt1;
    *pulse_d2     = pd2;
    *pulse_d3     = pd3;
    *pulse_d4     = pd4;
    *pulse_theta5 = pt5;
    *pulse_theta6 = pt6;

    return ;
}
