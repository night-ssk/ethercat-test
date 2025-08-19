#include "front.h"
#include <math.h>

/* 结构参数（单位：m） */
static const double L1 = 148e-3;
static const double L2 = 42e-3;
static const double L3 = 132e-3;
static const double L4 = 122e-3;
static const double L5 = 108e-3;
static const double L6 = 92.5e-3;  // 注意：L45 = L4 + L5，但在正运动学中直接使用 L4 和 L5

/* 转换比例 */
static const double POSITION_PULSE_PER_M         = 1.0 / 50e-9;       // 1脉冲对应50nm
static const double ROTATIONAL_PULSE_PER_DEG_DD    = 1.0 / 0.001;       // 1脉冲对应0.001°
static const double ROTATIONAL_PULSE_PER_DEG_ZOLIX = 1.0 / 0.00055277122; // 1脉冲对应0.00055277122°

/* 各轴零点脉冲数 */
static const int ZERO_PULSE_THETA6 = 0;   // 旋转关节6
static const int ZERO_PULSE_THETA5 = 71158;  // 旋转关节5
static const int ZERO_PULSE_D4     = 795823;  // 位移关节4
static const int ZERO_PULSE_D3     = 703125;  // 位移关节3
static const int ZERO_PULSE_D2     = 705678;  // 位移关节2
static const int ZERO_PULSE_THETA1 = 181148;   // 旋转关节1
//左极限 261784
//右极限 22853
//零点 181148
//上电右极限 138720
/**
 * @brief 根据理想 DH 模型计算正运动学
 *
 * 将输入的关节物理量（角度单位为°，位移单位为 m）转换为末端位姿。
 * 注意：内部计算时将角度转换为弧度。
 *
 * @param theta1 输入：关节1角度 (°)
 * @param d2     输入：直线关节 d2 (m)
 * @param d3     输入：直线关节 d3 (m)
 * @param d4     输入：直线关节 d4 (m)
 * @param theta5 输入：关节5角度 (°)
 * @param theta6 输入：关节6角度 (°)
 * @param alpha70 输出：末端姿态 alpha70 (°)，等于 theta1
 * @param beta70  输出：末端姿态 beta70 (°)，等于 theta5
 * @param gamma70 输出：末端姿态 gamma70 (°)，等于 -theta6
 * @param x70     输出：末端位置 x70 (m)
 * @param y70     输出：末端位置 y70 (m)
 * @param z70     输出：末端位置 z70 (m)
 */
void ideal_DH_FK(double theta1, double d2, double d3, double d4,
                 double theta5, double theta6,
                 double* alpha70, double* beta70, double* gamma70,
                 double* x70, double* y70, double* z70)
{
    /* 将输入角度转换为弧度 */
    double theta1_rad = theta1 * (M_PI / 180.0);
    double theta5_rad = theta5 * (M_PI / 180.0);
    double theta6_rad = theta6 * (M_PI / 180.0);

    /* 定义齐次变换矩阵 T70 (4x4) */
    double T70[4][4];

    /* 第一行 */
    T70[0][0] = cos(theta1_rad)*cos(theta6_rad) + sin(theta1_rad)*sin(theta5_rad)*sin(theta6_rad);
    T70[0][1] = - cos(theta5_rad)* sin(theta1_rad);
    T70[0][2] = cos(theta6_rad)* sin(theta1_rad)* sin(theta5_rad) - cos(theta1_rad)* sin(theta6_rad);
    T70[0][3] = - sin(theta1_rad)*(d2 + L6*cos(theta6_rad)* sin(theta5_rad))
                + cos(theta1_rad)*(d3 + L4 + L5 + L6*sin(theta6_rad));

    /* 第二行 */
    T70[1][0] = cos(theta6_rad)* sin(theta1_rad) - cos(theta1_rad)* sin(theta5_rad)* sin(theta6_rad);
    T70[1][1] = cos(theta1_rad)* cos(theta5_rad);
    T70[1][2] = - cos(theta1_rad)* cos(theta6_rad)* sin(theta5_rad) - sin(theta1_rad)* sin(theta6_rad);
    T70[1][3] = cos(theta1_rad)*(d2 + L6*cos(theta6_rad)* sin(theta5_rad))
                + sin(theta1_rad)*(d3 + L4 + L5 + L6*sin(theta6_rad));

    /* 第三行 */
    T70[2][0] = cos(theta5_rad)* sin(theta6_rad);
    T70[2][1] = sin(theta5_rad);
    T70[2][2] = cos(theta5_rad)* cos(theta6_rad);
    T70[2][3] = d4 + L1 + L2 + L3 - L6*cos(theta5_rad)* cos(theta6_rad);

    /* 第四行 */
    T70[3][0] = 0.0;
    T70[3][1] = 0.0;
    T70[3][2] = 0.0;
    T70[3][3] = 1.0;

    /* 末端姿态
     * 根据 Matlab 代码：
     *   alpha70 = rad2deg(theta1)  ==> 与输入 theta1 相同（单位 °）
     *   beta70  = rad2deg(theta5)  ==> 与输入 theta5 相同（单位 °）
     *   gamma70 = rad2deg(-theta6) ==> 即 -theta6 (单位 °)
     */
    *alpha70 = theta1;
    *beta70  = theta5;
    *gamma70 = -theta6;

    /* 末端位置 */
    *x70 = T70[0][3];
    *y70 = T70[1][3];
    *z70 = T70[2][3];
}

/**
 * @brief 根据电机脉冲数计算正运动学
 *
 * 先将脉冲转换为实际运动量，再调用 ideal_DH_FK() 计算末端位姿。
 *
 * @param pulse_theta1 旋转关节1脉冲数
 * @param pulse_d2     位移关节 d2 脉冲数
 * @param pulse_d3     位移关节 d3 脉冲数
 * @param pulse_d4     位移关节 d4 脉冲数
 * @param pulse_theta5 旋转关节5脉冲数
 * @param pulse_theta6 旋转关节6脉冲数
 * @param alpha70      输出：末端姿态 alpha70 (°)
 * @param beta70       输出：末端姿态 beta70 (°)
 * @param gamma70      输出：末端姿态 gamma70 (°)
 * @param x70          输出：末端位置 x70 (m)
 * @param y70          输出：末端位置 y70 (m)
 * @param z70          输出：末端位置 z70 (m)
 */
void forward_kinematics_from_pulses(int pulse_theta1, int pulse_d2, int pulse_d3, int pulse_d4,
                                    int pulse_theta5, int pulse_theta6,
                                    double* x70, double* y70, double* z70,
                                    double* alpha70, double* beta70, double* gamma70)
{
    /* 将电机脉冲转换为实际运动量 */
    double theta1 = (pulse_theta1 - ZERO_PULSE_THETA1) / ROTATIONAL_PULSE_PER_DEG_DD; // 单位 °
    double d2     = (pulse_d2 - ZERO_PULSE_D2) / POSITION_PULSE_PER_M;              // 单位 m
    double d3     = (pulse_d3 - ZERO_PULSE_D3) / POSITION_PULSE_PER_M;
    double d4     = (pulse_d4 - ZERO_PULSE_D4) / POSITION_PULSE_PER_M;
    double theta5 = (pulse_theta5 - ZERO_PULSE_THETA5) / ROTATIONAL_PULSE_PER_DEG_DD; // 单位 °
    double theta6 = (pulse_theta6 - ZERO_PULSE_THETA6) / ROTATIONAL_PULSE_PER_DEG_ZOLIX; // 单位 °

    /* 调用理想 DH 正运动学计算 */
    ideal_DH_FK(theta1, d2, d3, d4, theta5, theta6,
                alpha70, beta70, gamma70, x70, y70, z70);
    *x70 = 1000 * (*x70);
    *y70 = 1000 * (*y70);
    *z70 = 1000 * (*z70);
}
