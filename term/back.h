#ifndef BACK_H
#define BACK_H


/* 逆运动学函数返回值 */
#define IK_SUCCESS           0
#define IK_ERR_OUT_OF_RANGE  1

/**
 * @brief 逆运动学计算，将输入的末端位姿转换为六个关节的脉冲数
 *
 * @param x  末端位置 x (m)
 * @param y  末端位置 y (m)
 * @param z  末端位置 z (m)
 * @param rx 末端姿态 rx (°) 对应 theta1 (旋转，dd电机)
 * @param ry 末端姿态 ry (°) 对应 theta5 (旋转，dd电机)
 * @param rz 末端姿态 rz (°) 对应 theta6 (旋转，zolix电机，内部取 -rz)
 * @param pulse_theta1 输出：旋转关节1的脉冲数
 * @param pulse_d2     输出：直线关节 d2 的脉冲数
 * @param pulse_d3     输出：直线关节 d3 的脉冲数
 * @param pulse_d4     输出：直线关节 d4 的脉冲数
 * @param pulse_theta5 输出：旋转关节5的脉冲数
 * @param pulse_theta6 输出：旋转关节6的脉冲数
 *
 * @return IK_SUCCESS 正常返回
 *         IK_ERR_OUT_OF_RANGE 任一关节脉冲超出预设限位时返回错误
 */
void inverse_kinematics(double x, double y, double z,
                       double rx, double ry, double rz,
                       int* pulse_theta1, int* pulse_d2, int* pulse_d3,
                       int* pulse_d4, int* pulse_theta5, int* pulse_theta6);



#endif // BACK_H
