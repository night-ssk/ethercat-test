#ifndef FRONT_H
#define FRONT_H

/**
 * @brief 根据理想 DH 模型计算正运动学
 *
 * 输入参数单位：
 *   theta1, theta5, theta6：角度，单位为 °；
 *   d2, d3, d4：直线位移，单位为 m。
 *
 * 输出参数：
 *   alpha70, beta70, gamma70：末端姿态（单位 °），其中 gamma70 = -theta6；
 *   x70, y70, z70：末端位置（单位 m）。
 */
void ideal_DH_FK(double theta1, double d2, double d3, double d4,
                 double theta5, double theta6,
                 double* alpha70, double* beta70, double* gamma70,
                 double* x70, double* y70, double* z70);

/**
 * @brief 根据各关节电机脉冲数计算正运动学
 *
 * 根据预设参数，将电机脉冲转换为物理量后，
 * 利用理想 DH 模型计算末端姿态和位置。
 *
 * 输入参数：
 *   pulse_theta1, pulse_d2, pulse_d3, pulse_d4, pulse_theta5, pulse_theta6：各关节脉冲数。
 *
 * 输出参数：
 *   alpha70, beta70, gamma70：末端姿态（单位 °）；
 *   x70, y70, z70：末端位置（单位 m）。
 */
void forward_kinematics_from_pulses(int pulse_theta1, int pulse_d2, int pulse_d3, int pulse_d4,
                                    int pulse_theta5, int pulse_theta6,
                                    double* alpha70, double* beta70, double* gamma70,
                                    double* x70, double* y70, double* z70);



#endif // FK_SOLVER_H
