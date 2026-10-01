#include "usr_fusion.h"

#include <stdint.h>
#include <math.h>

#include "usr_algorithm.h"
 
#define Acc_Gain 0.0001220f			//加速度转换单位(初始化加速度计量程+-4g，由于mpu6050的数据寄存器是16位的，LSBa = 2*4 / 65535.0)
#define Gyro_Gain 0.0609756f		//角速度转换为角度(LSBg = 2000*2 / 65535)
#define Gyro_Gr 0.0010641f			//角速度转换成弧度(3.1415 / 180 * LSBg)
#define G 9.80665f					// m/s^2
#define Kp 1.50f
#define Ki 0.005f
#define halfT 0.001f				//计算周期的一半，单位s

#define COMP_FILTER(angle, gyro_rate, acc_angle, alpha, dt) \
    do { \
        (angle) = (alpha) * ((angle) + (gyro_rate) * (dt)) \
                + (1.0f - (alpha)) * (acc_angle); \
    } while (0)

static inline float invSqrt(float x){
    union {
        float    f;
        uint32_t i;
    } u;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - 0.5f * x * y * y);
    return y;
}

void usr_fusion_imu_update(fusion_t *fusion_data){
    // 单位转换
    fusion_data->acc_x = (float)fusion_data->raw.ax * Acc_Gain * G;
    fusion_data->acc_y = (float)fusion_data->raw.ay * Acc_Gain * G;
    fusion_data->acc_z = (float)fusion_data->raw.az * Acc_Gain * G;
    fusion_data->gyro_x = (float)fusion_data->raw.gx * Gyro_Gr;
    fusion_data->gyro_y = (float)fusion_data->raw.gy * Gyro_Gr;
    fusion_data->gyro_z = (float)fusion_data->raw.gz * Gyro_Gr;

    float vx, vy, vz;
    float ex, ey, ez;
    float norm;

    // 积分项应持续累加（注意：多实例不适用，建议移到 fusion_t 中）
    static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

    // 加速度平方和检查，避免 invSqrt(0)
    float acc_sq = SQUARE(fusion_data->acc_x) + SQUARE(fusion_data->acc_y) + SQUARE(fusion_data->acc_z);
    if (acc_sq < 1e-6f) {
        return; // 加速度无效，跳过本次融合
    }

    // 归一化加速度，得到测量重力方向（机体坐标系）
    norm = invSqrt(acc_sq);
    fusion_data->acc_x *= norm;
    fusion_data->acc_y *= norm;
    fusion_data->acc_z *= norm;

    // 用当前四元数计算重力方向估计（机体坐标系）
    float q0 = fusion_data->q[0];
    float q1 = fusion_data->q[1];
    float q2 = fusion_data->q[2];
    float q3 = fusion_data->q[3];

    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 叉积误差
    ex = (fusion_data->acc_y * vz - fusion_data->acc_z * vy);
    ey = (fusion_data->acc_z * vx - fusion_data->acc_x * vz);
    ez = (fusion_data->acc_x * vy - fusion_data->acc_y * vx);

    // 积分误差
    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    // 角速度补偿
    fusion_data->gyro_x += Kp * ex + exInt;
    fusion_data->gyro_y += Kp * ey + eyInt;
    fusion_data->gyro_z += Kp * ez + ezInt;

    // 更新四元数（使用临时变量 q0~q3，避免顺序更新）
    fusion_data->q[0] = q0 + (-q1 * fusion_data->gyro_x - q2 * fusion_data->gyro_y - q3 * fusion_data->gyro_z) * halfT;
    fusion_data->q[1] = q1 + ( q0 * fusion_data->gyro_x + q2 * fusion_data->gyro_z - q3 * fusion_data->gyro_y) * halfT;
    fusion_data->q[2] = q2 + ( q0 * fusion_data->gyro_y - q1 * fusion_data->gyro_z + q3 * fusion_data->gyro_x) * halfT;
    fusion_data->q[3] = q3 + ( q0 * fusion_data->gyro_z + q1 * fusion_data->gyro_y - q2 * fusion_data->gyro_x) * halfT;

    // 归一化四元数
    float q_sq = SQUARE(fusion_data->q[0]) + SQUARE(fusion_data->q[1]) + SQUARE(fusion_data->q[2]) + SQUARE(fusion_data->q[3]);
    if (q_sq < 1e-6f) {
        return; // 四元数无效，跳过
    }
    norm = invSqrt(q_sq);
    fusion_data->q[0] *= norm;
    fusion_data->q[1] *= norm;
    fusion_data->q[2] *= norm;
    fusion_data->q[3] *= norm;

    // 用归一化后的新四元数重新计算乘积，用于欧拉角
    q0 = fusion_data->q[0];
    q1 = fusion_data->q[1];
    q2 = fusion_data->q[2];
    q3 = fusion_data->q[3];

    float q0q0 = SQUARE(q0);
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = SQUARE(q1);
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = SQUARE(q2);
    float q2q3 = q2 * q3;
    float q3q3 = SQUARE(q3);

    // 四元数反解欧拉角（单位：度）
    fusion_data->roll  = fast_atan2f(2.0f * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3) * 57.3f;
    fusion_data->pitch = -asin(2.0f * (q1q3 - q0q2)) * 57.3f;
    fusion_data->yaw   = fast_atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.3f;
}
