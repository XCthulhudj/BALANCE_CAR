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

static void usr_q_mul(float q1[4], float q2[4], float res[4]);
static void usr_q_2_euler(float q[4], float euler[3]);

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

void usr_coordinate_rotation_internal(float q_in[4],
                                      float q_out[4],
                                      float euler_out[3],
                                      float rx, float ry, float rz,
                                      float rTheta_deg)
{
    /* --- 输入四元数归一化 --- */
    float q_old[4];
    float n = sqrtf(q_in[0]*q_in[0] + q_in[1]*q_in[1]
                  + q_in[2]*q_in[2] + q_in[3]*q_in[3]);
    if (n < 1e-12f) {
        q_out[0] = 1.0f; q_out[1] = q_out[2] = q_out[3] = 0.0f;
        if (euler_out) { euler_out[0] = euler_out[1] = euler_out[2] = 0.0f; }
        return;
    }
    q_old[0] = q_in[0] / n;
    q_old[1] = q_in[1] / n;
    q_old[2] = q_in[2] / n;
    q_old[3] = q_in[3] / n;

    /* --- 旋转轴归一化 --- */
    float axis_sq = rx*rx + ry*ry + rz*rz;
    if (axis_sq < 1e-12f) {
        q_out[0] = q_old[0]; q_out[1] = q_old[1];
        q_out[2] = q_old[2]; q_out[3] = q_old[3];
        if (euler_out) usr_q_2_euler(q_out, euler_out);
        return;
    }
    float inv_axis = 1.0f / sqrtf(axis_sq);
    rx *= inv_axis; ry *= inv_axis; rz *= inv_axis;

    /* --- 构造旋转四元数 q_rot --- */
    float half_rad = rTheta_deg * 0.5f * (float)M_PI / 180.0f;
    float s = sinf(half_rad);
    float c = cosf(half_rad);
    float q_rot[4] = { c, rx * s, ry * s, rz * s };

    /* --- 关键：右乘，做坐标系变换 --- */
    /* q_out = q_in * q_rot */
    usr_q_mul(q_old, q_rot, q_out);

    /* --- 归一化 --- */
    n = sqrtf(q_out[0]*q_out[0] + q_out[1]*q_out[1]
            + q_out[2]*q_out[2] + q_out[3]*q_out[3]);
    if (n > 1e-12f) {
        q_out[0] /= n; q_out[1] /= n; q_out[2] /= n; q_out[3] /= n;
    }

    if (euler_out) usr_q_2_euler(q_out, euler_out);
}

void usr_rq_vector(float vec_in[3],
                   float vec_out[3],
                   float rx, float ry, float rz,
                   float rTheta_deg)
{
    float vx = vec_in[0];
    float vy = vec_in[1];
    float vz = vec_in[2];

    float axis_sq = rx*rx + ry*ry + rz*rz;
    if (axis_sq < 1e-12f) {
        vec_out[0] = vx;
        vec_out[1] = vy;
        vec_out[2] = vz;
        return;
    }

    float inv_axis = 1.0f / sqrtf(axis_sq);
    rx *= inv_axis;
    ry *= inv_axis;
    rz *= inv_axis;

    float half_rad = rTheta_deg * 0.5f * (float)M_PI / 180.0f;

    /* 同样先用标准 sinf/cosf，确认不是 fast 函数问题 */
    float s = sinf(half_rad);
    float c = cosf(half_rad);

    float qw = c;
    float qx = rx * s;
    float qy = ry * s;
    float qz = rz * s;

    /* 归一化旋转四元数 */
    float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-12f) {
        vec_out[0] = vx;
        vec_out[1] = vy;
        vec_out[2] = vz;
        return;
    }
    qw /= n;
    qx /= n;
    qy /= n;
    qz /= n;

    /*
     * v_out = q * v_in * q^-1
     * t = 2 * (q_vec × v)
     * v_out = v + qw * t + q_vec × t
     */
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);

    vec_out[0] = vx + qw * tx + (qy * tz - qz * ty);
    vec_out[1] = vy + qw * ty + (qz * tx - qx * tz);
    vec_out[2] = vz + qw * tz + (qx * ty - qy * tx);
}

static void usr_q_mul(float q1[4], float q2[4], float res[4]){
    float q[4] = {0};
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

    res[0] = q[0];
    res[1] = q[1];
    res[2] = q[2];
    res[3] = q[3];
}

static void usr_q_2_euler(float q[4], float euler[3]){
    float q0q0 = SQUARE(q[0]);
    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q1 = SQUARE(q[1]);
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q2 = SQUARE(q[2]);
    float q2q3 = q[2] * q[3];
    float q3q3 = SQUARE(q[3]);

    euler[0] = fast_atan2f(2.0f * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3) * 57.3f;
    euler[1] = -asin(2.0f * (q1q3 - q0q2)) * 57.3f;
    euler[2] = fast_atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.3f;
}
