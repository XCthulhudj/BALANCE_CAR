#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define FAST_MATH_ENABLED           1
#define USE_APPROXIMATIONS          1

#ifndef M_PI
    #define M_PI                    3.14159265358979323846
#endif
#ifndef M_PI_2
    #define M_PI_2                  1.57079632679489661923
#endif
#ifndef M_PI_4
    #define M_PI_4                  0.78539816339744830962
#endif
#ifndef M_2_PI
    #define M_2_PI                  6.28318530717958647692
#endif
#ifndef M_E
    #define M_E                     2.71828182845904523536
#endif

#define FLOAT_EPSILON               1e-6f
#define DOUBLE_EPSILON              1e-12

#define DEG_TO_RAD(deg)             ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(rad)             ((rad) * 180.0 / M_PI)

#define INT32_MAX_VALUE             0x7FFFFFFF
#define INT32_MIN_VALUE             0x80000000
#define FLOAT_MAX_VALUE             3.402823466e+38f
#define FLOAT_MIN_VALUE             1.175494351e-38f

// 数学运算结果结构
typedef struct {
    float value;
    int error_code;
    bool is_valid;
    TickType_t timestamp;
} math_result_t;

// 向量运算结构
typedef struct {
    float x;
    float y;
    float z;
} vector3d_t;

// 矩阵运算结构
typedef struct {
    float data[3][3];
    uint8_t rows;
    uint8_t cols;
} matrix3x3_t;

// 四元数结构
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

// 坐标系变换参数
typedef struct {
    matrix3x3_t rotation;
    vector3d_t translation;
    vector3d_t scale;
    bool is_identity;
} transform_t;

static inline int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static inline float fabsf_custom(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline int32_t min32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

static inline int32_t max32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}

static inline float fminf_custom(float a, float b)
{
    return (a < b) ? a : b;
}

static inline float fmaxf_custom(float a, float b)
{
    return (a > b) ? a : b;
}

static inline int32_t clamp32(int32_t value, int32_t min, int32_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline float fclampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline bool float_is_zero(float value)
{
    return fabsf_custom(value) < FLOAT_EPSILON;
}

static inline bool float_equal(float a, float b)
{
    return fabsf_custom(a - b) < FLOAT_EPSILON;
}

static inline float fast_sqrtf(float x)
{
#if USE_APPROXIMATIONS
    // 快速平方根近似算法（精度与性能的平衡）
    union {
        int i;
        float f;
    } u;
    
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    return x * u.f * (1.5f - 0.5f * x * u.f * u.f);
#else
    return sqrtf(x);
#endif
}

static inline float fast_atan2f(float y, float x)
{
#if USE_APPROXIMATIONS
    // 快速反正切近似，精度约0.01弧度
    float abs_x = fabsf_custom(x);
    float abs_y = fabsf_custom(y);
    float a = fminf_custom(abs_x, abs_y) / fmaxf_custom(abs_x, abs_y);
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    
    if (abs_y > abs_x) r = M_PI_2 - r;
    if (x < 0) r = M_PI - r;
    if (y < 0) r = -r;
    
    return r;
#else
    return atan2f(y, x);
#endif
}

static inline float fast_sinf(float x)
{
#if USE_APPROXIMATIONS
    // 将x限制在[-π, π]范围内
    x = fmodf(x, M_2_PI);
    if (x < 0) x += M_2_PI;
    
    // 使用5阶多项式近似
    if (x > M_PI) {
        x = M_2_PI - x;
        x = -x;
    }
    
    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x2 * x3;
    
    return x - x3/6.0f + x5/120.0f;
#else
    return sinf(x);
#endif
}

static inline float fast_cosf(float x)
{
#if USE_APPROXIMATIONS
    return fast_sinf(x + M_PI_2);
#else
    return cosf(x);
#endif
}

static inline float calculate_distance2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return fast_sqrtf(dx * dx + dy * dy);
}

static inline float calculate_distance3d(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return fast_sqrtf(dx * dx + dy * dy + dz * dz);
}

// typedef struct {
//     float axes[NUM_AXES];                // 各轴坐标值
// } coordinate_t;
// static inline void set_coordinate_axis(coordinate_t *coord, int axis_id, float value)
// {
//     if (coord != NULL && axis_id >= 0 && axis_id < NUM_AXES) {
//         coord->axes[axis_id] = value;
//     }
// }
// static inline float get_coordinate_axis(const coordinate_t *coord, int axis_id)
// {
//     if (coord != NULL && axis_id >= 0 && axis_id < NUM_AXES) {
//         return coord->axes[axis_id];
//     }
//     return 0.0f;
// }
// static inline void add_to_coordinate_axis(coordinate_t *coord, int axis_id, float value)
// {
//     if (coord != NULL && axis_id >= 0 && axis_id < NUM_AXES) {
//         coord->axes[axis_id] += value;
//     }
// }

#ifdef __cplusplus
}
#endif

