#ifndef _FIXPOINT_H_
#define _FIXPOINT_H_
/**
 * @file fixpoint.h
 * @brief 定义定点数类型和相关操作函数
 */

#include <stdint.h> /* 包含标准整数类型定义 */

typedef int32_t fp16_int32_t; /* 定点数类型，使用32位整数表示 */


/******************** 定点数操作函数声明 ********************/

static inline fp16_int32_t fp16_from_float(float f) {
    return (fp16_int32_t)(f * 65536.0f); /* 将浮点数转换为定点数，乘以2^16 */
}

static inline float fp16_to_float(fp16_int32_t fp) {
    return (float)fp / 65536.0f; /* 将定点数转换为浮点数，除以2^16 */
}

static inline fp16_int32_t fp16_add(fp16_int32_t a, fp16_int32_t b) {
    return a + b; /* 定点数加法 */
}

static inline fp16_int32_t fp16_sub(fp16_int32_t a, fp16_int32_t b) {
    return a - b; /* 定点数减法 */
}

static inline fp16_int32_t fp16_mul(fp16_int32_t a, fp16_int32_t b) {
    return (fp16_int32_t)(((int64_t)a * (int64_t)b) >> 16); /* 定点数乘法，结果右移16位 */
}

static inline fp16_int32_t fp16_div(fp16_int32_t a, fp16_int32_t b) {
    return (fp16_int32_t)(((int64_t)a << 16) / (int64_t)b); /* 定点数除法，结果左移16位 */
}


#endif /* _FIXPOINT_H_ */
