/**
 * @brief ddp is date displayer and processor, 
 * which is responsible for processing the data and displaying it on the screen.
 * @author cos37
 */

#ifndef __DDP_H__
#define __DDP_H__

#include <stdint.h>
#include "fixpoint.h" /* 包含定点数类型定义 */

typedef enum {
    DDP_TYPE_INT32,     // 整数（可配置为Q16.16显示）
    DDP_TYPE_FP16,      // 定点数 Q16.16
    DDP_TYPE_FLOAT,     // 浮点数
    DDP_TYPE_STRING,    // 静态字符串
} DDP_Type_t;

typedef struct{
    uint8_t page;
    uint8_t line_start;
    uint8_t str_len;
    char* name;
    uint8_t name_len;
    DDP_Type_t type;
    union {
        int32_t int_val;
        fp16_int32_t fp16_val;
        float float_val;
        char *str_val;
    } data;
}DDP_Handle_t;

void DDP_Init(DDP_Handle_t *handle);
void DDP_Update(DDP_Handle_t *handle);

#endif /* __DDP_H__ */
