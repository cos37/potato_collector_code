#include "ddp.h"
#include "ssd1306_driver.h"
#include "fixpoint.h"
#include <string.h>
#define WORD_WIDETH 6 // 每个字符占6列

/** tool functions **/
static uint8_t uint_to_str(uint32_t val, char *buf) {
    uint8_t i = 0;
    char tmp[10];
    
    do {
        tmp[i++] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    
    // 反转
    for (uint8_t j = 0; j < i; j++) {
        buf[j] = tmp[i - 1 - j];
    }
    return i;
}

void DDP_Init(DDP_Handle_t *handle)
{
    SSD1306_Driver_WriteString(handle->line_start, handle->page, handle->name, handle->name_len);
    SSD1306_Driver_WriteChar(handle->line_start + handle->name_len * 6, handle->page, ':'); // 显示冒号分隔符

}

void DDP_DisplayInt32(DDP_Handle_t *handle);
void DDP_DisplayFP16(DDP_Handle_t *handle);
void DDP_DisplayFloat(DDP_Handle_t *handle);
void DDP_DisplayString(DDP_Handle_t *handle);

void DDP_Update(DDP_Handle_t *handle) {
    switch (handle->type) {
        case DDP_TYPE_INT32:
            DDP_DisplayInt32(handle);
            break;
        case DDP_TYPE_FP16:
            DDP_DisplayFP16(handle);
            break;
        case DDP_TYPE_FLOAT:
            DDP_DisplayFloat(handle);
            break;
        case DDP_TYPE_STRING:
            DDP_DisplayString(handle);
        default:
            break;
    }
}

void DDP_DisplayInt32(DDP_Handle_t *handle) {
    uint8_t line = handle->line_start + (handle->name_len + 1) * WORD_WIDETH; // 数据显示在名字后面
    char str[12];  // 最大11位 + 负号
    uint8_t len = int32_to_str(handle->data.int_val, str);
    if(len!= handle->str_len) {
        memset(str + len, ' ', handle->str_len - len); // 用空格覆盖之前的残留字符
    }
    SSD1306_Driver_WriteIntNums(line, handle->page, handle->data.int_val);
    handle->str_len = len; // 更新长度
}

void DDP_DisplayFP16(DDP_Handle_t *handle) {
    char str[16];
    int32_t raw = (int32_t)handle->data.fp16_val;
    int32_t display_val = (raw * 100) >> 16;  // 保留2位小数
    
    uint8_t neg = (display_val < 0);
    uint32_t abs_val = neg ? -display_val : display_val;
    
    uint32_t int_part = abs_val / 100;
    uint32_t frac_part = abs_val % 100;
    
    uint8_t pos = 0;
    if (neg) str[pos++] = '-';
    
    // 整数（至少0）
    if (int_part == 0) {
        str[pos++] = '0';
    } else {
        pos += uint_to_str(int_part, &str[pos]);
    }
    
    // 小数点 + 2位小数
    str[pos++] = '.';
    str[pos++] = '0' + (frac_part / 10);
    str[pos++] = '0' + (frac_part % 10);
    
    uint8_t new_len = pos;
    
    // 清除旧内容
    if (new_len < handle->str_len) {
        memset(str + new_len, ' ', handle->str_len - new_len);
    }
    
    SSD1306_Driver_WriteString(
        handle->line_start + (handle->name_len + 1) * WORD_WIDETH,
        handle->page,
        str,
        handle->str_len > new_len ? handle->str_len : new_len
    );
    
    handle->str_len = new_len;
}

void DDP_DisplayFloat(DDP_Handle_t *handle) {
    //保留2位小数
    char str[12];  // 最大11位 + 负号 
    int32_t display_val = (int32_t)(handle->data.float_val * 100);
    uint8_t len = int32_to_str(display_val, str);
    str[len] = str[len-1]; // 最后一位数字移动到小数点后
    str[len-1] = str[len-2]; // 倒数第二位数字移动到小数点前
    str[len-2] = '.'; // 插入小数点
    if(len!= handle->str_len) {
        memset(str + len, ' ', handle->str_len - len); // 用空格覆盖之前的残留字符
    }
    SSD1306_Driver_WriteString(handle->line_start + (handle->name_len + 1) * WORD_WIDETH, handle->page, str, len);
    handle->str_len = len; // 更新长度

}

void DDP_DisplayString(DDP_Handle_t *handle) {
    SSD1306_Driver_WriteString(handle->line_start + (handle->name_len + 1) * WORD_WIDETH, handle->page, handle->data.str_val, handle->str_len);
}
