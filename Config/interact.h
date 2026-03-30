/**
 * @brief 为交互而生,可以检测按键是否按下,目前要实现的功能有：
 *     1. 按键控制任务的开始和结束
 *     2. 任务进行中紧急进行停止
 *     3. 任务进行中可以调整目标角度和持续时间
 */

#ifndef __INTERACT_H
#define __INTERACT_H

/**
 * @brief 菜单项结构体 (双向链表节点)
 */
typedef struct MenuItem
{
    char name[16];               ///< 菜单显示名称
    void (*task)(void);          ///< 功能函数指针
    struct MenuItem *next;       ///< 指向下一项
    struct MenuItem *prve;       ///< 指向上一项 (注意: 原文拼写保留)
} MenuItem_t;


void Key_Init(void);
void Key_Update(void);
void Menu_Init(void);
void Menu_AddItem(MenuItem_t *new_item);
#endif
