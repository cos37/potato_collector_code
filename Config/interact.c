/**
 * @file    interact.c
 * @brief   人机交互模块 (按键与菜单管理)
 * @author  Qwen
 * 
 * @details 交互设计思路:
 * 1. 按键交互:
 *    - 机制: 采用“上升沿”触发逻辑（按下->释放 视为一次触发）。
 *    - 去抖: 依赖硬件电容去抖，软件状态机检测状态变化。
 *    - 映射: 3个按键 (UP/DOWN/START) 配合菜单链表。
 *      - UP/DOWN: 移动菜单指针。
 *      - START: 执行当前菜单项绑定的函数指针。
 * 
 * 2. 显示交互:
 *    - 结构: 使用双向循环链表存储菜单项。
 *    - 容量: 当前设计支持 <= 8页，无需复杂翻页逻辑。
 */

#include "stm32f1xx_hal.h"
#include "interact.h"
#include "stm32f1xx.h"
#include <stdint.h>
#include "ssd1306_driver.h"

/* ================= 宏定义与配置 ================= */
#define MENU_ITEM_NUM     2
#define MENU_ITEM_MAX     (MENU_ITEM_NUM - 1)
#define MENU_ITEM_MIN     0

/* ================= 类型定义 ================= */

/**
 * @brief 按键状态枚举
 */
typedef enum
{
    KEY_PRESSED,   ///< 按键按下状态
    KEY_RELEASED   ///< 按键释放状态
} Key_State_t;

/**
 * @brief 按键控制结构体
 * @note 包含硬件引脚信息及回调函数指针
 */
typedef struct
{
    Key_State_t state;           ///< 当前状态
    uint16_t pin;                ///< 引脚号
    GPIO_TypeDef *port;          ///< 端口号
    void (*callback)(void);      ///< 触发回调函数
} Key_t;



/**
 * @brief 菜单管理器结构体
 */
typedef struct
{
    MenuItem_t *head;            ///< 链表头指针
    MenuItem_t *current;         ///< 当前选中项指针
    uint8_t menu_index;          ///< 当前菜单索引
    uint8_t menu_num;            ///< 菜单总数
} Menu_manager_t;

/* ================= 全局变量 ================= */
Menu_manager_t menu_manager;     ///< 菜单管理实例
// uint8_t menu_index = 0;       // [已注释] 已在结构体中管理，避免重复定义

Key_t keyup, keydown, keystart;  ///< 按键实例

/* ================= 函数声明与实现 ================= */

/**
 * @brief UP键回调函数
 * @note 逻辑: 索引增加，指针下移 (根据原代码逻辑保留)
 */
void up_callback(void)
{
    if (menu_manager.menu_index < menu_manager.menu_num - 1)
    {
        menu_manager.menu_index++;
        menu_manager.current = menu_manager.current->next; // 更新当前菜单项指针
    }
    else
    {
        menu_manager.menu_index = menu_manager.menu_num - 1;
        // 到达底部，不需要更新指针
    }
}

/**
 * @brief DOWN键回调函数
 * @note 逻辑: 索引减小，指针上移 (根据原代码逻辑保留)
 */
void down_callback(void)
{
    if (menu_manager.menu_index > 0)
    {
        menu_manager.menu_index--;
        menu_manager.current = menu_manager.current->prve; // 更新当前菜单项指针
    }
    else
    {
        menu_manager.menu_index = 0;
        // 到达顶部，不需要更新指针
    }
}

/**
 * @brief START键回调函数
 * @brief 执行当前菜单项绑定的任务
 */
void start_callback(void)
{
    // 安全检查：防止空指针调用
    if (menu_manager.current != NULL && menu_manager.current->task != NULL)
    {
        menu_manager.current->task(); 
    }
}

/**
 * @brief 菜单管理器初始化
 */
void Menu_Init(void)
{
    menu_manager.head = NULL;
    menu_manager.current = NULL;
    menu_manager.menu_index = 0;
    menu_manager.menu_num = 0;
}

/**
 * @brief 向菜单链表添加新项
 * @param new_item 新的菜单项指针
 * @note 构建双向循环链表
 */
void Menu_AddItem(MenuItem_t *new_item)
{
    if (menu_manager.head == NULL)
    {
        // 链表为空，创建首节点
        menu_manager.head = new_item;
        menu_manager.current = new_item;
        new_item->next = new_item; // 指向自己
        new_item->prve = new_item; // 指向自己
    }
    else
    {
        // 插入到尾部
        // 注意: 此处变量名 tail 与类型名冲突，但为了保持原代码逻辑未修改
        Menu_manager_t *tail = menu_manager.head->prve; 
        
        tail->next = new_item;     // 原尾部 -> 新项
        new_item->prve = tail;     // 新项 -> 原尾部
        new_item->next = menu_manager.head; // 新项 -> 头部
        // menu_manager.head 保持不变
    }
}

/**
 * @brief 按键GPIO初始化
 */
void Key_Init(void)
{
    // 配置 UP 键 (PB15)
    keyup.state = KEY_RELEASED;
    keyup.pin = GPIO_PIN_15;
    keyup.port = GPIOB;
    keyup.callback = up_callback;

    // 配置 START 键 (PB14)
    keystart.state = KEY_RELEASED;
    keystart.pin = GPIO_PIN_14;
    keystart.port = GPIOB;
    keystart.callback = start_callback;

    // 配置 DOWN 键 (PB13)
    keydown.state = KEY_RELEASED;
    keydown.pin = GPIO_PIN_13;
    keydown.port = GPIOB;
    keydown.callback = down_callback;
}

/**
 * @brief 读取按键物理状态
 * @param key 按键结构体指针
 * @note 硬件低电平有效 (GPIO_PIN_RESET)
 */
void Key_Read(Key_t *key)
{
    if (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET)
    {
        key->state = KEY_PRESSED;
    }
    else
    {
        key->state = KEY_RELEASED;
    }
}

/**
 * @brief 按键状态机处理函数
 * @param key 按键结构体指针
 * 
 * @details 状态机逻辑:
 * - IDLE (RELEASED): 检测是否按下。若按下，进入 PRESSED 状态。
 * - ACTIVE (PRESSED): 等待释放。若释放，触发回调函数，并返回 IDLE。
 * 
 * @note 此逻辑实现了“释放触发”或“上升沿”效果，有效避免按键抖动干扰。
 */
void Key_State_Machine(Key_t *key)
{
    switch (key->state)
    {
        case KEY_RELEASED:
            Key_Read(key);
            if (key->state == KEY_PRESSED)
            {
                key->state = KEY_PRESSED;
            }
            break;

        case KEY_PRESSED:
            Key_Read(key);
            if (key->state == KEY_RELEASED)
            {
                key->callback();      // 执行回调函数
                key->state = KEY_RELEASED; // 复位状态
            }
            break;

        default:
            break;
    }
}
