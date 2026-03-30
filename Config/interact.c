/**思考中:怎么进行交互
 * 1.按键交互
 *  要有按键检测函数,可以检测按键是否按下,硬件自带去抖电容
 *  要不要加菜单？目前应该没必要，但是确实需要在菜单上显示按键信息，
 *  比如显示 "task_start" "task_stop" "adjust_target_angle" "adjust_duration" 等等
 * 按键是否只需要对应一个操作？
 * 可以这样子去设计:目前有三个按钮,我们就让它为up/down/start,设计专门的指针去指向当前的操作
 * 按键检测我们可以抛弃长按短按的概念,只要按下就执行对应的操作,up/down 是在菜单上下移动, start 是执行当前菜单项的操作
 * 怎么触发按键？就像通信协议里，是上升沿有效，当检测到按键从未按下到按下的状态变化时，就触发一次操作
 * 2.显示交互
 * 目前菜单还不会显示太多功能,所有8page是足够的，还不需要翻页了,
 * 我们可以创建一个链表来存储菜单项,每个菜单项包含一个字符串和一个函数指针,当菜单项被选中时,按下start键就执行对应的函数
 * 
 *  检测后如何优雅地去执行相应任务？回调函数怎么样？
 * 我们可以直接使用函数指针,改变loop函数的指向来改变当前执行的任务
*/
#include "stm32f1xx_hal.h"
#include "interact.h"
#include "stm32f1xx.h"
#include <stdint.h>
#include "ssd1306_driver.h"

#define MENU_ITEM_NUM 2
#define MENU_ITEM_MAX (MENU_ITEM_NUM - 1)
#define MENU_ITEM_MIN 0

typedef enum{
    KEY_PRESSED,
    KEY_RELEASED
} Key_State_t;

typedef struct{
    Key_State_t state;
    uint16_t pin;
    GPIO_TypeDef *port;
    void (*callback)(void);
}Key_t;

typedef struct MenuItem{
    char name[16];
    void (*task)(void);
    struct MenuItem *next;
    struct MenuItem *prve;    
}MenuItem_t;

typedef struct 
{
    MenuItem_t *head;
    MenuItem_t *current;
    uint8_t menu_index;
    uint8_t menu_num;

}Menu_manager_t;

Menu_manager_t menu_manager;

uint8_t menu_index = 0; // 当前菜单索引

void up_callback(void)
{
    if(menu_manager.menu_index < menu_manager.menu_num - 1)
    {
        menu_manager.menu_index++;
        menu_manager.current = menu_manager.current->next; // 更新当前菜单项指针
    }else{
        menu_manager.menu_index = menu_manager.menu_num - 1;
        //不需要更新
    }
}

void down_callback(void)
{
    if(menu_manager.menu_index > 0)
    {
        menu_manager.menu_index--;
        menu_manager.current = menu_manager.current->prve; // 更新当前菜单项指针
    }else{
        menu_manager.menu_index = 0;
        //不需要更新
    }
}

void start_callback(void)
{
    if(menu_manager.current != NULL && menu_manager.current->task != NULL)//防它一手
    {
        menu_manager.current->task(); // 执行当前菜单项的任务
    }
}

void Menu_Init(void)
{
    menu_manager.head = NULL;
    menu_manager.current = NULL;   
    menu_manager.menu_index = 0;
    menu_manager.menu_num = 0;

}

void Menu_AddItem(MenuItem_t *new_item)
{
    if(menu_manager.head == NULL)
    {
        menu_manager.head = new_item;
        menu_manager.current = new_item;
        new_item->next = new_item; // 形成循环链表
        new_item->prve = new_item;

    }else{
        Menu_manager_t *tail = menu_manager.head->prve; // 获取当前链表的尾部
        tail->next = new_item; // 尾部指向新项
        new_item->prve = tail; // 新项的前驱指向原尾部
        new_item->next = menu_manager.head; // 新项的后继指向头部
        //head是不变的，还是指向原来的头部
    }
}

Key_t keyup,keydown,keystart;

void Key_Init(void)
{
    keyup.state = KEY_RELEASED;
    keyup.pin = GPIO_PIN_15;
    keyup.port = GPIOB;
    keyup.callback = up_callback;

    keystart.state = KEY_RELEASED;
    keystart.pin = GPIO_PIN_14;
    keystart.port = GPIOB;
    keystart.callback = start_callback;

    keydown.state = KEY_RELEASED;
    keydown.pin = GPIO_PIN_13;
    keydown.port = GPIOB;
    keydown.callback = down_callback;

}

void Key_Read(Key_t *key)
{
    if(HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET) // 按键按下
    {
        key->state = KEY_PRESSED;
    }
    else
    {
        key->state = KEY_RELEASED;
    }
}
/**
 * IDLE: 检测按键是否按下按下进入下个阶段
 * Ready: 检测按键是否释放,没有释放就啥也不干,释放了就执行相应操作并且跳转状态机回到IDLE
 * 这样就实现了上升沿触发的按键操作,并且不需要担心按键抖动的问题,因为只有在按键释放时才会执行操作
 * 这里我们就使用按钮的结构体,没必要使用新的,节省空间
 */
void Key_State_Machine(Key_t *key)
{
    switch (key->state)
    {
    case KEY_RELEASED:
        Key_Read(key);
        if(key->state == KEY_PRESSED)
        {
            key->state = KEY_PRESSED;
        }
        break;
    case KEY_PRESSED:
        Key_Read(key);
        if(key->state == KEY_RELEASED)
        {
           key->callback(); // 执行回调函数
           key->state = KEY_RELEASED;
        }
        break;
    default:
        break;
    }

}

