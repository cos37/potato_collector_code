#include "application.h"
#include "mc_service.h"
#include "sys.h"
#include "stm32f1xx_hal.h"
#include "imu_driver.h"
/**
 * 先水平向右移，到达第一个陇，然后
 * 向前YN方向移动
 * 
 */

#define Get_ms() HAL_GetTick()
#define Get_us() Get_Tick_us()

typedef enum 
{
    APPLICATION_STATE_IDLE = 0 , //初始状态,什么不做
    APPLICATION_STATE_ENTRY = 1 , //入场状态
    APPLICATION_STATE_KUALONG = 2 , //垮拢状态
    APPLICATION_STATE_MOVETONEXT = 3, //移动到下一个拢
    APPLICATION_STATE_CAMEBACK = 4 , //回到出发地点
}Application_State_t;

Application_State_t ast;

void APPLICATION_IDLE_FUNC(void);
void APPLICATION_ENTRY_FUNC(void);
void APPLICATION_KUALONG_FUNC(void);
void APPLICATION_MOVETONEXT_FUNC(void);
void APPLICATION_CAMEBACK_FUNC(void);

void Application_Loop(void)
{
    switch (ast)
    {
    case APPLICATION_STATE_IDLE:
        APPLICATION_IDLE_FUNC();
        break;
    case APPLICATION_STATE_ENTRY:
        APPLICATION_ENTRY_FUNC();
        break;
    case APPLICATION_STATE_KUALONG:
        APPLICATION_KUALONG_FUNC();//胯隆部分
        break;
    case APPLICATION_STATE_MOVETONEXT:
        APPLICATION_MOVETONEXT_FUNC();//移动到下一部分
        break;
    case APPLICATION_STATE_CAMEBACK:
        APPLICATION_CAMEBACK_FUNC();
        break;
    
    default:
        break;
    }
}

void APPLICATION_IDLE_FUNC(void)
{
    //什么都不干
    return;
}
/**
 * @brief 进入部分状态机函数
 */
#define ENTRY_XN_TIME 10000 //5s

void APPLICATION_ENTRY_FUNC(void)
{
    //内部状态机
    static uint8_t Entrystate = 0 ;
    static uint32_t start = 0 ;
    
    if(Entrystate == 0 ) //启动状态
    {
        
        MC_Service_Enable(imuHandle.yaw,MOVE_X_NEGATIVE,ENTRY_XN_TIME);
        Entrystate = 1 ; //更改状态

    }else if (Entrystate == 1) //
    {
        //继续执行，观察是否满足状态
        if(GetComplate_flag()==1)
        {
            Entrystate = 2 ;
            start = Get_ms();
        }
    }else if (Entrystate == 2) //结束,准备停下
    {
        if(Get_ms() - start >= 500) //0.5s
        {
            MC_Service_Disable();
            Entrystate = 0 ;
            ast = APPLICATION_STATE_KUALONG;
        }

    }
    
}

/**
 * @brief 胯隆部分的状态机函数
 */

#define TIME_LONG 50000 //5s

void APPLICATION_KUALONG_FUNC(void)
{
    static uint8_t KuaLong_State = 0;
    static uint32_t start = 0 ;
    if(KuaLong_State == 0)
    {
        MC_Service_Enable(imuHandle.yaw,MOVE_Y_NEGATIVE,TIME_LONG);
        KuaLong_State = 1 ;
    }else if (KuaLong_State == 1)
    {
        if(GetComplate_flag() == 1)
        {
            KuaLong_State = 2 ;
        }
    }else if (KuaLong_State == 2)
    {
        KuaLong_State = 0;
        ast = APPLICATION_STATE_IDLE;
    }

}


/**
 * @brief 移动到下一个状态
 */
void APPLICATION_MOVETONEXT_FUNC(void)
{

}

/**
 * @brief 返回状态机
 */

void APPLICATION_CAMEBACK_FUNC(void)
{

}

/**
 * @brief 启用接口
 */
void APP_ENABLE(void)
{
    ast = APPLICATION_STATE_ENTRY;
}

void APP_DISABLE(void)
{
    ast = APPLICATION_STATE_IDLE;
}

