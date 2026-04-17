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
    APPLICATION_STATE_KUALONG1 = 2 , //垮拢状态1
    APPLICATION_STATE_MOVETONEXT = 3, //移动到下一个拢
		APPLICATION_STATE_KUALONG2 = 4 , //垮拢状态2
    APPLICATION_STATE_CAMEBACK = 5 , //回到出发地点
}Application_State_t;

Application_State_t ast;

void APPLICATION_IDLE_FUNC(void);
void APPLICATION_ENTRY_FUNC(void);
void APPLICATION_KUALONG1_FUNC(void);
void APPLICATION_KUALONG2_FUNC(void);
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
    case APPLICATION_STATE_KUALONG1:
        APPLICATION_KUALONG1_FUNC();//胯隆部分1
        break;
    case APPLICATION_STATE_MOVETONEXT:
        APPLICATION_MOVETONEXT_FUNC();//移动到下一部分
        break;
		case APPLICATION_STATE_KUALONG2:
        APPLICATION_KUALONG2_FUNC();//胯隆部分2
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
        
        MC_Service_Enable(imuHandle.yaw,MOVE_X_POSITIVE,ENTRY_XN_TIME);
        Mechanism_Motor5_Control(0, 30);
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
            ast = APPLICATION_STATE_KUALONG1;
        }

    }
    
}

/**
 * @brief 胯隆部分的状态机函数
 */

#define TIME_LONG 30000 //5s

void APPLICATION_KUALONG1_FUNC(void)
{
    static uint8_t KuaLong_State = 0;
    static uint32_t start = 0 ;
    
    if(KuaLong_State == 0)
    {
        MC_Service_Enable(imuHandle.yaw, MOVE_Y_NEGATIVE, TIME_LONG);
        KuaLong_State = 1 ;
    }
    else if (KuaLong_State == 1)
    {
        if(GetComplate_flag() == 1)
        {
            KuaLong_State = 2 ;
            start = Get_ms(); // 记录停下的瞬间
        }
    }
    else if (KuaLong_State == 2)
    {
        // 缓冲 0.5 秒
        if (Get_ms() - start >= 500) 
        {
            MC_Service_Disable();
            KuaLong_State = 0;
            
            // 跨陇1结束，交接给下一个任务：换陇
            ast = APPLICATION_STATE_MOVETONEXT; 
        }
    }
}


/**
 * @brief 移动到下一个状态
 */

#define TIME_MOVETONEXT 10000 

void APPLICATION_MOVETONEXT_FUNC(void)
{
    static uint8_t MoveNextState = 0;
    static uint32_t start = 0;

    if (MoveNextState == 0) // 第0步：启动状态
    {
        // 告诉底层：保持当前车头角度，向右平移，运行 500ms
        MC_Service_Enable(imuHandle.yaw, MOVE_X_POSITIVE, TIME_MOVETONEXT);
        MoveNextState = 1; // 指令下达完毕，切到监工状态
        
    }
    else if (MoveNextState == 1) // 第1步：监工等待
    {
        // 观察底层是否跑完了这 0.5 秒
        if (GetComplate_flag() == 1)
        {
            MoveNextState = 2; // 跑完了，准备进入缓冲休息
            start = Get_ms();  // 掐下秒表，记录停下来的瞬间
        }
        
    }
    else if (MoveNextState == 2) // 第2步：缓冲休息与交接
    {
        // 给小车 0.5s 的时间让车身晃动平息下来（极大地提高下一步的精度）
        if (Get_ms() - start >= 500) 
        {
            MC_Service_Disable(); // 彻底锁死电机
            MoveNextState = 0;    // 自己内部的状态机归零，方便下次再次调用
            
            ast = APPLICATION_STATE_KUALONG2; 
        }
    }
}


/**
 * @brief 胯隆部分的状态机函数
 */

void APPLICATION_KUALONG2_FUNC(void)
{
    static uint8_t KuaLong_State = 0;
    static uint32_t start = 0 ;
    
    if(KuaLong_State == 0)
    {
        MC_Service_Enable(imuHandle.yaw, MOVE_Y_POSITIVE, TIME_LONG);
        KuaLong_State = 1 ;
    }
    else if (KuaLong_State == 1)
    {
        if(GetComplate_flag() == 1)
        {
            KuaLong_State = 2 ;
            start = Get_ms(); // 记录停下的瞬间
        }
    }
    else if (KuaLong_State == 2)
    {
        // 缓冲 0.5 秒
        if (Get_ms() - start >= 500) 
        {
            MC_Service_Disable();
            KuaLong_State = 0;
            
            // 跨陇2结束，所有的陇都收完了，交接给最终任务：返航
            ast = APPLICATION_STATE_CAMEBACK; 
        }
    }
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

