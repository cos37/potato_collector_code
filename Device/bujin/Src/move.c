#include "move.h"
#include "Bujin.h"
#include "usart.h"

#define CW 0
#define CCW 1
/**
 *  1    2
 *  /    /
 *  /    /
 *  /    /
 *  /    /
 *  /    /
 *  /    /
 *  4    3
 */

void Init(void)
{
    Emm_V5_En_Control(1,1,1);
    Emm_V5_En_Control(2,1,1);
    Emm_V5_En_Control(3,1,1);
    Emm_V5_En_Control(4,1,1);
    Emm_V5_Synchronous_motion(0xFF);
}

void Move_X(uint16_t speed,Dir_t dir)
{
    uint8_t step13_dir ;
    uint8_t step24_dir ;
    if(dir==POSITIVE)
    {
        step13_dir = CW;
        step24_dir = CCW;
    }else{
        step13_dir = CCW;
        step24_dir = CW;
    }
    Emm_V5_Vel_Control(1,step13_dir,speed,5,RESET);
    Emm_V5_Vel_Control(2,step24_dir,speed,5,RESET);
    Emm_V5_Vel_Control(3,step13_dir,speed,5,RESET);
    Emm_V5_Vel_Control(4,step24_dir,speed,5,RESET);
    Emm_V5_Synchronous_motion(0xFF);

}

void Move_Y(uint16_t speed,Dir_t dir)
{
    uint8_t step12_dir ;
    uint8_t step34_dir ;
    if(dir==POSITIVE)
    {
        step12_dir = CW;
        step34_dir = CCW;
    }else{
        step12_dir = CCW;
        step34_dir = CW;
    }
    Emm_V5_Vel_Control(1,step12_dir,speed,5,RESET);
    Emm_V5_Vel_Control(2,step12_dir,speed,5,RESET);
    Emm_V5_Vel_Control(3,step34_dir,speed,5,RESET);
    Emm_V5_Vel_Control(4,step34_dir,speed,5,RESET);
    Emm_V5_Synchronous_motion(0xFF);
}

void Move_STOP(void)
{
    Emm_V5_Stop_Now(1,RESET);
    Emm_V5_Stop_Now(2,RESET);
    Emm_V5_Stop_Now(3,RESET);
    Emm_V5_Stop_Now(4,RESET);
    Emm_V5_Synchronous_motion(0xFF);
}

void Move_Init(MoveHandle_t *hmov)
{
    hmov->init=&Init;
    hmov->move_x=&Move_X;
    hmov->move_y=&Move_Y;
    hmov->stop=&Move_STOP;

    hmov->init();

}