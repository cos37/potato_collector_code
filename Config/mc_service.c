#include "mc_service.h"
#include "bujin.h"
#include "imu_driver.h"
#include "sys.h"

//积分限幅
#define INTEGRAL_LIMIT 50*65536
fp16_int32_t move_vel;
fp16_int32_t pai;
Mc_State_t mc_state;

Pid_handle_t pidYaw = {
    .target = 0,
    .current = 0,
    .error = 0,
    .integral = 0,
    .derivative = 0,
    .last_error = 0
};

MtGroup_t gleft ={
    .speed = 0,
    .dir = 0,
    .id1 = 1,
    .id2 = 2
};

MtGroup_t gright ={
    .speed = 0,
    .dir = 0,
    .id1 = 3,
    .id2 = 4
};

void MC_Init(void)
{
    pai = fp16_from_float(3.1415926f);
    move_vel = fp16_from_float(50.0f); // 设定一个默认的移动速度
    pidYaw.kp = fp16_from_float(1.0f);
    pidYaw.ki = fp16_from_float(0.0f);
    pidYaw.kd = fp16_from_float(0.1f);
    mc_state = MC_STATE_DISABLED;
}

// 角度归一化到 [-pi, pi]
void Pai_cricle(fp16_int32_t *angle)
{
    if(*angle > pai) {
        *angle -= 2*pai;
    } else if(*angle < -pai) {
        *angle += 2*pai;
    }
}

void Pid_Clear_Integral(Pid_handle_t *hpid)
{
    hpid->integral = 0;
}

// PID计算函数，更新error、integral、derivative等参数
fp16_int32_t Pid_Calculate(Pid_handle_t *hpid)
{
    fp16_int32_t output;
    hpid->error = hpid->target - hpid->current;
    Pai_cricle(&hpid->error); // 误差归一化到 [-pi, pi]
    hpid->integral += hpid->error;
    // 积分限幅
    if(hpid->integral > INTEGRAL_LIMIT) {
        hpid->integral = INTEGRAL_LIMIT;
    } else if(hpid->integral < -INTEGRAL_LIMIT) {
        hpid->integral = -INTEGRAL_LIMIT;
    }

    hpid->derivative = hpid->error - hpid->last_error;
    hpid->last_error = hpid->error;

    output = fp16_mul(hpid->kp, hpid->error) + 
             fp16_mul(hpid->ki, hpid->integral) + 
             fp16_mul(hpid->kd, hpid->derivative);
    return output;
}



void Yaw_Control(fp16_int32_t target_angle, fp16_int32_t current_angle)
{
    pidYaw.target = target_angle;
    pidYaw.current = current_angle;
    fp16_int32_t diff = Pid_Calculate(&pidYaw);
    // 将PID输出转换为电机速度和方向
    if(diff >= 0) {
        gleft.dir = 1; // 正转
        gright.dir = 0; // 反转
        gleft.speed = diff;
        gright.speed = diff;
    } else if(diff < 0)
    {
        gleft.dir = 0; // 反转
        gright.dir = 1; // 正转
        gleft.speed = -diff;
        gright.speed = -diff;
    } 

}

void Mc_Update(void)
{

    Yaw_Control(pidYaw.target, pidYaw.current);

}


//写成专门的服务函数,在loop里调用，或者放到一个单独的任务里调用
//参数是目标角度,移动速度，持续时间
//状态机控制
//state: DISENABLE 没有任务
//     : ENABLE 任务开始,设定目标角度和持续时间,开启定时器
//     : RUNNING 任务进行中,持续更新PID控制
//     : END 任务结束,停止电机

void MC_DISABLE_STATE(void);
void MC_ENABLE_STATE(void);
void MC_RUNNING_STATE(void);
void MC_END_STATE(void);
void Mc_Task_IT(void);

uint16_t mc_duration_ms; // 任务持续时间
fp16_int32_t mc_target_angle; // 目标角度


void Mc_StateMachine(void)
{
    switch(mc_state) {
        case MC_STATE_DISABLED:
            MC_DISABLE_STATE();
            break;
        case MC_STATE_ENABLED:
            MC_ENABLE_STATE();
            break;
        case MC_STATE_RUNNING:
            MC_RUNNING_STATE();
            // 判断是否达到持续时间，若达到则 mc_state = MC_STATE_END;
            break;
        case MC_STATE_END:
            MC_END_STATE();
            break;
        default:
            break;
    }

}

void MC_DISABLE_STATE(void)
{
    return;
}

void MC_ENABLE_STATE(void)
{
    // 设定目标角度和持续时间
    pidYaw.target = mc_target_angle;
    //设置持续时间
    Sys_StartTimer(mc_duration_ms, Mc_Task_IT); 
    // 切换到运行状态
    mc_state = MC_STATE_RUNNING;
}

void MC_RUNNING_STATE(void)
{
    Mc_Update();
    Emm_V5_Vel_Control(1, gleft.dir, gleft.speed, 0, SET);
    Emm_V5_Vel_Control(2, gleft.dir, gleft.speed, 0, SET);
    Emm_V5_Vel_Control(3, gright.dir, gright.speed, 0, SET);
    Emm_V5_Vel_Control(4, gright.dir, gright.speed, 0, SET);
    Emm_V5_Synchronous_motion(0xFF);
}

void MC_END_STATE(void)
{
    // 停止电机
    gleft.speed = 0;
    gright.speed = 0;
    // 切换回禁用状态
    Emm_V5_Vel_Control(1, gleft.dir, gleft.speed, 0, SET);
    Emm_V5_Vel_Control(2, gleft.dir, gleft.speed, 0, SET);
    Emm_V5_Vel_Control(3, gright.dir, gright.speed, 0, SET);
    Emm_V5_Vel_Control(4, gright.dir, gright.speed, 0, SET);
    Emm_V5_Synchronous_motion(0xFF);
    
    mc_state = MC_STATE_DISABLED;
}

void Mc_Task_IT(void)
{
    //关闭定时器
    CloseTimer2();
    //切换到结束状态
    mc_state = MC_STATE_END;
}

//接口函数
void MC_Service_Enable(fp16_int32_t target_angle, uint16_t duration_ms)
{
    pidYaw.target = target_angle;
    Sys_StartTimer(duration_ms, MC_END_STATE); 
    mc_state = MC_STATE_RUNNING;
}
