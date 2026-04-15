#include "mc_service.h"
#include "bujin.h"
#include "imu_driver.h"
#include "sys.h"
#include "ssd1306_driver.h"
#include "mecanum.h"
#include "fixpoint.h"
//#include "flash.h"

// 积分限幅
#define INTEGRAL_LIMIT 25000       // 0.38 rad/s 积分限幅
#define MAX_SPEED 50000      // 0.765 rad/s ≈ 43.5°/s
#define DEG180 11796480  // 180度对应的定点数值，180° = π rad = 3.1415926 rad，定点数表示为 int32_t，乘以65536
fp16_int32_t move_vel;
fp16_int32_t pai;
fp16_int32_t deg180 ;
Mc_State_t mc_state;
uint16_t mc_duration_ms;       // 任务持续时间
fp16_int32_t mc_target_angle;  // 目标角度
uint8_t compelate_flag = 0;
/**
 * 
 * right 1 , 2,
 */
Pid_handle_t pidYaw = {
    .target = 0,
    .current = 0,
    .error = 0,
    .integral = 0,
    .derivative = 0,
    .last_error = 0
};


void (*MC_RUNNING_FUNCTION)(const fp16_int32_t target_angle, const fp16_int32_t current_angle);
void Move_Y_POSITIVE(const fp16_int32_t target_angle, const fp16_int32_t current_angle);
void Move_Y_NEGATIVE(const fp16_int32_t target_angle, const fp16_int32_t current_angle);
void Move_X_Positive(const fp16_int32_t target_angle, const fp16_int32_t current_angle);
void Move_X_Negative(const fp16_int32_t target_angle, const fp16_int32_t current_angle);
void MC_DISABLE_FUNC(void);
void MC_ENABLE_FUNC(void);
void MC_RUNNING_FUNC(void);
void MC_END_FUNC(void);
void Mc_Task_IT(void);
void Mc_Soft_Task_IT(void);




// static uint32_t abs_speed(fp16_int32_t speed)
// {
//     return (speed >= 0) ? speed : -speed;
// }

void MC_Init(void)
{
    pai = fp16_from_float(3.1415926f);
    move_vel = fp16_from_float(0.5f);  // 设定一个默认的移动速度
    pidYaw.kp = fp16_from_float(2.5f);
    pidYaw.ki = fp16_from_float(0.0f);
    pidYaw.kd = fp16_from_float(3.5f);

    Mecanum_Init();

    Emm_V5_En_Control(1, SET, SET);
    Emm_V5_En_Control(2, SET, SET);
    Emm_V5_En_Control(3, SET, SET);
    Emm_V5_En_Control(4, SET, SET);
    Emm_V5_Synchronous_motion(0xFF);
    mc_state = MC_STATE_DISABLED;
    MC_RUNNING_FUNCTION = Move_Y_NEGATIVE;
}

// 角度归一化到 [-pi, pi]
void Pai_cricle(fp16_int32_t *angle)
{
    while (*angle>=DEG180)
    {
        *angle -= 2 * DEG180;
    }
    while (*angle <=-DEG180)
    {
        *angle += 2 * DEG180; 
    }
}



// PID计算函数，更新error、integral、derivative等参数
fp16_int32_t Pid_Calculate(Pid_handle_t *hpid)
{
    fp16_int32_t output;
    
    hpid->error = hpid->target - hpid->current;
    Pai_cricle(&hpid->error);  // 误差归一化到 [-pi, pi]
    hpid->integral += hpid->error;
    // 积分限幅
    if (hpid->integral > INTEGRAL_LIMIT) {
        hpid->integral = INTEGRAL_LIMIT;
    } else if (hpid->integral < -INTEGRAL_LIMIT) {
        hpid->integral = -INTEGRAL_LIMIT;
    }

    hpid->derivative = hpid->error - hpid->last_error;
    hpid->last_error = hpid->error;

    output = fp16_mul(hpid->kp, hpid->error) + 
             fp16_mul(hpid->ki, hpid->integral) + 
             fp16_mul(hpid->kd, hpid->derivative);

    if(output>MAX_SPEED)
    {
        output=MAX_SPEED;
    }else if (output<-MAX_SPEED)
    {
        output=-MAX_SPEED;
    }
    

    return output;
}

/**
 * RUNNING STATE FUNCTION STATRT
 */
void Move_Y_POSITIVE(const fp16_int32_t target_angle, const fp16_int32_t current_angle)
{
    pidYaw.target = target_angle;
    pidYaw.current = current_angle;
    // SSD1306_Driver_WriteFP16(0,6,abs_speed(target_angle));
    // SSD1306_Driver_WriteFP16(0,7,abs_speed(current_angle));
    fp16_int32_t omega = Pid_Calculate(&pidYaw);
    Mecanum_kinematics(0,move_vel,-omega);
}

void Move_Y_NEGATIVE(const fp16_int32_t target_angle, const fp16_int32_t current_angle)
{
    pidYaw.target = target_angle;
    pidYaw.current = current_angle;
    // SSD1306_Driver_WriteFP16(0,6,abs_speed(target_angle));
    // SSD1306_Driver_WriteFP16(0,7,abs_speed(current_angle));
    fp16_int32_t omega = Pid_Calculate(&pidYaw);
    Mecanum_kinematics(0,-move_vel,-omega);
}

void Move_X_Positive(const fp16_int32_t target_angle, const fp16_int32_t current_angle)
{
    pidYaw.target = target_angle;
    pidYaw.current = current_angle; 
    fp16_int32_t omega = Pid_Calculate(&pidYaw);
    Mecanum_kinematics(move_vel,0,-omega);
}

void Move_X_Negative(const fp16_int32_t target_angle, const fp16_int32_t current_angle)
{
    pidYaw.target = target_angle;
    pidYaw.current = current_angle; 
    fp16_int32_t omega = Pid_Calculate(&pidYaw);
    Mecanum_kinematics(-move_vel,0,-omega);
}

/**
 * RUNNING STATE FUNCTION END
 */

 /**
  * STATE FUNC CHANGE FUNC START
  */



 void MC2XP(void)
 {
    MC_RUNNING_FUNCTION = Move_X_Positive;
 }

 void MC2XN(void)
 {
    MC_RUNNING_FUNCTION = Move_X_Negative;
 }

 void MC2YP(void)
 {
    MC_RUNNING_FUNCTION = Move_Y_POSITIVE;
 }

 void MC2YN(void)
 {
    MC_RUNNING_FUNCTION = Move_Y_NEGATIVE;
 }


 /**
  * STATE FUNC CHANGE FUNC END
  */




// 写成专门的服务函数,在loop里调用，或者放到一个单独的任务里调用
// 参数是目标角度,移动速度，持续时间
// 状态机控制
// state: DISENABLE 没有任务
//      : ENABLE 任务开始,设定目标角度和持续时间,开启定时器
//      : RUNNING 任务进行中,持续更新PID控制
//      : END 任务结束,停止电机





void Mc_StateMachine(void)
{
    switch (mc_state) {
        case MC_STATE_DISABLED:
            MC_DISABLE_FUNC();
            break;
        case MC_STATE_ENABLED:
            MC_ENABLE_FUNC();
            break;
        case MC_STATE_RUNNING:
            MC_RUNNING_FUNC();
            break;
        case MC_STATE_END:
            MC_END_FUNC();
            break;
        default:
            break;
    }
}

void MC_DISABLE_FUNC(void)
{
    // HAL_Delay(40);
    return;
}

void MC_ENABLE_FUNC(void)
{
    // 设定目标角度和持续时间
    pidYaw.target = mc_target_angle;
    pidYaw.current = 0;
    pidYaw.derivative = 0;
    pidYaw.error = 0;
    pidYaw.last_error = 0;
    pidYaw.integral = 0;
    // 切换到运行状态
    mc_state = MC_STATE_RUNNING;
    Sys_SoftTime_Start(Mc_Soft_Task_IT);


}
volatile uint16_t speed_group_left;
volatile uint16_t speed_group_right;
void MC_RUNNING_FUNC(void)
{
    Move_Y_NEGATIVE(mc_target_angle,imuHandle.yaw);
    Mecanum_Update();
}

void MC_END_FUNC(void)
{
    pidYaw.integral =  0;
    pidYaw.derivative = 0;
    pidYaw.last_error = 0;


    Mecanum_kinematics(0,0,0);
    Mecanum_Update();
    mc_state = MC_STATE_DISABLED;


}




void Mc_Task_IT(void)
{
    // 关闭定时器
    CloseTimer2();
    // 切换到结束状态
    mc_state = MC_STATE_END;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); // 任务结束，点亮LED

}



void Mc_Soft_Task_IT(void)
{
    static uint32_t count = 0;
    count++;
    if(count>=mc_duration_ms)
    {
        count = 0 ;
        Mc_Task_IT();
        compelate_flag = 1;
    }
}

uint8_t GetComplate_flag(void)
{
    return compelate_flag;
}

// 接口函数
void MC_Service_Enable(fp16_int32_t target_angle,RUNNING_STATE_t dir,fp16_int32_t destinantion)
{
    mc_target_angle = target_angle;
    //计算运行的时间
    fp16_int32_t time = fp16_div(destinantion,move_vel)*1000;
    mc_duration_ms = time>>16;
    //根据dir换方向
    if(dir == Move_X_Positive)
    {
        MC2XP();
    }else if (dir == Move_X_Negative)
    {
        MC2XN();
    }else if (dir == Move_Y_POSITIVE)
    {
        MC2YP();
    }else if (dir == Move_Y_NEGATIVE)
    {
        MC2YN();
    }
    
    
    compelate_flag = 0 ;
    mc_state = MC_STATE_ENABLED;
}


void MC_Service_Disable(void)
{
    mc_state = MC_STATE_END;
    compelate_flag = 1;
}

