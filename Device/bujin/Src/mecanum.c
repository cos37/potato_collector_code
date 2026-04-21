/**
 *    左  右
 * 前  3   4
 * 后  1   2
 */
#include "mecanum.h"
#include "Bujin.h"
#include "fixpoint.h"
#include "sys.h"

// 转换系数：rad/s (Q16.16) → RPM (整数)
// 9.5493 * 65536 ≈ 625762
#define RAD_TO_RPM_FIX  625762  

typedef struct 
{
    uint8_t id;
    uint16_t speed;
    uint8_t dir;
}Wheel_t;


typedef struct 
{
    Wheel_t flw;
    Wheel_t frw;
    Wheel_t llw;
    Wheel_t lrw;
    fp16_int32_t L;
    fp16_int32_t W;
    fp16_int32_t R;
}Mecanum_t;

Mecanum_t mecaunm;

void Mecanum_Init(void)
{
    mecaunm.flw.dir = 0;
    mecaunm.flw.id = 3;
    mecaunm.flw.speed = 0;

    mecaunm.frw.dir = 0;
    mecaunm.frw.id = 4;
    mecaunm.frw.speed = 0;

    mecaunm.llw.dir = 0 ;
    mecaunm.llw.id = 1 ;
    mecaunm.llw.speed = 0;

    mecaunm.lrw.dir = 0;
    mecaunm.lrw.id = 2;
    mecaunm.lrw.speed = 0;

    mecaunm.L = fp16_from_float(0.29);
    mecaunm.R = fp16_from_float(0.125);
    mecaunm.W = fp16_from_float(0.039);

    // Motor_Buf_Init();

}

void Mecanum_kinematics(fp16_int32_t vx,fp16_int32_t vy,fp16_int32_t omega)
{


    fp16_int32_t K = mecaunm.L + mecaunm.W;  // L+W = 0.415

    // 计算 K*omega (Q32.32 → Q16.16)
    int64_t k_omega = ((int64_t)K * (int64_t)omega) >> 16;

    // 四个轮子的线速度 (Q16.16 m/s)
    int64_t v_fl = -((int64_t)vx - (int64_t)vy - k_omega);  // 前左: vx - vy - (L+W)*ω
    int64_t v_fr = ((int64_t)vx + (int64_t)vy + k_omega);  // 前右: vx + vy + (L+W)*ω  
    int64_t v_ll = -((int64_t)vx + (int64_t)vy - k_omega);  // 后左: vx + vy - (L+W)*ω
    int64_t v_lr = ((int64_t)vx - (int64_t)vy + k_omega);  // 后右: vx - vy + (L+W)*ω

    // 除以 R 得到轮转速 (rad/s)，结果 Q16.16
    int64_t w_fl = (v_fl << 16) / mecaunm.R;
    int64_t w_fr = (v_fr << 16) / mecaunm.R;
    int64_t w_ll = (v_ll << 16) / mecaunm.R;
    int64_t w_lr = (v_lr << 16) / mecaunm.R;

    int32_t rpm_fl = (int32_t)((w_fl * RAD_TO_RPM_FIX) >> 16);
    int32_t rpm_fr = (int32_t)((w_fr * RAD_TO_RPM_FIX) >> 16);
    int32_t rpm_ll = (int32_t)((w_ll * RAD_TO_RPM_FIX) >> 16);
    int32_t rpm_lr = (int32_t)((w_lr * RAD_TO_RPM_FIX) >> 16);

    // 存入结构体（现在 speed 是 RPM，右移16位）
    mecaunm.flw.speed = (uint16_t)(((rpm_fl < 0) ? -rpm_fl : rpm_fl) >> 16);
    mecaunm.flw.dir   = (rpm_fl < 0) ? 0 : 1;

    mecaunm.frw.speed = (uint16_t)(((rpm_fr < 0) ? -rpm_fr : rpm_fr) >> 16);
    mecaunm.frw.dir   = (rpm_fr < 0) ? 0 : 1;

    mecaunm.llw.speed = (uint16_t)(((rpm_ll < 0) ? -rpm_ll : rpm_ll) >> 16);
    mecaunm.llw.dir   = (rpm_ll < 0) ? 0 : 1;

    mecaunm.lrw.speed = (uint16_t)(((rpm_lr < 0) ? -rpm_lr : rpm_lr) >> 16);
    mecaunm.lrw.dir   = (rpm_lr < 0) ? 0 : 1;
    
}

void Mecanum_Update(void)
{
    
    Emm_V5_Vel_Control(mecaunm.flw.id,mecaunm.flw.dir,mecaunm.flw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.frw.id,mecaunm.frw.dir,mecaunm.frw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.llw.id,mecaunm.llw.dir,mecaunm.llw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.lrw.id,mecaunm.lrw.dir,mecaunm.lrw.speed,0,RESET);

//    Emm_V5_Synchronous_motion(0xFF);
	HAL_Delay(3);
    // Motor_Set_Vel(mecaunm.flw.id,mecaunm.flw.dir,mecaunm.flw.speed);
    // Motor_Set_Vel(mecaunm.frw.id,mecaunm.frw.dir,mecaunm.frw.speed);
    // Motor_Set_Vel(mecaunm.llw.id,mecaunm.llw.dir,mecaunm.llw.speed);
    // Motor_Set_Vel(mecaunm.lrw.id,mecaunm.lrw.dir,mecaunm.lrw.speed);

}

// void Mecanum_DMA_Mode_Enable(void)
// {
//     Sys_startTimer3(20,DMA_State_Machine);
// }

