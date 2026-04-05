/**
 *    左  右
 * 前  3   4
 * 后  1   2
 */
#include "mecanum.h"
#include "Bujin.h"
#include "fixpoint.h"

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


}

void Mecanum_kinematics(fp16_int32_t vx,fp16_int32_t vy,fp16_int32_t omega)
{


    fp16_int32_t K = mecaunm.L + mecaunm.W;  // L+W = 0.415

    // 计算 K*omega (Q32.32 → Q16.16)
    int64_t k_omega = ((int64_t)K * (int64_t)omega) >> 16;

    // 四个轮子的线速度 (Q16.16 m/s)
    int64_t v_fl = (int64_t)vx - (int64_t)vy - k_omega;  // 前左: vx - vy - (L+W)*ω
    int64_t v_fr = -((int64_t)vx + (int64_t)vy + k_omega);  // 前右: vx + vy + (L+W)*ω  
    int64_t v_ll = (int64_t)vx + (int64_t)vy - k_omega;  // 后左: vx + vy - (L+W)*ω
    int64_t v_lr = -((int64_t)vx - (int64_t)vy + k_omega);  // 后右: vx - vy + (L+W)*ω

    // 除以 R 得到轮转速 (rad/s)，结果 Q16.16
    int64_t w_fl = (v_fl << 16) / mecaunm.R;
    int64_t w_fr = (v_fr << 16) / mecaunm.R;
    int64_t w_ll = (v_ll << 16) / mecaunm.R;
    int64_t w_lr = (v_lr << 16) / mecaunm.R;

    // 存入结构体 (分离方向与绝对值)
    mecaunm.flw.speed = (uint16_t)((w_fl < 0) ? -w_fl : w_fl);
    mecaunm.flw.dir   = (w_fl < 0) ? 0 : 1;

    mecaunm.frw.speed = (uint16_t)((w_fr < 0) ? -w_fr : w_fr);
    mecaunm.frw.dir   = (w_fr < 0) ? 0 : 1;

    mecaunm.llw.speed = (uint16_t)((w_ll < 0) ? -w_ll : w_ll);
    mecaunm.llw.dir   = (w_ll < 0) ? 0 : 1;

    mecaunm.lrw.speed = (uint16_t)((w_lr < 0) ? -w_lr : w_lr);
    mecaunm.lrw.dir   = (w_lr < 0) ? 0 : 1;
}

void Mecanum_Update(void)
{
    Emm_V5_Vel_Control(mecaunm.flw.id,mecaunm.flw.dir,mecaunm.flw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.frw.id,mecaunm.frw.dir,mecaunm.frw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.llw.id,mecaunm.llw.dir,mecaunm.llw.speed,0,RESET);
    Emm_V5_Vel_Control(mecaunm.lrw.id,mecaunm.lrw.dir,mecaunm.lrw.speed,0,RESET);

    Emm_V5_Synchronous_motion(0xFF);
}
