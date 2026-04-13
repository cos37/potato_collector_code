#ifndef __MC_SERVICE_H_
#define __MC_SERVICE_H_

#include "fixpoint.h"

typedef enum
{
    MC_STATE_DISABLED,
    MC_STATE_ENABLED,
    MC_STATE_RUNNING,
    MC_STATE_RUNNING_CHANGE,
    MC_STATE_END

} Mc_State_t;

typedef enum
{
    MOVE_Y_POSITIVE = 1,
    MOVE_Y_NEGATIVE = 2,
    MOVE_X_POSITIVE = 3,
    MOVE_X_NEGATIVE = 4
}RUNNING_STATE_t;


typedef struct 
{
    fp16_int32_t speed;
    uint8_t dir;
    uint8_t id1;
    uint8_t id2;   

}MtGroup_t;

typedef struct 
{
    fp16_int32_t kp;
    fp16_int32_t ki;
    fp16_int32_t kd;

    fp16_int32_t target;
    fp16_int32_t current;
    fp16_int32_t error;
    fp16_int32_t integral;
    fp16_int32_t derivative;
    fp16_int32_t last_error;

}Pid_handle_t;

void MC_Init(void);
void Mc_StateMachine(void);
void MC_Service_Enable(fp16_int32_t target_angle, uint32_t duration_ms);
void MC_Service_Disable(void);
void MC2XP(void);
void MC2XN(void);
void MC2YP(void);
void MC2YN(void);
#endif
