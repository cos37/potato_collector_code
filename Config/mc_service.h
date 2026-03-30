#ifndef __MC_SERVICE_H_
#define __MC_SERVICE_H_

#include "fixpoint.h"

typedef enum
{
    MC_STATE_DISABLED,
    MC_STATE_ENABLED,
    MC_STATE_RUNNING,
    MC_STATE_END
} Mc_State_t;

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
void MC_Service_Enable(fp16_int32_t target_angle, uint16_t duration_ms);
#endif
