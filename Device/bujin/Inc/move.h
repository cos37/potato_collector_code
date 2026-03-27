#ifndef __MOVE_H_
#define __MOVE_H_


#include <stdint.h>

typedef enum
{
    POSITIVE,
    NEGATIVE,
}Dir_t;

typedef struct 
{
    void (*init)(void);
    void (*move_x)(uint16_t speed,Dir_t dir);
    void (*move_y)(uint16_t speed,Dir_t dir);
    void (*stop)(void);
}MoveHandle_t;

void Move_Init(MoveHandle_t *hmov);

#endif 
