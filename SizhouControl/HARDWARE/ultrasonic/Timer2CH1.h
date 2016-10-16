#ifndef __TIMER2CH1_H
#define __TIMER2CH1_H


#include "sys.h"

extern u8 flag_catch;
extern u16 flag_val;

void TIM2_Catch_Init(u16 arr,u16 psc);


#endif
