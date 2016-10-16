#ifndef __TIMER4_H
#define __TIMER4_H

#include "sys.h"

extern int flag_reporter, flag_control,flag_ult_count;

void TIM1_Int_Init(u16 arr,u16 psc);


#endif

