#ifndef __TIMER3_H
#define __TIMER3_H

#include "sys.h"
#define Moto_PWM_1 TIM3->CCR1 		//Moto_PWM_1
#define Moto_PWM_2 TIM4->CCR3    //Moto_PWM_3
#define Moto_PWM_3 TIM4->CCR4 		//Moto_PWM_4
#define Moto_PWM_4 TIM3->CCR2 		//Moto_PWM_2



void TIM3_PWM_Init(u16 arr,u16 psc);

void TIM4_PWM_Init(u16 arr,u16 psc);
#endif