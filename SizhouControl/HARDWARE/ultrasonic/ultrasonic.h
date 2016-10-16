#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "sys.h"
#include "delay.h"
#include "Timer2CH1.h"
//#include "math.h"


#define ULT PAout(1)				// PA1	

extern int ult_distance;
//extern u32 ult_temp;

void ultrasonic_init(void);
void ultrasonic_work(void);
void distance_count(u32 ult_temp);

#endif









