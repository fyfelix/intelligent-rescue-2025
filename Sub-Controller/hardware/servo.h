#ifndef __SERVO_H
#define __SERVO_H

#include "tim.h"

#define SERVO_TIM htim2

void Servo_Init(void);
void Set_Servo_Angle(int servo_id,int angle);
int Get_Servo_Angle(int servo_id);






#endif
