#include "servo.h"

int Servo_Angle[4]={0};

void Servo_Init(void){
	__HAL_TIM_SET_PRESCALER(&SERVO_TIM,720-1);
	__HAL_TIM_SET_AUTORELOAD(&SERVO_TIM,2000-1);
//	HAL_TIM_PWM_Start(&SERVO_TIM,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

//angle to duty
int __a2d(int angle){
	int duty=(200.0f*angle)/180.0f + 50;
	return duty;
}

uint16_t SERVO_CHANNELS[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

void Set_Servo_Angle(int servo_id,int angle){
	int duty=__a2d(angle);
	__HAL_TIM_SET_COMPARE(&SERVO_TIM,SERVO_CHANNELS[servo_id-1],duty);
	Servo_Angle[servo_id-1]=angle;
}

int Get_Servo_Angle(int servo_id){
	return Servo_Angle[servo_id-1];
}



