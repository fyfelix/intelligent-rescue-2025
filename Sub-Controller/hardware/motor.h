#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

/****************************************
使用TIM1的通道1和通道2分别控制电机1和电机2
使用TIM3作为电机1的编码器接口
使用TIM4作为电机2的编码器接口
****************************************/
#define MOTOR_TIM htim1 
//电机1参数
#define AIN1_GPIO GPIOA
#define AIN1_PIN GPIO_PIN_12

#define AIN2_GPIO GPIOA
#define AIN2_PIN GPIO_PIN_11

#define MOTOR1_CHANNEL TIM_CHANNEL_1 //PA8 电机1PWM接口
//编码器接口
#define MOTOR1_ENCODER_TIM htim3
#define MOTOR1_ENCODER_CHANNEL1 TIM_CHANNEL_2 //PA6
#define MOTOR1_ENCODER_CHANNEL2 TIM_CHANNEL_1 //PA7

//电机2参数
#define BIN1_GPIO GPIOB
#define BIN1_PIN GPIO_PIN_14

#define BIN2_GPIO GPIOB
#define BIN2_PIN GPIO_PIN_15

#define MOTOR2_CHANNEL TIM_CHANNEL_2 //PA9 电机2PWM接口
//编码器接口
#define MOTOR2_ENCODER_TIM htim4
#define MOTOR2_ENCODER_CHANNEL1 TIM_CHANNEL_1 //PB6
#define MOTOR2_ENCODER_CHANNEL2 TIM_CHANNEL_2 //PB7


// 电机最大输入的PWM值
#define MOTOR_PWM_MAX 9999

void Motor_Init(void);
void Set_Motor1_PWM(int pwm);
void Set_Motor2_PWM(int pwm);
float Get_Motor1_Current_Speed(float period);
float Get_Motor2_Current_Speed(float period);
void Reset_Motor1_Encoder(void);
void Reset_Motor2_Encoder(void);






//PID部分===========================================================
typedef struct
{
  float kp, ki, kd; 						      // 三个静态系数
  float change_p, change_i, change_d;	          // 三个动态参数
  float error, last_error; 						  // 误差、之前误差
  float max_change_i; 							  // 积分限幅
  float output, max_output; 				      // 输出、输出限幅
  int target;                                     // 目标
}PID;


void pid_init(PID* pid, float p, float i, float d, float maxI, float maxOut, int target);
void pid_change_zero(PID* pid);
float pid_calc(PID *pid, float reference, float feedback);

void speed_pid_init(void);
PID* get_speed_pid1(void);
PID* get_speed_pid2(void);
int get_speed_pid_target1(void);
int get_speed_pid_target2(void);

PID motor_speed_control1(int target_speed,float current_speed);
PID motor_speed_control2(int target_speed,float current_speed);

#endif
