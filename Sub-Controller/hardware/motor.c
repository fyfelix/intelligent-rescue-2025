#include "motor.h"

void Motor_Init(void){
	//初始化PSC和ARR
	__HAL_TIM_SET_PRESCALER(&MOTOR_TIM,72-1);
	__HAL_TIM_SET_AUTORELOAD(&MOTOR_TIM,MOTOR_PWM_MAX);
	HAL_TIM_PWM_Start(&MOTOR_TIM,MOTOR1_CHANNEL);
	HAL_TIM_PWM_Start(&MOTOR_TIM,MOTOR2_CHANNEL);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR1_CHANNEL,0);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR2_CHANNEL,0);
	
	//开启电机编码器接口
	HAL_TIM_Encoder_Start(&MOTOR1_ENCODER_TIM,MOTOR1_ENCODER_CHANNEL1);
	HAL_TIM_Encoder_Start(&MOTOR1_ENCODER_TIM,MOTOR1_ENCODER_CHANNEL2);
	
	HAL_TIM_Encoder_Start(&MOTOR2_ENCODER_TIM,MOTOR2_ENCODER_CHANNEL1);
	HAL_TIM_Encoder_Start(&MOTOR2_ENCODER_TIM,MOTOR2_ENCODER_CHANNEL2);
	
	//默认不使能电机
	HAL_GPIO_WritePin(AIN1_GPIO,AIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO,AIN2_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN1_GPIO,BIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO,BIN2_PIN,GPIO_PIN_RESET);
	
}

void Motor1_Forward(void){
	HAL_GPIO_WritePin(AIN1_GPIO,AIN1_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO,AIN2_PIN,GPIO_PIN_RESET);
}
void Motor1_Backward(void){
	HAL_GPIO_WritePin(AIN1_GPIO,AIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO,AIN2_PIN,GPIO_PIN_SET);
}

void Motor2_Forward(void){
	HAL_GPIO_WritePin(BIN1_GPIO,BIN1_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO,BIN2_PIN,GPIO_PIN_RESET);
}
void Motor2_Backward(void){
	HAL_GPIO_WritePin(BIN1_GPIO,BIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO,BIN2_PIN,GPIO_PIN_SET);
}

/********************************
电机速度控制
速度范围：10000
********************************/
void Set_Motor1_PWM(int pwm){
	if(pwm>=0){
		Motor1_Forward();
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR1_CHANNEL,pwm);
	}else if(pwm<0){
		Motor1_Backward();
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR1_CHANNEL,-pwm);
	}

}

void Set_Motor2_PWM(int pwm){
	if(pwm>=0){
		Motor2_Forward();
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR2_CHANNEL,pwm);
	}else if(pwm<0){
		Motor2_Backward();
		__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR2_CHANNEL,-pwm);
	}

}


//获取当前电机速度
#define PULSE (4*45*13) //(EncoderModeT1andT2)*减速比*编码器线圈数
float Get_Motor1_Current_Speed(float period){
	float speed=0;
	int count=__HAL_TIM_GET_COUNTER(&MOTOR1_ENCODER_TIM);
	if (count>32767)count=count-65535;
	speed = (float)count*100/period/PULSE; //当前电机的每秒转速
	return -speed;//100的范围改成10000的范围
}
float Get_Motor2_Current_Speed(float period){
	float speed=0;
	int count=__HAL_TIM_GET_COUNTER(&MOTOR2_ENCODER_TIM);
	if (count>32767)count=count-65535;
	speed = (float)count*100/period/PULSE; //当前电机的每秒转速
	return -speed;//100的范围改成10000的范围
}

void Reset_Motor1_Encoder(){
	__HAL_TIM_SET_COUNTER(&MOTOR1_ENCODER_TIM,0);
}
void Reset_Motor2_Encoder(){
	__HAL_TIM_SET_COUNTER(&MOTOR2_ENCODER_TIM,0);
}







/*PID部分====================================================================================*/

/************************************************
功能：初始化PID各参数
参数：pid = 对应pid的结构体地址
      p = pid的静态kp值
      i = pid的静态ki值
      d = pid的静态kd值
      maxI = pid计算后的I最大值，即最大误差累加值
      maxOut = PID最大输出值
      target = 目标值
************************************************/
void pid_init(PID* pid, float p, float i, float d, float maxI, float maxOut, int target)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->max_change_i = maxI;
    pid->max_output = maxOut;
    pid->target = target;
}

/******************************************************************
 * 函 数 说 明：PID变化累计的参数清零
 * 函 数 形 参：pid=对应的PID地址
******************************************************************/
void pid_change_zero(PID* pid)
{
    pid->change_p = 0;
    pid->change_i = 0;
    pid->change_d = 0;
}

/****************************************************
功能：单级PID计算
参数：pid = pid的参数输入
     target = 目标值
     current = 当前值
返回：PID计算后的结果
****************************************************/
float pid_calc(PID *pid, float target, float current)
{
    //用上一次的误差值更新 之前误差last_error
    pid->last_error = pid->error;
    //获取新的误差 = 目标值 - 当前值
    pid->error = (target - current)/100;

    //计算比例P = 目标值与实际值之间的误差e
    float pout = pid->error;
    //计算积分I = 误差e的累加
    pid->change_i += pid->error;
    //计算微分D = 当前误差e - 之前的误差last_e
    float dout = pid->error - pid->last_error;

    //积分I 限制不能超过正负最大值
    if(pid->change_i > pid->max_change_i)
    {
      pid->change_i = pid->max_change_i;
    }
    else if(pid->change_i < -pid->max_change_i)
    {
      pid->change_i = -pid->max_change_i;
    }

    //计算输出PID_OUT = （Kp x P）+ （Ki x I）+（Kd x D）
    pid->output = (pid->kp * pout) + (pid->ki * pid->change_i) + (pid->kd * dout);

    //输出 限制不能超过正负最大值
    if(pid->output > pid->max_output) pid->output = pid->max_output;
    else if(pid->output < -pid->max_output) pid->output = -pid->max_output;

    //返回PID计算的结果
    return pid->output;
}

PID speed_pid1;
PID speed_pid2;

//定速PID初始化
void speed_pid_init(void)
{
    pid_init(&speed_pid1, 35, 6, 10, MOTOR_PWM_MAX, MOTOR_PWM_MAX,0);
	pid_init(&speed_pid2, 35, 6, 10, MOTOR_PWM_MAX, MOTOR_PWM_MAX,0);
}

//获取定速PID全局变量的地址
PID* get_speed_pid1(void)
{
	return &speed_pid1;
}
PID* get_speed_pid2(void)
{
	return &speed_pid2;
}

//获取定速PID的目标值
int get_speed_pid_target1(void)
{
	return speed_pid1.target;
}
int get_speed_pid_target2(void)
{
	return speed_pid2.target;
}

/************************************************
功能：PID电机定速控制器
参数：target_speed = 目标值
返回：对应PID的地址
************************************************/
PID motor_speed_control1(int target_speed,float current_speed)
{
	if (target_speed==0){
		HAL_GPIO_WritePin(AIN1_GPIO,AIN1_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO,AIN2_PIN,GPIO_PIN_SET);
		pid_change_zero(&speed_pid1);
		speed_pid1.output = 0;
		return speed_pid1;
	}
	int PWM;
	PWM = pid_calc(&speed_pid1, target_speed, current_speed);
	Set_Motor1_PWM(PWM);
	return speed_pid1;
}
PID motor_speed_control2(int target_speed,float current_speed)
{
	if (target_speed==0){
		HAL_GPIO_WritePin(BIN1_GPIO,BIN1_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO,BIN2_PIN,GPIO_PIN_SET);
		pid_change_zero(&speed_pid2);
		speed_pid2.output = 0;
		return speed_pid1;
	}
	int PWM;
	PWM = pid_calc(&speed_pid2, target_speed, current_speed);
	Set_Motor2_PWM(PWM);
	return speed_pid2;
}
