#include "callback.h"
#include "motor.h"
#include "servo.h"
#include "stdio.h"
#include "string.h"
#include "oled.h"
#include "stdbool.h"
/****************************************************
使用TIM1作为系统时基+主程序回调,频率为100Hz，周期10ms
同时使用TIM1作为2个电机的PWM输出
****************************************************/
extern char transmitData[50];
extern char transmitData1[50];
uint8_t command[50];
//每periods触发一次
const float period = 0.01;
DeviceValues values={0};
//使用串口获取目标速度
float current_speed1;
float current_speed2;
float output_speed1;
float output_speed2;
void uart_show(void){
	static int sys_tick=0;
	sys_tick++;
	
	if(sys_tick%1000==0){
		sprintf(transmitData,"State:%d,%d,%d,%d\n实际数值：%.2f,%.2f,%d,%d\n",
			values.motor1_speed,
			values.motor2_speed,
			values.servo1_angle,
			values.servo2_angle,
			current_speed1,
			current_speed2,
			Get_Servo_Angle(1),
			Get_Servo_Angle(2)
		);
		HAL_UART_Transmit_DMA(&UART,(uint8_t*)transmitData,strlen(transmitData));
	}
	
}
void oled_show(void){
	OLED_NewFrame();
	//OLED显示         
	char show[20];
	sprintf(show,"Target1:%d",values.motor1_speed);
	OLED_PrintASCIIString(0,0,show,&afont8x6,OLED_COLOR_NORMAL);
	sprintf(show,"Current1:%.2f",current_speed1);
	OLED_PrintASCIIString(5,8,show,&afont12x6,OLED_COLOR_NORMAL);
	sprintf(show,"Output1:%.2f%%",output_speed1);
	OLED_PrintASCIIString(5,20,show,&afont12x6,OLED_COLOR_NORMAL);
	
	sprintf(show,"Target2:%d",values.motor2_speed);
	OLED_PrintASCIIString(0,32,show,&afont8x6,OLED_COLOR_NORMAL);
	sprintf(show,"Current2:%.2f",current_speed2);
	OLED_PrintASCIIString(5,40,show,&afont12x6,OLED_COLOR_NORMAL);
	sprintf(show,"Output2:%.2f%%",output_speed2);
	OLED_PrintASCIIString(5,52,show,&afont12x6,OLED_COLOR_NORMAL);

	OLED_ShowFrame();
	
}
uint8_t byteToSend = 0xAA;
uint8_t state_button1=0;
uint8_t state_button2=0;
uint8_t state_button3=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim==&htim1){
		//按键检测+0.01s软件消抖
		if(state_button1==0&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_RESET)state_button1=1;
		else if (state_button1==1&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_RESET){
			byteToSend=0xAA;state_button1=0;
		}else state_button1=0;
		
		if(state_button2==0&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==GPIO_PIN_RESET)state_button2=1;
		else if (state_button2==1&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==GPIO_PIN_RESET){
			byteToSend=0xBB;state_button2=0;
		}else state_button2=0;
		
		if(state_button3==0&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==GPIO_PIN_RESET)state_button3=1;
		else if (state_button3==1&&HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==GPIO_PIN_RESET){
			byteToSend=0xCC;state_button3=0;
		}else state_button3=0;
		
		HAL_UART_Transmit_DMA(&UART,&byteToSend,1);
		
		//获取各个设备的目标值
 		values=Get_Values();
		
		//获取当前电机速度
		current_speed1=Get_Motor1_Current_Speed(period)*7;
		current_speed2=Get_Motor2_Current_Speed(period)*7;
		Reset_Motor1_Encoder();
		Reset_Motor2_Encoder();
		
		
//		//获取当前PWM占空比输出
		PID* pid1 = get_speed_pid1();
		PID* pid2 = get_speed_pid2();
		output_speed1=pid1->output/100.0f;
		output_speed2=pid2->output/100.0f;
		
		//PID输出电机速度
//		Set_Motor_PWM(1,values.motor1_speed*100);
//		Set_Motor_PWM(2,values.motor2_speed*100);
		motor_speed_control1(values.motor1_speed,current_speed1);
		motor_speed_control2(values.motor2_speed,current_speed2);


//		motor_speed_control1(999,current_speed1);
//		motor_speed_control2(-999,current_speed2);
		
		Set_Servo_Angle(1,values.servo1_angle);
		Set_Servo_Angle(2,values.servo2_angle);
		
		Command_GetCommand(command,sizeof(command));
		Process_Control_Packet(command);
		
		oled_show();
		//uart_show();

	}
}




