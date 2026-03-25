#ifndef __UART_H
#define __UART_H

#include "main.h"
#include "usart.h"

#pragma pack(1)
typedef struct {
    uint8_t header;     // 包头 0x55
    uint8_t length_byte;// 数据包长度 应该为13
    uint8_t speed1_high;   // 电机1的速度 0~200
	uint8_t speed1_low;   // 电机1的速度 0~200
	uint8_t speed2_high;   // 电机2的速度 0~200
	uint8_t speed2_low;   // 电机2的速度 0~200
	uint8_t angle1_high;  // 舵机1的数据高字节 0~360
	uint8_t angle1_low;  // 舵机1的数据低字节 0~360
    uint8_t angle2_high;  // 舵机2的数据高字节 0~360
	uint8_t angle2_low;  // 舵机2的数据低字节 0~360
    uint16_t crc16;     // CRC16校验
    uint8_t footer;     // 包尾 0xAA
} DeviceControlPacket;
#pragma pack()

typedef struct {
    int motor1_speed;
    int motor2_speed;
    int servo1_angle;
    int servo2_angle;
} DeviceValues;

//TX->PB10
//RX->PB11
#define UART huart3
void Process_Control_Packet(uint8_t *data);
void UART_Init(void);
DeviceValues Get_Values(void);

uint8_t Command_Write(uint8_t *data, uint8_t length);
uint8_t Command_GetCommand(uint8_t *command,uint8_t cmd_length);

#endif
