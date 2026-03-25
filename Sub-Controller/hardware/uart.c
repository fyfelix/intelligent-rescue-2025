#include "uart.h"
#include "callback.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
uint8_t received_data[50];
extern uint8_t command[50];
//uint16_t received_length = 0;

extern DMA_HandleTypeDef hdma_usart3_rx;
void UART_Init(void){
//	HAL_UART_Receive_DMA(&UART, received_data, sizeof(received_data)); 
	HAL_UARTEx_ReceiveToIdle_DMA(&UART, received_data, sizeof(received_data)); 
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}
//用于发送的数据
char transmitData[100]={0};
char transmitData1[50]={0};

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &UART)
    {
		Command_Write(received_data, Size);
//		sprintf(transmitData1, "Received %d bytes\n", Size);
//		HAL_UART_Transmit(&UART,(uint8_t*)transmitData1, strlen(transmitData1),50);
		
		// 重新启动接收
//		HAL_UART_Receive_DMA(&UART, received_data, sizeof(received_data)); 
		HAL_UARTEx_ReceiveToIdle_DMA(&UART, received_data, sizeof(received_data)); 
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
    }
}

/**
 * @brief 计算数据的CRC16校验值（Modbus标准）
 * @param data 数据指针
 * @param length 数据长度（字节数）
 * @return 计算得到的CRC16值
 */
uint16_t Calculate_CRC16(const uint8_t *data, uint8_t start_byte, uint8_t end_byte) {
    /* 计算指定字节范围的CRC16 */
    uint16_t crc = 0xFFFF;

    // 计算指定字节范围
    for(uint8_t i = start_byte; i <= end_byte; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    
    return crc;
}

// 全局变量定义
static int motor1_target_speed = 0;   // 电机1目标速度(-100~100)
static int motor2_target_speed = 0;   // 电机2目标速度(-100~100)
static int servo1_target_angle = 0;  // 舵机1目标角度(0-360)
static int servo2_target_angle = 0;  // 舵机2目标角度(0-360)

// 解包指令
bool Debug=false;//调试模式
void Process_Control_Packet(uint8_t *data){
	if(Debug){
		sprintf(transmitData1,"Processing packet\n");
		HAL_UART_Transmit(&UART,(uint8_t*)transmitData1,strlen(transmitData1),50);
	}
    // 检查最小长度（增加了length_byte后总长度+1）
    if(sizeof(DeviceControlPacket)!=13) {
		if(Debug){
			sprintf(transmitData1,"length error\n");
			HAL_UART_Transmit(&UART,(uint8_t*)transmitData1,strlen(transmitData1),50);
		}
        return;
    }
    
    DeviceControlPacket *packet = (DeviceControlPacket *)data;
    
    // 校验包头包尾
    if(packet->header != 0x55 || packet->footer != 0xAA) {
		if(Debug){
			sprintf(transmitData1,"head byte or tail byte error\n");
			HAL_UART_Transmit(&UART,(uint8_t*)transmitData1,strlen(transmitData1),50);
		}
        return;
    }
    // 校验长度位是否匹配
    if(packet->length_byte != sizeof(DeviceControlPacket)) {
		if(Debug){
			sprintf(transmitData1,"length byte error\n");
			HAL_UART_Transmit(&UART,(uint8_t*)transmitData1,strlen(transmitData1),50);
		}
        return;
    }

    // 校验CRC16
	uint16_t calculated_crc = Calculate_CRC16(&data[0],2,9);
    if(calculated_crc != packet->crc16) {
		if(Debug){
			sprintf(transmitData1,"crc16 error,The correct one should be %d\n",(int)calculated_crc);
			HAL_UART_Transmit(&UART,(uint8_t*)transmitData1,strlen(transmitData1),50);
		}
        return;
    }
    // 合并数据字节
    uint16_t angle1 = ((uint16_t)packet->angle1_high << 8) | packet->angle1_low;
	uint16_t angle2 = ((uint16_t)packet->angle2_high << 8) | packet->angle2_low;
    
	motor1_target_speed=((uint16_t)packet->speed1_high << 8) | packet->speed1_low;motor1_target_speed-=10000;
	motor2_target_speed=((uint16_t)packet->speed2_high << 8) | packet->speed2_low;motor2_target_speed-=10000;
	
	servo1_target_angle = angle1;
	servo2_target_angle = angle2;
}

/**
 * @brief 获取设备当前目标值
 * @return 包含所有值的结构体（按值返回，安全）
 */
DeviceValues Get_Values(void)
{
    DeviceValues values;
    values.motor1_speed = motor1_target_speed;
    values.motor2_speed = motor2_target_speed;
    values.servo1_angle = servo1_target_angle;
    values.servo2_angle = servo2_target_angle;
    return values;
}


/********************************************
 * 循环缓冲区
********************************************/

// 指令的最小长度
#define COMMAND_MIN_LENGTH 4 

// 循环缓冲区大小
#define BUFFER_SIZE 128
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint8_t readIndex = 0;
// 循环缓冲区写索引
uint8_t writeIndex = 0;

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/**
* @brief 计算未处理的数据长度
* @return 未处理的数据长度
* @retval 0 缓冲区为空
* @retval 1~BUFFER_SIZE-1 未处理的数据长度
* @retval BUFFER_SIZE 缓冲区已满
*/
//uint8_t Command_GetLength() {
//  // 读索引等于写索引时，缓冲区为空
//  if (readIndex == writeIndex) {
//    return 0;
//  }
//  // 如果缓冲区已满,返回BUFFER_SIZE
//  if (writeIndex + 1 == readIndex || (writeIndex == BUFFER_SIZE - 1 && readIndex == 0)) {
//    return BUFFER_SIZE;
//  }
//  // 如果缓冲区未满,返回未处理的数据长度
//  if (readIndex <= writeIndex) {
//    return writeIndex - readIndex;
//  } else {
//    return BUFFER_SIZE - readIndex + writeIndex;
//  }
//}

uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint8_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();
}

/**
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/
uint8_t Command_Write(uint8_t *data, uint8_t length) {
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (writeIndex + length < BUFFER_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        uint8_t firstLength = BUFFER_SIZE - writeIndex;
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = length - firstLength;
    }
    return length;
}

/**
* @brief 尝试获取一条指令
* @param command 指令存放指针
* @return 获取的指令长度
* @retval 0 没有获取到指令
*/
uint8_t length = 13;
uint8_t Command_GetCommand(uint8_t *command,uint8_t cmd_length) {
    // 寻找完整指令
    while (1) {
        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
        return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(readIndex) != 0x55) {
			Command_AddReadIndex(1);
			continue;
        }
        // 如果缓冲区长度小于指令长度 则不可能有完整的指令
        if (Command_GetLength() < length) {
			return 0;
        }
		
		// 如果不是包尾 则跳过 重新开始寻找
		if (Command_Read(readIndex+12) != 0xAA) {
			Command_AddReadIndex(1);
			continue;
		}
//        // 如果校验和不正确 则跳过 重新开始寻找
//        uint8_t sum = 0;
//        for (uint8_t i = 0; i < length - 1; i++) {
//        	  sum += Command_Read(readIndex + i);
//        }
//        if (sum != Command_Read(readIndex + length - 1)) {
//        	  Command_AddReadIndex(1);
//        continue;
//        }
        // 如果找到完整指令 则将指令写入command 返回指令长度
		memset(command,0,cmd_length);
        for (uint8_t i = 0; i < length; i++) {
			command[i] = Command_Read(readIndex + i);
        }
        Command_AddReadIndex(length);
        return length;
    }
}
