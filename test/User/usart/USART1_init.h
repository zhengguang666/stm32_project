#ifndef _USART1_INIT_H_	
#define _USART1_INIT_H_

#ifdef __cplusplus			//定义对CPP进行C处理
extern "C" {
#endif

#include "stm32f10x.h"
#include "static_init.h"	//串口结构体
    
#define USART1_BAUD_RATE 1200

void USART1_Config(void);		  				//串口配置
void USART1_RX_Buffer_Clear(void); 				//清空接收缓冲区

void USART1_SendByte(u8 Data);					//单字符数据发送
void USART1_SendString(u8* Data,u32 Len);		//多字符发送
void USART1_DMASendString(u8* Data,u32 Len);	//DMA多字符发送
u8 USART1_GetByte(u8* Data);					//单字符接收
u8 USART1_GetByte_WaitTime(u8* Data,u32 TimeLater);	//串口单字符接收等待时长
u32 USART1_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater);	//等待时长之后获取多字节数据
u8 USART1_GetByte_WaitLong(void);				//串口单字符接收一直等待
void USART1_GPIO_Init(void);						//串口引脚初始化

u32 LookUSART1_GetBuffMax(void);				//查询接收缓冲区大小
u32 LookUSART1_GetBuffCount(void);				//查询缓冲区中接收到的数据个数


#ifdef __cplusplus		   //定义对CPP进行C处理 //结束部分
}
#endif

#endif
