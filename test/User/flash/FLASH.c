
#include "stm32f10x.h"
#include "flash.h"
#include <string.h>
#include <stdio.h>
uint32_t TMAddress=0x08010000;//读指针存储单元，2字节
uint32_t StartAddress=0x08010000;//64k
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

extern uint16_t TM_SEND;

uint16_t PC_R=0;//读指针
uint16_t PC_W=0;//写指针

#define PC_LEN 2518	//定义Flash存储的记录长度，不超过2518条



uint8_t Readflash(uint32_t StartAddr)
{
	uint8_t data;
	uint8_t *p = (uint8_t*)StartAddr;
	data = *p;
//	USART_SendData(USART1, data);
//	while( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
	
	return data;
}

void Writeflash(uint32_t StartAddr,char *src,uint8_t n)
{
	uint8_t i;
	
/* 解锁 FLASH 控制块*/
	FLASH_Unlock();
	/* 清除一些标志位 */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	/* 擦除起始地址为 StartAddress 的 FLASH 页 */	
	FLASHStatus = FLASH_ErasePage(StartAddr);

	if(FLASHStatus == FLASH_COMPLETE)
	{
		for(i=0;i<n/2;i++)
		{
//			USART_SendData(USART1, src[2*i]);
//			while( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
//			USART_SendData(USART1, src[2*i+1]);
//			while( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
			
			FLASHStatus = FLASH_ProgramHalfWord(StartAddr+i*2, (src[2*i+1]<<8)+src[2*i]);
		}
	}	
	/* 锁定 FLASH 控制块*/
	FLASH_Lock();
}

void ClearBlock(uint32_t StartAddr){
	
	/* 擦除起始地址为 StartAddress 的 FLASH 页 */
  FLASHStatus = FLASH_ErasePage(StartAddr);	
	
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
