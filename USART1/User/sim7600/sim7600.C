
#include <stddef.h>
#include <stdlib.h>

#include "bsp_usart2.h"
#include "sim7600.h"
#include "stdio.h"
#include "bsp_common.h"
#include "bsp_key.h"  

extern u8 Usart2_temp_buffer[256];
extern unsigned char ip[];
extern unsigned char pt[];

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

uint8_t Sim_ini(void)
{

	int i;
	int RecLen=0;
	u8 Ubyte=0;
	
	PWR_ON();	//开机
	USART2_DMASS("ATE1\r\n",1000,1000);		
	USART2_DMASS("AT+CSCLK=0\r\n",1000,1000);	
	USART2_DMASS("AT+CFGRI=0\r\n",1000,1000);
	USART2_DMASS("AT+CSQ\r\n",1000,1000);
	USART2_DMASS("AT+CPIN?\r\n",1000,1000);
	USART2_DMASS("AT+CREG?\r\n",1000,1000);
	USART2_DMASS("AT+CREG=1\r\n",1000,1000);
	USART2_DMASS("AT+CGREG?\r\n",1000,1000);
	USART2_DMASS("AT+CGREG=1\r\n",1000,1000);
	USART2_DMASS("AT+CICCID\r\n",2000,1000);	
//等待注册成功	
	printf("\r\n=======等待网络注册=======\r\n");
	if(Creg_CK()==0){
		printf("\r\n=======网络注册失败=======\r\n");
		return 0x00;
	}
	printf("\r\n=======网络注册成功=======\r\n");
//TCP/IP
	printf("\r\n=======TCP/IP初始化=======\r\n");
	USART2_DMASS("AT+CGATT=1\r\n",10000,1000);
	USART2_DMASS("AT+CGACT?\r\n",1000,1000);
	USART2_DMASS("AT+CGSOCKCONT=1,\"IP\",\"CMNET\"\r\n",10000,1000);
	USART2_DMASS("AT+CSOCKSETPN=1\r\n",1000,1000);		//
	USART2_DMASS("AT+CIPMODE=0\r\n",1000,1000);	
	USART2_DMASS("AT+NETOPEN\r\n",1000,1000);	
	USART2_DMASS("AT+IPADDR\r\n",1000,1000);		
	
//连接服务器	

	USART2_DMAS("AT+CIPOPEN=0,\"TCP\",\"");
	USART2_DMAS(ip);
	USART2_DMAS("\",");
	USART2_DMAS(pt);
	USART2_DMAS("\r\n");	
	USART2_DMASS(NULL,1000,1000);
	if(strstr(Usart2_temp_buffer,"+CIPOPEN: 0,0")==NULL){
		USART2_DMASS(NULL,5000,1000);//发送一个NULL，用于读取USART2数据
		if(strstr(Usart2_temp_buffer,"+CIPOPEN: 0,0")==NULL){		
			return 0;
		}
	}
//TCP发送数据
	USART2_DMASS("AT+CIPSEND=0,5\r\n",1000,1000);
	if(strstr(Usart2_temp_buffer,">")){
		USART2_SendByte('H');
		USART2_SendByte('E');
		USART2_SendByte('L');
		USART2_SendByte('L');
		USART2_SendByte('O');
		USART2_SendByte(0x1A);
	}
	USART2_DMASS(NULL,1000,1000);
	USART2_DMASS("AT+CIPCLOSE?\r\n",1000,1000);
	if(strstr(Usart2_temp_buffer,"+CIPCLOSE: 1")){
		USART2_DMASS("AT+CIPCLOSE=0\r\n",3000,20000);
		if(strstr(Usart2_temp_buffer,"+CIPCLOSE: 0,0")==NULL){
			USART2_DMASS(NULL,1000,15000);
		}
	}
	return 0x01;
}
//判断移动网络注册状态
//成功：1，失败：0
uint8_t Creg_CK()
{
	uint8_t i,j;
	i=0;
	do{
		USART2_DMASS("AT+CREG?\r\n",1000,1000);
		i++;
	}while((strstr(Usart2_temp_buffer,"+CREG: 1,5")==NULL) && (strstr(Usart2_temp_buffer,"+CREG: 1,1")==NULL)&& i<30);
	
	j=0;
	do{		
		USART2_DMASS("AT+CGREG?\r\n",1000,1000);
		j++;
	}while((strstr(Usart2_temp_buffer,"+CGREG: 1,5")==NULL) && (strstr(Usart2_temp_buffer,"+CGREG: 1,1")==NULL)&& j<50);
	
	if(i<30 && j<30){
		return 0x01;
	}else{
		return 0x00;
	}
}

/**
  * @brief  ???????I/O?
  * @param  ?
  * @retval ?
  */
void SIM_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);
//STATUS
    SIM_GPIO_STATUS_APBxClkCmd(SIM_GPIO_STATUS_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = SIM_GPIO_STATUS_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING; 	
	GPIO_Init(SIM_GPIO_STATUS_PORT, &GPIO_InitStructure);		

//IGT
    SIM_GPIO_IGT_APBxClkCmd(SIM_GPIO_IGT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = SIM_GPIO_IGT_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_Init(SIM_GPIO_IGT_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(SIM_GPIO_IGT_PORT,SIM_GPIO_IGT_PIN);	
}

//??STATUS??
uint8_t GetSimSTA()
{	
	return Key_Scan(SIM_GPIO_STATUS_PORT,SIM_GPIO_STATUS_PIN);
}

void PWR_ON()
{
    uint8_t i;
	while(GetSimSTA())
	{	
		IGT_Lo;
		Delay_MS(500);       
		IGT_Hi;
		Delay_MS(5000);
		Delay_MS(5000);
		Delay_MS(5000);
		Delay_MS(5000);
		Delay_MS(5000);		
	}	
}
void PWR_OFF()
{
	
	while(!GetSimSTA())
	{		
		IGT_Lo;
		Delay_MS(3000);
		IGT_Hi;
		Delay_MS(5000);
		Delay_MS(5000);
		Delay_MS(5000);
	}
}
void SIM_RST()
{	
	PWR_OFF();
	Delay_MS(3000);
	PWR_ON();	
}

uint8_t PWR_DOWN()
{
		
//	USART2_DMASS("AT+CPOWD=1\r\n",2500,2000);
//	if(strstr(Ub,"NORMAL POWER DOWN"))
//	{
		return 0x01;
//	}
//	
//	return 0x00;
}
