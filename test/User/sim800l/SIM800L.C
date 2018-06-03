

#include <stddef.h>
#include <stdlib.h>

#include "USART1_init.h"
#include "USART2_init.h"
#include "stdio.h"
#include "SIM800L.h"
#include "flash.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "stm32f10x_iwdg.h"

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

//#define KEY_ON	0
//#define KEY_OFF	1

extern unsigned char hd[];
extern unsigned char id[];
extern unsigned char tp[];
extern unsigned char lat[];
extern unsigned char lng[];
extern unsigned char cbc[];
extern unsigned char end[];

extern unsigned char ip[];
extern unsigned char pt[];

extern u8 Ub[256];

static void Delay_MS(u32 nCount_temp);
static void Ub_Clr();

extern bool flag_cool;	//SIM868冷启动标识：通过PWR键启动为冷启动0，通过RST为热启动1
extern bool flag_gsm;	//GSM Registe OK Flag:1 ok,0 NOT OK

extern bool flag_gps;//GPS OK标识
extern bool flag_rst;	//SIM868 RESET标识

extern unsigned char U1TOU2[50];	//串口1收到的单条有效数据包
extern unsigned char U2RECV[50];	//串口2收到的单条有效数据包
extern unsigned char U1SEND[50];	//串口2发送的有效数据包

extern uint16_t TM_SEND;

extern uint16_t TM_SEND;
extern uint16_t T_SEND;

extern u8 Key_flag;
extern uint8_t Send_Fail_Count;

uint8_t Sim_ini(void)
{

	int i;
	int RecLen=0;
	u8 Ubyte=0;
	
	PWR_ON();	//开机

	//printf("\r\n=======SIM868初始化=======\r\n");
    Delay_MS(5000);
	USART2_DMASS("ATE1\r\n",1000,1000);	
	USART2_DMASS("AT+CSCLK=0\r\n",1000,1000);	
	USART2_DMASS("AT+CFGRI=0\r\n",1000,1000);
	USART2_DMASS("AT+CSQ\r\n",1000,1000);
	USART2_DMASS("AT+CPIN?\r\n",1000,1000);
	USART2_DMASS("AT+CREG?\r\n",1000,1000);
	USART2_DMASS("AT+CREG=1\r\n",1000,1000);
    //USART2_DMASS("AT+CPSI\r\n",1000,1000);
	USART2_DMASS("AT+CGREG?\r\n",1000,1000);
	USART2_DMASS("AT+CGREG=1\r\n",1000,1000);
	USART2_DMASS("AT+CICCID\r\n",2000,1000);	
//等待注册成功	
	//printf("\r\n=======等待网络注册=======\r\n");
	if(Creg_CK()==0){
		//printf("\r\n=======网络注册失败=======\r\n");
		return 0x00;
	}
	//printf("\r\n=======网络注册成功=======\r\n");
	
//TCP/IP
	//printf("\r\n=======TCP/IP初始化=======\r\n");
	USART2_DMASS("AT+CGATT=1\r\n",10000,1000);
	USART2_DMASS("AT+CGACT?\r\n",1000,1000);
	USART2_DMASS("AT+CGSOCKCONT=1,\"IP\",\"3GNET\"\r\n",10000,1000);
	USART2_DMASS("AT+CSOCKSETPN=1\r\n",1000,1000);		//
	USART2_DMASS("AT+CIPMODE=0\r\n",1000,1000);	
	USART2_DMASS("AT+NETOPEN\r\n",1000,1000);	
	USART2_DMASS("AT+IPADDR\r\n",1000,1000);		
	
    Connect_Server();
//连接服务器	
/*	printf("\r\n=====TCP测试，连接服务器=====\r\n");
	
	USART2_DMAS("AT+CIPOPEN=0,\"TCP\",\"");
	USART2_DMAS(ip);
	USART2_DMAS("\",");
	USART2_DMAS(pt);
	USART2_DMAS("\r\n");	
	USART2_DMASS(NULL,1000,1000);
	if(strstr(Ub,"+CIPOPEN: 0,0")==NULL){
		USART2_DMASS(NULL,5000,1000);//发送一个NULL，用于读取USART2数据
		if(strstr(Ub,"+CIPOPEN: 0,0")==NULL){		
			return 0;
		}
	}
*/
//TCP发送数据
/*	USART2_DMASS("AT+CIPSEND=0,5\r\n",1000,1000);
	if(strstr(Ub,">")){
		USART2_SendByte('w');
		USART2_SendByte('o');
		USART2_SendByte('r');
		USART2_SendByte('l');
		USART2_SendByte('O');
		USART2_SendByte(0x1A);
	}
	USART2_DMASS(NULL,1000,1000);
	USART2_DMASS("AT+CIPCLOSE?\r\n",1000,1000);
	if(strstr(Ub,"+CIPCLOSE: 1")){
		printf("\r\n关闭SOCKET，该过程延时较长\r\n");
		USART2_DMASS("AT+CIPCLOSE=0\r\n",3000,20000);
		if(strstr(Ub,"+CIPCLOSE: 0,0")==NULL){
			USART2_DMASS(NULL,1000,15000);
		}
	}
    */
	return 0x01;
}

uint8_t Send_to_Server(void)
{
    uint8_t i;
    USART2_DMASS("AT+CIPSEND=0,11\r\n",1000,1000);
    if(strstr(Ub,">")){
        for(i=0;i<11;i++)
//            USART2_SendByte(Uart1_buffer[i]);
        USART2_SendByte(0x1A);
        Key_flag = 0;
        Send_Fail_Count = 0;
        USART2_DMASS(NULL,10000,1000);
    } 
    else if(strstr(Ub,"+CIPERROR:")){
        Send_Fail_Count++;
        if(Send_Fail_Count == 3)
        {
            Send_Fail_Count = 0;
            while(!Sim_ini()){
                SIM_RST();
            }
        }
        else
        {
            USART2_DMASS("AT+CIPCLOSE?\r\n",1000,1000);
            if(strstr(Ub,"+CIPCLOSE: 1")){
                USART2_DMASS("AT+CIPCLOSE=0\r\n",3000,20000);
                if(strstr(Ub,"+CIPCLOSE: 0,0")==NULL){
                    USART2_DMASS(NULL,1000,15000);
                }
            }
            Connect_Server();
        }
    }
    //Uart1_buffer_Clr();		//
    //USART1_RX_Buffer_Clear();
    LED_BLUE;
}

uint8_t Connect_Server(void)
{
	USART2_DMAS("AT+CIPOPEN=0,\"TCP\",\"");
	USART2_DMAS(ip);
	USART2_DMAS("\",");
	USART2_DMAS(pt);
	USART2_DMAS("\r\n");	
	USART2_DMASS(NULL,1000,1000);
	if(strstr(Ub,"+CIPOPEN: 0,0")==NULL){
		USART2_DMASS(NULL,5000,1000);//发送一个NULL，用于读取USART2数据
		if(strstr(Ub,"+CIPOPEN: 0,0")==NULL){		
			return 0;
		}
	}
    return 1;
}

//判断移动网络注册状态
//成功：1，失败：0
uint8_t Creg_CK()
{
	uint8_t i,j;
	i=0;
	do{
					
		//IWDG_ReloadCounter();	//喂狗

		USART2_DMASS("AT+CREG?\r\n",1000,1000);
		i++;
	}while((strstr(Ub,"+CREG: 1,5")==NULL) && (strstr(Ub,"+CREG: 1,1")==NULL)&& i<30);
	
	j=0;
	do{
		
		//IWDG_ReloadCounter();	//喂狗
		
		USART2_DMASS("AT+CGREG?\r\n",1000,1000);
		j++;
	}while((strstr(Ub,"+CGREG: 1,5")==NULL) && (strstr(Ub,"+CGREG: 1,1")==NULL)&& j<50);
	
	if(i<30 && j<30){
		return 0x01;
	}else{
		return 0x00;
	}
}

//GPS初始化
uint8_t GPSINI()
{

	USART2_DMASS("AT+CGPS=1,1\r\n",1000,1000);
	USART2_DMASS("AT+CGPSINFO\r\n",1000,1000);
	
	return 0x01;
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING; 
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);		

//IGT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
//GPS EN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);
	
}


//uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin,uint8_t Down_state)
//{					   
//		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state )  
//		{	 
//			return 	0;	 
//		}
//		else{
//			return 1;
//		}
//}



//读取STATUS引脚
uint8_t GetKeySTA()
{	
	return Key_Scan(GPIOA,GPIO_Pin_5,0);
}

//读取PWR引脚
uint8_t GetKeyPWR()
{	
	return Key_Scan(GPIOA,GPIO_Pin_0,0);
}


void USART2_DMAS(u8* Data)
{
	while(*Data)
	{
		USART2_SendByte(*Data++);
	}
}

int USART2_DMASS(u8* Data,uint16_t BeTime,uint16_t AfTime)
{

		int i=0;
		int RecLen=0;
		int x,y;
		u8 Ubyte=0;
		uint16_t aftime;
		
		Ub_Clr();		//清空Ub[]
		USART2_RX_Buffer_Clear();
		if(Data!=NULL){
			USART2_DMASendString(Data,strlen(Data));
		}
//等待BeTime	
		do{
			BeTime--;
			Delay_MS(1);
			//IWDG_ReloadCounter();	//喂狗
		}while(!(LookUSART2_GetBuffCount()) && BeTime>0);
//检测串口数据	
		aftime=AfTime;
		do{
			//IWDG_ReloadCounter();	//喂狗
			Delay_MS(1);
			while(LookUSART2_GetBuffCount()){
				USART2_GetByte(&Ubyte);
				Ub[RecLen]=Ubyte;
				RecLen=RecLen+1;			
				aftime=AfTime;
			}
		
			aftime--;
			
		}while(LookUSART2_GetBuffCount() || aftime>0);//注意避免aftime=0时LookUSART2_GetBuffCount()=1的情况出现
					
		USART2_RX_Buffer_Clear();
			
//通过串口1显示				
			//DebugPf("USART2 Receive=%d\r\n",RecLen);
			for(i=0;i<RecLen;i++){
                //DebugPf("%d",Ub[i]);
				//DebugPf("%c",Ub[i]);
			}
			//DebugPf("\r\n");
			
		return RecLen;	
		
}

//等待串口2数据
uint8_t USART2_RD(uint16_t bTime,uint16_t aTime)
{
	
		int i=0;
		int RecLen=0;
		int x,y;
		uint16_t aftime;
		u8 Ubyte=0;
		
//		USART2_RX_Buffer_Clear();

//等待BeTime	
		do{
			bTime--;
			Delay_MS(1);
		}while(!(LookUSART2_GetBuffCount()) && bTime>0);
//检测串口数据	
		aftime=aTime;
		do{
			
			Delay_MS(1);
			while(LookUSART2_GetBuffCount()){
				USART2_GetByte(&Ubyte);
				Ub[RecLen]=Ubyte;
				RecLen=RecLen+1;			
				aftime=aTime;
			}
		
			aftime--;
			
		}while(LookUSART2_GetBuffCount() || aftime>0);	//注意避免aftime=0时LookUSART2_GetBuffCount()=1的情况出现
					
		USART2_RX_Buffer_Clear();
			
//通过串口1显示				
			DebugPf("USART2 Receive=%d\r\n",RecLen);
			for(i=0;i<RecLen;i++){
				DebugPf("%c",Ub[i]);
			}
			DebugPf("\r\n");
			
		return RecLen;		
	
}
/****************************************************************************
* 名	称：void Delay_MS(u32 nCount_temp)
* 功	能：毫秒级
* 入口参数：u32 nCount_temp	延时量
* 出口参数：无
* 说	明：无
****************************************************************************/
void Delay_MS(u32 nCount_temp)
{
	u32 nCount=nCount_temp*8000;
	while(nCount--);
}

//获取GPS数据
uint8_t GetGPS()
{
	uint8_t i;
	
	if(USART2_DMASS("AT+CGNSINF\r\n",100,300)>70){//查询GPRMC数据
		
		if(GPSVLD()!='1')	//判断GPS数据有效性，有效则发送，无效则连续判断
		{
			flag_gps=0;
			
			USART2_DMASS("AT+CGNSINF\r\n",100,300);//查询GPRMC数据
			
			if(strstr(Ub,"+CGNSINF:")==NULL){				//若GPS错误，重启
				DebugPf("GPS ERROR");
				flag_rst=1;
				SIM_RST();
				return 0x00;
			}
		}else{

			flag_gps=1;		

			GPSDATA();			//GPS数据提取到lat[],lng[]
				
			tp[0]='A';	

		}
				
	}
	
	return 0x01;
}

uint8_t SendGPS(){		
		
//TCP SEND
	USART2_DMASS("AT+CIPSEND=46\n",1000,1000);		//
	if(strstr(Ub,">")){
		
		USART2_DMAS(hd);
		USART2_DMAS(id);
		USART2_DMAS(",");
		USART2_DMAS(tp);
		USART2_DMAS(",");
		USART2_DMAS(lat);
		USART2_DMAS(",");
		USART2_DMAS(lng);
		USART2_DMAS(",");
		USART2_DMAS(cbc);
		USART2_DMAS(",");
		USART2_DMAS(end);
		USART2_DMAS("\r");
		USART2_SendByte(0x0D);		//
		USART2_SendByte(0x1A);		//
	
		DebugPf(hd);
		DebugPf(id);
		DebugPf(",");
		DebugPf(tp);
		DebugPf(",");
		DebugPf(lat);
		DebugPf(",");
		DebugPf(lng);
		DebugPf(",");
		DebugPf(cbc);
		DebugPf(",");
		DebugPf(end);
		DebugPf("\r\n");
	}
//	Delay_MS(1000);
	USART2_DMASS(NULL,3000,1000);		//

	
}

uint8_t TCP_GET()
{
	uint8_t i,j;	
	uint8_t U2_LEN[4]="0000";
	uint16_t len=0;
	unsigned char *gps;
	unsigned char s;
	
	for(i=0;i<50;i++){
		U2RECV[i]=0;
	}
	
	if(USART2_DMASS("AT+CIPRXGET=4\r\n",1000,100)){
		
		if(strstr(Ub,"ERROR")){
			
			return 0;
			
		}else if(strstr(Ub,"+CIPRXGET: 4")){
				gps=strstr(Ub,"+CIPRXGET: 4");

				i=0;
				j=0;
				do{
					i++;
					if(gps[i]==','){
						j++;
					}
				}while(j<1 && i<100);

				gps+=i;
				
				i=0;
				j=0;
				do{
					i++;
				}while(gps[i+1]!=0x0d && i<5);

				U2_LEN[0]='0';
				U2_LEN[1]='0';
				U2_LEN[2]='0';
				U2_LEN[3]='0';

				switch(i){
					case 1:
						U2_LEN[3]=gps[1];
					break;
						
					case 2:
						U2_LEN[2]=gps[1];
						U2_LEN[3]=gps[2];
					break;
					
					case 3:
						U2_LEN[1]=gps[1];
						U2_LEN[2]=gps[2];
						U2_LEN[3]=gps[3];
					break;
					
					case 4:
						U2_LEN[0]=gps[1];
						U2_LEN[1]=gps[2];
						U2_LEN[2]=gps[3];
						U2_LEN[3]=gps[4];
					break;
					
					default:
						
					break;
					
				}
				len=((uint16_t)(U2_LEN[0]-0x30)*1000)+((uint16_t)(U2_LEN[1]-0x30)*100)+((uint16_t)(U2_LEN[2]-0x30)*10)+(U2_LEN[3]-0x30);
				//读取U2_LEN长度的数据
				if(U2_LEN[0]!='0' || U2_LEN[1]!='0' || U2_LEN[2]!='0' || U2_LEN[3]!='0'){
					USART2_DMAS("AT+CIPRXGET=2,");
					USART2_SendByte(U2_LEN[0]);
					USART2_SendByte(U2_LEN[1]);
					USART2_SendByte(U2_LEN[2]);
					USART2_SendByte(U2_LEN[3]);
					USART2_DMAS("\r\n");
					USART2_DMASS(NULL,500,100);
					
					//将读取到的数据存入U2_RECV[]	
					if(strstr(Ub,"+CIPRXGET: 2,")){
						gps=strstr(Ub,"+CIPRXGET: 2,");
						i=0;
						do{
							i++;
						}while((gps[i]!='#' || gps[i+1]!=':') && i<127);
						
						gps+=i;
						i=0;
						do{
							U2RECV[i]=gps[i];
							i++;
						}while(gps[i]!='O' && gps[i+1]!='K' && i<127);//while(gps[i]!='#');// || i<100);

					}	
					
//					U2Handle();	//U2数据处理
					
					return 2;
				}			
			
			return 1;	
		}
	}else{
		
		return 0;
		
	}
}


unsigned char GPSVLD()
{
	unsigned char *gps;
	unsigned char i,j;

	gps=strstr(Ub,"+CGNSINF:");

	i=0;
	j=0;
	do{
		if(gps[i]==','){
			j++;
		}
		i++;
	}while(j<1 && i<100);

	gps+=i;
		
	return gps[0];
}

void GPSDATA(void)
{
	unsigned char *gps;
	unsigned char i,j;

	gps=strstr(Ub,"+CGNSINF:");

	i=0;
	j=0;
	do{
		if(gps[i]==','){
			j++;
		}
		i++;
	}while(j<3 && i<100);

	gps+=i;
	
	i=0;
	do{
		lat[i]=gps[i];
		i++;
	}while(gps[i]!=',' && i<9);
//	
//	for(j=0;j<9;j++){
//		USART1_SendByte(lat[j]);
//	}
//		USART1_SendByte(';');
//	
	gps=strstr(Ub,"+CGNSINF:");

	i=0;
	j=0;
	do{
		if(gps[i]==','){
			j++;
		}
		i++;
	}while(j<4 && i<100);

	gps+=i;
		
	i=0;
	do{
		lng[i]=gps[i];
		i++;
	}while(gps[i]!=',' && i<10);

//	
//	for(j=0;j<10;j++){
//		USART1_SendByte(lng[j]);
//	}
//		USART1_SendByte(';');
//		
	
}

void GPSTOU2(void)
{
	unsigned char *gps;
	unsigned char i,j;

	gps=strstr(Ub,"+CGNSINF:");

	i=0;
	j=0;
	do{
		if(gps[i]==','){
			j++;
		}
		i++;
	}while(j<3 && i<100);

	gps+=i;
	
	i=0;
	do{
		lat[i]=gps[i];
		i++;
	}while(gps[i]!=',' && i<9);
	
//	for(j=0;j<9;j++){
//		USART1_SendByte(lat[j]);
//	}
//	USART1_SendByte(';');
	
	gps=strstr(Ub,"+CGNSINF:");

	i=0;
	j=0;
	do{
		if(gps[i]==','){
			j++;
		}
		i++;
	}while(j<4 && i<100);

	gps+=i;
		
	i=0;
	do{
		lng[i]=gps[i];
		i++;
	}while(gps[i]!=',' && i<10);

	
//	for(j=0;j<10;j++){
//		USART1_SendByte(lng[j]);
//	}
//		USART1_SendByte(';');
	

	
	U1TOU2[0]='#';
	U1TOU2[1]=':';
	for(i=0;i<8;i++){
		U1TOU2[i+2]=id[i];
	}
	for(i=0;i<8;i++){
		U1TOU2[i+2]=id[i];
	}
	U1TOU2[10]=',';
	U1TOU2[11]='1';
	U1TOU2[12]=',';
	
	for(i=0;i<9;i++)
	{
		U1TOU2[i+13]=lat[i];
	}
	U1TOU2[i+13]=',';
	
	for(i=0;i<10;i++)
	{
		U1TOU2[i+23]=lng[i];
	}
	U1TOU2[i+23]=',';
	U1TOU2[i+24]='#';
	
//	USART1_SendByte('\"');
//	for(i=0;i<35;i++){
//		USART1_SendByte(U1TOU2[i]);
//	}
//	USART1_SendByte('\"');
					
	
}

void GET_CBC()
{
	if(USART2_DMASS("AT+CBC\r\n",1000,100)){
		CBCDATA();
	}else{
		
	}
}

//获取电量，存入cbc[]
void CBCDATA()
{
	unsigned char *bc;
	unsigned char i;

	bc=strstr(Ub,"+CBC:");

	bc+=8;
	
	i=0;
	do{		
		i++;
	}while(bc[i]!=',' && i<20);
		
	switch (i){
		case 3:
			cbc[0]=bc[0];
			cbc[1]=bc[1];
			cbc[2]=bc[2];	
			break;
		
		case 2:
			cbc[0]=0x30;
			cbc[1]=bc[0];
			cbc[2]=bc[1];	
			break;
		
		case 1:
			cbc[0]=0x30;
			cbc[1]=0x30;
			cbc[2]=bc[0];	
			break;
		
		default:
			break;
	}
	
//	DebugPf(cbc);

}

void PWR_ON()
{
	Delay_MS(500);
	while(GetKeySTA())
	{
		//IWDG_ReloadCounter();	//喂狗
	
		//printf("\r\nPOWER ON...\r\n");
		IGT_Lo;
		Delay_MS(500);
		IGT_Hi;
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		
	}	

	//printf("\r\nPOWERED ON\r\n");
}
void PWR_OFF()
{
	
	while(!GetKeySTA())
	{

		//IWDG_ReloadCounter();	//喂狗
		//printf("\r\nPOWER OFF...\r\n");
		
		IGT_Lo;
		Delay_MS(3000);
		IGT_Hi;
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗
		Delay_MS(5000);
		//IWDG_ReloadCounter();	//喂狗

	}
	//printf("\r\nPOWERED OFF\r\n");
}
void SIM_RST()
{
	IWDG_ReloadCounter();	//喂狗
	
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

void Ub_Clr()
{
	uint16_t i;
	for(i=0;i<256;i++)
	{
		Ub[i]=0x00;
	}
	
}
