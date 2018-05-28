
#include "bsp_iso_test.h"
#include "bsp_led.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "TIMER.h"
#include "FLASH.h"
#include "SIM800L.h"
#include <string.h>
#include "USART1_init.h"
#include "USART2_init.h"
#include "stm32f10x_iwdg.h"

static void delay(uint16_t t);
void GetId();
void Uart1_buffer_Clr(void);
int Get_Weight(uint16_t AfTime);

void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);

uint8_t	uart3_buff[255];
uint8_t CentL,CentR;//左右眼中心值存储单元

unsigned char PK_Cnt=0;//开关机按钮按下后，计时次数

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

bool FLAG_BLE_CTS0=0;//CTS是否被拉低，拉低为1，否则为0
bool FLAG_BLE_CTS1=0;//CTS是否被拉高，拉高为1，否则为0
bool FLAG_LONG=0;		//分包长数据接收标识，准备接收为1，无为0

bool flag_send;//TCP发送标识
bool flag_get;//TCP获取标识
bool flag_cbc;//定时电量检测标识
bool flag_gps;//GPS OK标识
bool flag_svs=0;//服务器数据更新标识
bool flag_pos=0;//定时获取GPS位置标识
	
unsigned char hd[]="TE005:";
unsigned char id[]="00000000";
unsigned char tp[]="0";
unsigned char lat[]="0000.0000";
unsigned char lng[]="00000.0000";
unsigned char cbc[]="000";
unsigned char end[]="END";

unsigned char U1RECV[50]={0};	//串口1收到的单条有效数据包
unsigned char U1SEND[50]={0};	//串口1发送的有效数据包
unsigned char U1TOU2[50]={0};	//串口1数据整理后发送到串口2（增加ID）
unsigned char U2TOU1[50]={0};	//串口2数据整理后发送到串口1（去掉ID）
unsigned char U2RECV[50]={0};	//串口2收到的单条有效数据包
unsigned char U2SEND[50]={0};	//串口2发送的有效数据包


unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";
//unsigned char ip[]="172.21.186.10";
//unsigned char pt[]="10001";

uint8_t sts;	//SIM868 STATUS PIN

bool flag_rst=0;	//SIM868 RESET标识
bool flag_cool=0;	//SIM868冷启动标识：通过PWR键启动为冷启动0，通过RST为热启动1
bool flag_gsm=0;	//GSM Registe OK Flag:1 ok,0 NOT OK


u8 Ub[256];
u8 Uart1_buffer[256];
u8 temp_buffer[256];
u8 Key_flag=0;
#define WEIGHT_INFO_LEN 12

/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{ 
	char *s;
	uint8_t i,j;
	uint8_t LEN[2];
    int Uart1_RecLen=0;
    u32 temp=0,first=0;
    uint8_t Send_Fail_Count=0;
	GPIO_InitTypeDef GPIO_InitStructure;		//串口引脚结构
	
//	/* 设置系统时钟 */
	RCC_Configuration();
//	/* 设置 NVIC */
	NVIC_Configuration();
	
	LED_GPIO_Config();
    Key_GPIO_Config();
	SIM_GPIO_Config();
	
	ISO_Init();//初始化串口1/2
	//printf("\r\n========SIM7600CE测试程序========\r\n");
	//printf("*本程序提供了AT指令操作SIM7600CE模块的基本思路\r\n");
	//printf("*实现模块初始化、TCP/IP，HTTPS、GPS、WLAN等功能\r\n");
	//printf("*更多操作，请查阅AT指令手册\r\n");
	//printf("*如有疑问，可加入我们的技术交流QQ群：586340506\r\n");
	//printf("*接线方式：\r\n");
	//printf("*PA2--RXD\r\n");
	//printf("*PA3--TXD\r\n");
	//printf("*PB4--IGT\r\n");
	//printf("*PB3--STATUS\r\n");	
	//printf("===============================\r\n");
	
	//GetId();
    
    //初始化SIM7600CE			
	while(!Sim_ini()){
				SIM_RST();
	}
	Uart1_buffer_Clr();		//
	USART1_RX_Buffer_Clear();
	while(1){
        //LED_BLUE;
        LED_RED;
	//如果获取到数据,等待100ms存放到一个数组,
        Uart1_RecLen=0;
        while(LookUSART1_GetBuffCount())
        {
            Uart1_buffer_Clr();
            Get_Weight(10);
            /*first = 1;
            delay(300);
            while(LookUSART1_GetBuffCount())
            {                
                USART1_GetByte(&Uart1_byte);
                Uart1_buffer[Uart1_RecLen]=Uart1_byte;
                Uart1_RecLen=Uart1_RecLen+1;	
            }  */              
        }
        
        //检查是否有按键按下
		if(Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN,1) == KEY_ON)	//
		{
            Key_flag = 1;           
		}
    //如果按键中断标志为1,将重量信息发送给服务器,
        if(Key_flag == 1)
        {
            LED_GREEN;
            /*Key_flag = 0;             //debug模拟测试
            for(i=0;i<11;i++)
                temp_buffer[i]=Uart1_buffer[i];
            temp_buffer[i] = 0;*/
            
            USART2_DMASS("AT+CIPSEND=0,11\r\n",1000,1000);
            if(strstr(Ub,">")){
                for(i=0;i<11;i++)
                    USART2_SendByte(Uart1_buffer[i]);
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
            Uart1_buffer_Clr();		//
            USART1_RX_Buffer_Clear();
            LED_BLUE;
        }
        
        //每次循环中首先检测SIM7600是否处于开启状态			
		//if(!GetKeySTA())	//SIM7600意外关闭
		//{
			//SIM_RST();
			//break;
		//}

	}
}

int Get_Weight(uint16_t AfTime)
{
    int Uart1_RecLen=0;
    u8 Uart1_byte=0;
    uint16_t aftime;
        
    aftime=AfTime;
    do{
        delay(1);
        while(LookUSART1_GetBuffCount()){
            USART1_GetByte(&Uart1_byte);
            Uart1_buffer[Uart1_RecLen]=Uart1_byte;
            Uart1_RecLen=Uart1_RecLen+1;			
            aftime=AfTime;
        }		
        aftime--;
	}while(LookUSART1_GetBuffCount() || aftime>0);//
					
    USART1_RX_Buffer_Clear();			
    return Uart1_RecLen;			
}

void Uart1_buffer_Clr()
{
	uint16_t i;
	for(i=0;i<256;i++)
	{
		Uart1_buffer[i]=0x00;
	}
	
}

static void delay(uint16_t t)
{
	uint16_t i;
	while(t--)
	{
		for(i=0;i<8000;i++);
	}
}

/*
//获取ID
*/
void GetId()
{
	u32 CpuID[1];
	u8 i_d[9];
	u8 i;
	CpuID[0]=*(vu32*)(0x1ffff7f0);
	
	for(i=0;i<4;i++)
	{
		if(((CpuID[0]>>(2*i*4))&0x0f)>0x09){
			id[i*2]=((CpuID[0]>>(2*i*4))&0x0f)+0x37;
		}else{
			id[i*2]=((CpuID[0]>>(2*i*4))&0x0f)+0x30;
		}
		if(((CpuID[0]>>(2*i+1)*4)&0x0f)>0x09){
			id[i*2+1]=((CpuID[0]>>((2*i+1)*4))&0x0f)+0x37;
		}else{
			id[i*2+1]=((CpuID[0]>>((2*i+1)*4))&0x0f)+0x30;
		}
	}

	USART1_SendByte('I');
	USART1_SendByte('D');
	USART1_SendByte(':');
	for(i=0;i<8;i++)
	{
		USART1_SendByte(id[i]);
	}

}


/**
 * 初始化独立看门狗
 * prer:分频数:0~7(只有低 3 位有效!)
 * 分频因子=4*2^prer.但最大值只能是 256!
 * rlr:重装载寄存器值:低 11 位有效.
 * 时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
 */
void IWDG_Init(u8 prer,u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); /* 使能对寄存器IWDG_PR和IWDG_RLR的写操作*/
    IWDG_SetPrescaler(prer);    /*设置IWDG预分频值:设置IWDG预分频值*/
    IWDG_SetReload(rlr);     /*设置IWDG重装载值*/
    IWDG_ReloadCounter();    /*按照IWDG重装载寄存器的值重装载IWDG计数器*/
    IWDG_Enable();        /*使能IWDG*/
}

/**
 * 喂独立看门狗
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();    /*reload*/
}

/* ----------------------------------------end of file------------------------------------------------ */

