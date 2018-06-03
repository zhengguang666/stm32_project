
#include "bsp_iso_test.h"
#include "bsp_led.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_TiMbase.h"
#include "crc.h"
#include "TIMER.h"
#include "FLASH.h"
#include "SIM800L.h"
#include <string.h>
#include "USART1_init.h"
#include "USART2_init.h"
#include "stm32f10x_iwdg.h"

static void delay(uint16_t t);
//void GetId();

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
u8 Key_flag=0;
uint8_t Send_Fail_Count=0;
volatile uint32_t time = 0; // ms 计时变量 
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
	GPIO_InitTypeDef GPIO_InitStructure;		//串口引脚结构
	
//	/* 设置系统时钟 */
	RCC_Configuration();
//	/* 设置 NVIC */
	NVIC_Configuration();
	
	LED_GPIO_Config();
    Key_GPIO_Config();
	SIM_GPIO_Config();
    BASIC_TIM_Init();
	
	ISO_Init();//初始化串口1/2
    
	
	//GetId();
    
    //初始化SIM7600CE			
	//while(!Sim_ini()){
				//SIM_RST();
	//}
	while(1){
        LED_RED;
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
/*
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

}*/


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

