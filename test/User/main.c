
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
uint8_t CentL,CentR;//����������ֵ�洢��Ԫ

unsigned char PK_Cnt=0;//���ػ���ť���º󣬼�ʱ����

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

bool FLAG_BLE_CTS0=0;//CTS�Ƿ����ͣ�����Ϊ1������Ϊ0
bool FLAG_BLE_CTS1=0;//CTS�Ƿ����ߣ�����Ϊ1������Ϊ0
bool FLAG_LONG=0;		//�ְ������ݽ��ձ�ʶ��׼������Ϊ1����Ϊ0

bool flag_send;//TCP���ͱ�ʶ
bool flag_get;//TCP��ȡ��ʶ
bool flag_cbc;//��ʱ��������ʶ
bool flag_gps;//GPS OK��ʶ
bool flag_svs=0;//���������ݸ��±�ʶ
bool flag_pos=0;//��ʱ��ȡGPSλ�ñ�ʶ
	
unsigned char hd[]="TE005:";
unsigned char id[]="00000000";
unsigned char tp[]="0";
unsigned char lat[]="0000.0000";
unsigned char lng[]="00000.0000";
unsigned char cbc[]="000";
unsigned char end[]="END";

unsigned char U1RECV[50]={0};	//����1�յ��ĵ�����Ч���ݰ�
unsigned char U1SEND[50]={0};	//����1���͵���Ч���ݰ�
unsigned char U1TOU2[50]={0};	//����1����������͵�����2������ID��
unsigned char U2TOU1[50]={0};	//����2����������͵�����1��ȥ��ID��
unsigned char U2RECV[50]={0};	//����2�յ��ĵ�����Ч���ݰ�
unsigned char U2SEND[50]={0};	//����2���͵���Ч���ݰ�


unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";
//unsigned char ip[]="172.21.186.10";
//unsigned char pt[]="10001";

uint8_t sts;	//SIM868 STATUS PIN

bool flag_rst=0;	//SIM868 RESET��ʶ
bool flag_cool=0;	//SIM868��������ʶ��ͨ��PWR������Ϊ������0��ͨ��RSTΪ������1
bool flag_gsm=0;	//GSM Registe OK Flag:1 ok,0 NOT OK


u8 Ub[256];
u8 Uart1_buffer[256];
u8 temp_buffer[256];
u8 Key_flag=0;
#define WEIGHT_INFO_LEN 12

/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{ 
	char *s;
	uint8_t i,j;
	uint8_t LEN[2];
    int Uart1_RecLen=0;
    u32 temp=0,first=0;
    uint8_t Send_Fail_Count=0;
	GPIO_InitTypeDef GPIO_InitStructure;		//�������Žṹ
	
//	/* ����ϵͳʱ�� */
	RCC_Configuration();
//	/* ���� NVIC */
	NVIC_Configuration();
	
	LED_GPIO_Config();
    Key_GPIO_Config();
	SIM_GPIO_Config();
	
	ISO_Init();//��ʼ������1/2
	//printf("\r\n========SIM7600CE���Գ���========\r\n");
	//printf("*�������ṩ��ATָ�����SIM7600CEģ��Ļ���˼·\r\n");
	//printf("*ʵ��ģ���ʼ����TCP/IP��HTTPS��GPS��WLAN�ȹ���\r\n");
	//printf("*��������������ATָ���ֲ�\r\n");
	//printf("*�������ʣ��ɼ������ǵļ�������QQȺ��586340506\r\n");
	//printf("*���߷�ʽ��\r\n");
	//printf("*PA2--RXD\r\n");
	//printf("*PA3--TXD\r\n");
	//printf("*PB4--IGT\r\n");
	//printf("*PB3--STATUS\r\n");	
	//printf("===============================\r\n");
	
	//GetId();
    
    //��ʼ��SIM7600CE			
	while(!Sim_ini()){
				SIM_RST();
	}
	Uart1_buffer_Clr();		//
	USART1_RX_Buffer_Clear();
	while(1){
        //LED_BLUE;
        LED_RED;
	//�����ȡ������,�ȴ�100ms��ŵ�һ������,
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
        
        //����Ƿ��а�������
		if(Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN,1) == KEY_ON)	//
		{
            Key_flag = 1;           
		}
    //��������жϱ�־Ϊ1,��������Ϣ���͸�������,
        if(Key_flag == 1)
        {
            LED_GREEN;
            /*Key_flag = 0;             //debugģ�����
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
        
        //ÿ��ѭ�������ȼ��SIM7600�Ƿ��ڿ���״̬			
		//if(!GetKeySTA())	//SIM7600����ر�
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
//��ȡID
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
 * ��ʼ���������Ź�
 * prer:��Ƶ��:0~7(ֻ�е� 3 λ��Ч!)
 * ��Ƶ����=4*2^prer.�����ֵֻ���� 256!
 * rlr:��װ�ؼĴ���ֵ:�� 11 λ��Ч.
 * ʱ�����(���):Tout=((4*2^prer)*rlr)/40 (ms).
 */
void IWDG_Init(u8 prer,u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); /* ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����*/
    IWDG_SetPrescaler(prer);    /*����IWDGԤ��Ƶֵ:����IWDGԤ��Ƶֵ*/
    IWDG_SetReload(rlr);     /*����IWDG��װ��ֵ*/
    IWDG_ReloadCounter();    /*����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������*/
    IWDG_Enable();        /*ʹ��IWDG*/
}

/**
 * ι�������Ź�
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();    /*reload*/
}

/* ----------------------------------------end of file------------------------------------------------ */

