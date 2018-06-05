/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "bsp_TiMbase.h"
#include "bsp_usart2.h"
#include "sim7600.h"

unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";

volatile uint8_t Customer_Num = 0;//�ͻ�����
volatile uint8_t Customer_Current = 0;//��ǰ�ͻ�
volatile uint8_t pCustomer1 = 0;//�ͻ����ش���
volatile uint8_t pCustomer2 = 0;
volatile uint8_t pCustomer3 = 0;
volatile uint8_t pCustomer4 = 0;
volatile struct customer Customer1[16];//����������Ϣ
volatile struct customer Customer2[16];
volatile struct customer Customer3[16];
volatile struct customer Customer4[16];

volatile uint32_t time = 0; // ms ��ʱ���� 

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
   /* led �˿����� */ 
	LED_GPIO_Config();
    BASIC_TIM6_Init();
    /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
    USART_Config();
    USART2_Config();
    BASIC_TIM7_Init();
    SIM_GPIO_Config();
    while(!Sim_ini()){
		SIM_RST();
	}
	
    while(1)
	{	
        if(Customer_Num == 0)
            Customer_Current = 0;
        
        switch ( KeyboardScan() )
        {
            case A: //����ĳ���ͻ������߼�
                break;
            case B: //����,����µĿͻ�,�������봦���߼�
                break;    
            case C: //����
                break;  
            case D: //����
                break;  
            case E: //�ۼ�
                break; 
            case F: //����
                break;            
        }
        if( KeyboardScan() == 'A') //A:�ۼ�
	}	
}
/*********************************************END OF FILE**********************/
