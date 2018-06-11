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
#include "keyboard.h"
#include "sim7600.h"
#include <string.h>

unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";

volatile int8_t Customer_Num = 0;//�ͻ�����
volatile int Customer_Current = 0;//��ǰ�ͻ�
volatile struct customer Customer[4][16];//���г�����Ϣ
volatile uint8_t pCustomer[4][3];

volatile uint32_t time = 0; // ms ��ʱ���� 



/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
    uint8_t keyboard_value;
    char weight_value[20];
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
        
        keyboard_value = keyboard_scan();
        switch ( keyboard_value )
        {
            case A: //0~9 . ����ĳ���ͻ������߼�
                if(pCustomer[Customer_Current][2] == 0)
                {
                    if(pCustomer[Customer_Current][1] == 0)
                    {
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].type = keyboard_value;
                    }
                    else if(pCustomer[Customer_Current][1] == 1)
                    {
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].unit_price = strncat( Customer[Customer_Current][pCustomer[Customer_Current][0]].unit_price, &keyboard_value, 1 );
                    }
                    else if(pCustomer[Customer_Current][1] == 2)
                    {
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].Block = strncat( Customer[Customer_Current][pCustomer[Customer_Current][0]].Block, &keyboard_value, 1 );
                    }
                }
                else if(pCustomer[Customer_Current][2] == 1)
                {
                    if(pCustomer[Customer_Current][1] == 0)
                    {
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].unit_price = strncat( Customer[Customer_Current][pCustomer[Customer_Current][0]].unit_price, &keyboard_value, 1 );
                    }
                }
                break;
            case B: //ȷ��
                if(pCustomer[Customer_Current][2] == 0)
                {
                    if(pCustomer[Customer_Current][1] == 0)
                    {
                        pCustomer[Customer_Current][1] = 1;
                    }
                    else if(pCustomer[Customer_Current][1] == 1)
                    {
                        pCustomer[Customer_Current][1] = 2;
                    }
                    else if(pCustomer[Customer_Current][1] == 2)
                    {
                        pCustomer[Customer_Current][1] = 3;
                    }
                    if(pCustomer[Customer_Current][1] == 3)
                    {
                        weight_value = Get_Weight();
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].weight = weight_value;
                        pCustomer[Customer_Current][1] = 4;
                        //send to server
                    }
                }
                else if(pCustomer[Customer_Current][2] == 1)
                {
                    if(pCustomer[Customer_Current][1] == 0)
                    {
                        pCustomer[Customer_Current][1] = 1;
                    }
                    if(pCustomer[Customer_Current][1] == 1)
                    {
                        weight_value = Get_Weight();
                        Customer[Customer_Current][pCustomer[Customer_Current][0]].weight = weight_value;
                        pCustomer[Customer_Current][1] = 2;
                        //send to server
                    }
                }
                break;  
            case C: //�ۼ�
                if(pCustomer[Customer_Current][2] == 0)
                {
                    if(pCustomer[Customer_Current][1] == 4)
                    {
                        pCustomer[Customer_Current][1] = 0;
                    }
                }
                else if(pCustomer[Customer_Current][2] == 1)
                {
                    if(pCustomer[Customer_Current][1] == 2)
                    {
                        pCustomer[Customer_Current][1] = 0;
                    }
                }
                break; 
            case D: //ȥƤ
                pCustomer[Customer_Current][2] = 1;
                break;  
            case E: //����,����µĿͻ�,�������봦���߼�
                if(Customer_Num <= 4)
                {
                    do{
                    Customer_Current++;
                    if(Customer_Current >= 4)
                        Customer_Current = 0;
                    }while((pCustomer[Customer_Current][1] != 0) || (pCustomer[Customer_Current][2] != 0));
                    Customer_Num++;
                }
                break;    
            case F: //����
                if(Customer_Num >= 2)
                {
                    do{
                    Customer_Current--;
                    if(Customer_Current < 0)
                        Customer_Current = 4;
                    }while((pCustomer[Customer_Current][1] == 0) || (pCustomer[Customer_Current][2] == 0));
                }
                break;  
            case G: //����
                if(Customer_Num >= 2)
                {
                    do{
                    Customer_Current++;
                    if(Customer_Current > 4)
                        Customer_Current = 0;
                    }while((pCustomer[Customer_Current][1] == 0) || (pCustomer[Customer_Current][2] == 0));
                }
                break;  
            case H: //����
                //send to server
                for (int i=0; i<=pCustomer[Customer_Current][0]; i++);
                {
                    Customer[Customer_Current][i] = 0;
                }
                pCustomer[Customer_Current][0] = 0;
                pCustomer[Customer_Current][1] = 0;
                pCustomer[Customer_Current][2] = 0;               
                Customer_Num--;
                break;                  
        }
        if( KeyboardScan() == 'A') //A:�ۼ�
	}	
}
/*********************************************END OF FILE**********************/
