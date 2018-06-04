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

unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";

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
	
  while(1)
	{	
        if ( time == 1000 ) /* 1000 * 1 ms = 1s ʱ�䵽 */
        {
            time = 0;
			/* LED1 ȡ�� */      
			LED1_TOGGLE; 
        } 
	}	
}
/*********************************************END OF FILE**********************/
