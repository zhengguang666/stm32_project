/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
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

volatile uint32_t time = 0; // ms 计时变量 

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
   /* led 端口配置 */ 
	LED_GPIO_Config();
    BASIC_TIM6_Init();
    /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
    USART_Config();
    USART2_Config();
    BASIC_TIM7_Init();
	
  while(1)
	{	
        if ( time == 1000 ) /* 1000 * 1 ms = 1s 时间到 */
        {
            time = 0;
			/* LED1 取反 */      
			LED1_TOGGLE; 
        } 
	}	
}
/*********************************************END OF FILE**********************/
