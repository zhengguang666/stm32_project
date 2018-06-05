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
#include "sim7600.h"

unsigned char ip[]="position.iego.net";
unsigned char pt[]="10001";

volatile uint8_t Customer_Num = 0;//客户数量
volatile uint8_t Customer_Current = 0;//当前客户
volatile uint8_t pCustomer1 = 0;//客户称重次数
volatile uint8_t pCustomer2 = 0;
volatile uint8_t pCustomer3 = 0;
volatile uint8_t pCustomer4 = 0;
volatile struct customer Customer1[16];//所有稳重信息
volatile struct customer Customer2[16];
volatile struct customer Customer3[16];
volatile struct customer Customer4[16];

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
            case A: //进入某个客户处理逻辑
                break;
            case B: //多人,添加新的客户,并处进入处理逻辑
                break;    
            case C: //向上
                break;  
            case D: //向下
                break;  
            case E: //累加
                break; 
            case F: //结算
                break;            
        }
        if( KeyboardScan() == 'A') //A:累加
	}	
}
/*********************************************END OF FILE**********************/
