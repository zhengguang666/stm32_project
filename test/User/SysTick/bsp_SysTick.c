/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SysTick ??????10us?????,?????????,
  *          ???? 1us 10us 1ms ???     
  ******************************************************************************
  * @attention
  *
  * ????:?? iSO STM32 ??? 
  * ??    :http://www.chuxue123.com
  * ??    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_SysTick.h"

static __IO u32 TimingDelay;
 
/**
  * @brief  ????????? SysTick
  * @param  ?
  * @retval ?
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms????
	 * SystemFrequency / 100000	 10us????
	 * SystemFrequency / 1000000 1us????
	 */ 
 
  if (SysTick_Config(SystemCoreClock / 1000000))	// ST3.5.0???
	{ 
		/* Capture error */ 
		while (1);
	} 
  
		// ???????  
	//SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief   us????,10us?????
  * @param  
  *		@arg nTime: Delay_us( 1 ) ??????? 1 * 10us = 10us
  * @retval  ?
  */
void Delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	// ???????  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

/**
  * @brief  ??????
  * @param  ?
  * @retval ?
  * @attention  ? SysTick ???? SysTick_Handler()??
  */
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}
/*********************************************END OF FILE**********************/
