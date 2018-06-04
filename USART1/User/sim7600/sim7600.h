#ifndef _SIM7600_H_	
#define _SIM7600_H_
#include "stm32f10x.h"

#define	digitalHi(p,i)			{p->BSRR=i;}					
#define digitalLo(p,i)			{p->BRR=i;}				
#define digitalToggle(p,i)		{p->ODR ^=i;}	

#define  SIM_GPIO_STATUS_PORT   GPIOA 
#define  SIM_GPIO_STATUS_PIN    GPIO_Pin_5
#define  SIM_GPIO_IGT_PORT   GPIOA 
#define  SIM_GPIO_IGT_PIN    GPIO_Pin_4

#define  SIM_GPIO_STATUS_CLK                RCC_APB2Periph_GPIOA
#define  SIM_GPIO_STATUS_APBxClkCmd         RCC_APB2PeriphClockCmd

#define  SIM_GPIO_IGT_CLK                RCC_APB2Periph_GPIOA
#define  SIM_GPIO_IGT_APBxClkCmd         RCC_APB2PeriphClockCmd

#define IGT_Hi				digitalHi(GPIOA,GPIO_Pin_4)
#define IGT_Lo				digitalLo(GPIOA,GPIO_Pin_4)

void SIM_GPIO_Config(void);
uint8_t Sim_ini(void);
void PWR_ON();
void SIM_RST();
uint8_t Creg_CK();
void PWR_OFF();

#endif