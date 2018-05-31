/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>
#include "usb_lib.h"
#include "usb_istr.h"
#include "bsp_led.h"
#include "sim800l.h"

#include "bsp_TiMbase.h"

extern volatile uint32_t time;

// ����ͷ-����ov7725
extern u8 Ov7725_vsync;

// iso �������
volatile uint8_t rec_cmd;
extern void Soft_Reset(void);


extern vu16 CCR1_Val;					/* ��������Ƚ�ͨ��1�������ڱ���*/
extern vu16 CCR2_Val;					/* ��������Ƚ�ͨ��2�������ڱ���*/
extern vu16 CCR3_Val;					/* ��������Ƚ�ͨ��3�������ڱ���*/
extern vu16 CCR4_Val;					/* ��������Ƚ�ͨ��4�������ڱ���*/

extern unsigned char PK_Cnt;	//��ʱ����

extern bool TIMOK;

extern bool FLAG_BLE_CTS0;//CTS�Ƿ����ͣ�����Ϊ1������Ϊ0
extern bool FLAG_BLE_CTS1;//CTS�Ƿ����ߣ�����Ϊ1������Ϊ0

// ϵͳ��ʱ��
extern void TimingDelay_Decrement(void);

// rtc
extern uint32_t TimeDisplay;

extern uint8_t key_state;

extern bool flag_send;//TCP���ͱ�ʶ
extern bool flag_get;//TCP��ȡ��ʶ
extern bool flag_cbc;	//��ʱ��������ʶ
extern bool flag_chg;//����ʶ
extern bool flag_gps;//GPS OK��ʶ
extern bool flag_bmp;//BMP��ȡ��ʶ
extern bool flag_gsm;	//GSM Registe OK Flag:1 ok,0 NOT OK
extern bool flag_pos;//��ʱ��ȡGPSλ�ñ�ʶ

uint16_t TM_GPS=1;
uint16_t TM_SEND=10;
uint16_t TM_GET=1;

uint16_t T_GPS=1;	//GPS��ȡ�����1s
uint16_t T_SEND=1;	//TCP���ͼ����1s
uint16_t T_GET=1;	//TCP���ͼ����1s
uint16_t T_FLASH=1;	//��˸

uint16_t T_U1=0;
unsigned char T_U1_EN=0;

uint16_t cntt=0;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		printf("\r\n hardfault");
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
		printf("\r\n MemManage");
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
		printf("\r\n BusFault");
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
		printf("\r\n UsageFault");
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	
}




//��ʱ��2
void TIM2_IRQHandler(void)
{
	vu16  capture = 0; 			/* ��ǰ�������ֵ�ֲ����� */

	/* 
	*	TIM2 ʱ�� = 72 MHz, ��Ƶ�� = 7299 + 1, TIM2 counter clock = 10KHz
	*	CC1 ������ = TIM2 counter clock / CCRx_Val
	*/
		
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{

//GPS OK
		if(flag_gps){			
			//LED_TOGGLE;
		}
//GPS GET
		T_GPS--;
		if(T_GPS==0){
			T_GPS=TM_GPS;
			flag_pos=1;
		}
			
//TCP SEND		
		T_SEND--;
		if(T_SEND==0){
			T_SEND=TM_SEND;
			flag_send=1;			
		}
//TCP GET		
		T_GET--;
		if(T_GET==0){
			T_GET=TM_GET;
			flag_get=1;			
		}

		/* ������ǰ����ֵ */
		capture = TIM_GetCapture1(TIM2);
		/* ���ݵ�ǰ����ֵ�����������Ĵ��� */
		TIM_SetCompare1(TIM2, capture + CCR1_Val);

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	}
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//���ڿ��ػ��������
	{		
		
		capture = TIM_GetCapture2(TIM2);
		TIM_SetCompare2(TIM2, capture + CCR2_Val);

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
	else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
	{

		capture = TIM_GetCapture3(TIM2);
		TIM_SetCompare3(TIM2, capture + CCR3_Val);

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	}
	else
	{
		
		capture = TIM_GetCapture4(TIM2);
		TIM_SetCompare4(TIM2, capture + CCR4_Val);

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}
}

/*
 * ��������USB_LP_CAN1_RX0_IRQHandler
 * ����  ��USB�жϺ�CAN�����жϷ������USB��CAN����I/O������ֻ�õ�CAN���жϡ�
 * ����  ����
 * ���  : ��	 
 * ����  ����
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{

  
}



/*
 * ��������SDIO_IRQHandler
 * ����  ����SDIO_ITConfig(���������������sdio�ж�	��
 			���ݴ������ʱ�����ж�
 * ����  ����		 
 * ���  ����
 */
void SDIO_IRQHandler(void) 
{
  /* Process All SDIO Interrupt Sources */

}

/*
 * ��������USB_LP_CAN1_RX0_IRQHandler
 * ����  ��USB�����ȼ��ж�����
 * ����  ����		 
 * ���  ����
 */
#if 0
void USB_LP_CAN1_RX0_IRQHandler(void)
{

}
#endif
/*
 * ��������USB_HP_CAN1_TX_IRQHandler
 * ����  ��USB�����ȼ��ж����� �����ж�
 * ����  ����		 
 * ���  ����
 */
void USB_HP_CAN1_TX_IRQHandler(void)
{

}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void  BASIC_TIM_IRQHandler (void)
{
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
	{	
		time++;
		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  		 
	}		 	
}