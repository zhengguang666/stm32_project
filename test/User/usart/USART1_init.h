#ifndef _USART1_INIT_H_	
#define _USART1_INIT_H_

#ifdef __cplusplus			//�����CPP����C����
extern "C" {
#endif

#include "stm32f10x.h"
#include "static_init.h"	//���ڽṹ��
    
#define USART1_BAUD_RATE 1200

//USART�豸���ƽṹ��
typedef struct
{
	u32			BaudRate;				//������

	//���տ��ƶ���
	u8			*QStart;		        //��ʼλ��
	u8			*QEnd;		            //����λ��
    u8			*QIn;				    //����λ��
	u8			*QOut;				    //���λ��
	u8			*QDataBuf;            	//�ռ�ָ��
    u32			QDataCount;             //�ռ������ݸ��� 
} USART1_STRUCT;

void USART1_Config(void);		  				//��������
void USART1_RX_Buffer_Clear(void); 				//��ս��ջ�����

void USART1_SendByte(u8 Data);					//���ַ����ݷ���
void USART1_SendString(u8* Data,u32 Len);		//���ַ�����
void USART1_DMASendString(u8* Data,u32 Len);	//DMA���ַ�����
u8 USART1_GetByte(u8* Data);					//���ַ�����
u8 USART1_GetByte_WaitTime(u8* Data,u32 TimeLater);	//���ڵ��ַ����յȴ�ʱ��
u32 USART1_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater);	//�ȴ�ʱ��֮���ȡ���ֽ�����
u8 USART1_GetByte_WaitLong(void);				//���ڵ��ַ�����һֱ�ȴ�
void USART1_GPIO_Init(void);						//�������ų�ʼ��

USART1_STRUCT* USART1GetDCB(void);				//����Ӧ������
u32 LookUSART1_GetBuffMax(void);				//��ѯ���ջ�������С
u32 LookUSART1_GetBuffCount(void);				//��ѯ�������н��յ������ݸ���


#ifdef __cplusplus		   //�����CPP����C���� //��������
}
#endif

#endif
