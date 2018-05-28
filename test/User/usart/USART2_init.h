#ifndef _USART2_INIT_H_	
#define _USART2_INIT_H_

#ifdef __cplusplus			//�����CPP����C����
extern "C" {
#endif

#include "stm32f10x.h"
#include "static_init.h"	//���ڽṹ��

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
} USART2_STRUCT;

void USART2_Config(void);		  				//��������
void USART2_RX_Buffer_Clear(void); 				//��ս��ջ�����
void USART2_DMATxd_Init(void);
void USART2_SendByte(u8 Data);					//���ַ����ݷ���
void USART2_SendString(u8* Data,u32 Len);		//���ַ�����
void USART2_DMASendString(u8* Data,u32 Len);	//DMA���ַ�����
u8 USART2_GetByte(u8* Data);					//���ַ�����
u8 USART2_GetByte_WaitTime(u8* Data,u32 TimeLater);	//���ڵ��ַ����յȴ�ʱ��
u8 USART2_GetByte_WaitLong(void);				//���ڵ��ַ�����һֱ�ȴ�
u32 USART2_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater);	//�ȴ�ʱ��֮���ȡ���ֽ�����

USART2_STRUCT* USART2GetDCB(void);				//����Ӧ������
u32 LookUSART2_GetBuffMax(void);				//��ѯ���ջ�������С
u32 LookUSART2_GetBuffCount(void);				//��ѯ�������н��յ������ݸ���

#ifdef __cplusplus		   //�����CPP����C���� //��������
}
#endif

#endif
