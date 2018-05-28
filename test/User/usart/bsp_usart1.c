#include "USART1_init.h"
#include "TIMER.h"
#include "bsp_led.h"

//���ջ���������
#define USART1_RX_BUFF_SIZEMAX 256
#define USART1_TX_BUFF_SIZEMAX 256
typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;
//�ڲ��ñ���
static u8 USART1_GetChar=0;							//���յ��ĵ����ַ�				�ж���ʹ��					
static u8 USART1RxBuffer[USART1_RX_BUFF_SIZEMAX];	//���ڽ��ջ����� 
static u8 USART1TxBuffer[USART1_TX_BUFF_SIZEMAX];	//���ڷ��ͻ����� 

static void Delay_MS(u32 nCount_temp);

extern uint16_t T_U1;
extern unsigned char T_U1_EN;

extern uint16_t TM_SEND;
extern uint16_t T_SEND;

//�������ñ���
USART1_STRUCT USART1_Real = 
{
	USART1_BAUD_RATE,				//������
	
	//���տ��ƶ���
	((void*)0),			//��ʼλ��
	((void*)0),			//����λ��
	((void*)0),			//����λ��
	((void*)0),			//���λ��
	((void*)0),			//�ռ�ָ��
	0,					//�ռ������ݸ���
};



/****************************************************************************
* ��	�ƣ�void USART1_GPIO_Init(void)
* ��	�ܣ��������ų�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART1_GPIO_Init(void)						//�������ų�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;		//�������Žṹ
	
	//�������ŷ���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//���ô��� Tx (PA.09) Ϊ�����������
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					//���ڷ�������
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//�����������
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//Ƶ��50MHz
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//��ʼ������
    
	// ���ô��� Rx (PA.10) Ϊ��������
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;					//���ڽ�������
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//��������
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//��ʼ������
}

/****************************************************************************
* ��	�ƣ�void USART1_Init(void)
* ��	�ܣ����ڳ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART1_Init(void)
{		
	USART_InitTypeDef UART_InitStructure;		//���ڽṹ

	//���ڷ���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//���ڳ�ʼ��
	UART_InitStructure.USART_BaudRate            = USART1_Real.BaudRate;			//������
	UART_InitStructure.USART_WordLength          = USART_WordLength_8b;				//����λ8bit
	UART_InitStructure.USART_StopBits            = USART_StopBits_1;				//ֹͣλ����
	UART_InitStructure.USART_Parity              = USART_Parity_No ;				//��������żЧ��
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//RTS��CTSʹ��(None��ʹ��)
	UART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	//���ͺͽ���ʹ��
	USART_Init(USART1, &UART_InitStructure);										//��ʼ������
}

/****************************************************************************
* ��	�ƣ�void USART1_NVIC_Init(void)
* ��	�ܣ������ж��������ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART1_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 						//�жϿ���������
	
	/* ѡ�����ȼ�����1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//�򿪴����ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ���ж�������
}

/****************************************************************************
* ��	�ƣ�void USART1_DMATxd_Init(void)
* ��	�ܣ�����DMA��ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART1_DMATxd_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 						//�жϿ���������
	DMA_InitTypeDef DMA_InitStructure;							//DMA�ṹ

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);			//ʹ��DMA1ʱ��

	/* ѡ�����ȼ�����1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//DMA�ж���������
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;	//����DMA�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//���ж�
	NVIC_Init(&NVIC_InitStructure); 

	//DMA����
	DMA_DeInit(DMA1_Channel4);  		   									//��λDMA1_Channel4ͨ��ΪĬ��ֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_BASE + 4;				//DMAͨ���������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1TxBuffer;				//DMAͨ���洢������ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMAĿ�ĵ�	(DMA_DIR_PeripheralSRCԴ)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//��ǰ����Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//��ǰ�洢�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݿ��Ϊ�ֽ�(8λ)
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݿ��Ϊ�ֽ�(8λ)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//��������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;					//DMAͨ�����ȼ��ǳ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//DMAͨ��δ���ô洢�����洢������
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);							//�����������ó�ʼ��DMA
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);    						//����DMAͨ���ж�
}

/****************************************************************************
* ��	�ƣ�void USART1_RX_Buffer_init(void)
* ��	�ܣ����ڽ��ճ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	���������ж�����ջ�������
****************************************************************************/
void USART1_RX_Buffer_init(void)
{
	USART1_Real.QStart = USART1RxBuffer;							//��ʼλ��
	USART1_Real.QEnd = &USART1RxBuffer[USART1_RX_BUFF_SIZEMAX - 1];	//����λ��
	USART1_Real.QIn = USART1_Real.QStart;							//����λ��=��ʼλ��
	USART1_Real.QOut = USART1_Real.QStart;							//���λ��=��ʼλ��
	USART1_Real.QDataBuf = USART1RxBuffer;							//ָ�����ջ������ռ�
	USART1_Real.QDataCount = 0;										//�ռ������ݸ���
}

/****************************************************************************
* ��	�ƣ�void USART1_Config(void)
* ��	�ܣ���������
* ��ڲ�������
* ���ڲ�������
* ˵	����Ĭ��Ϊ�����ݽ���					 
****************************************************************************/
void USART1_Config(void)
{
	USART1_Init();				//���ڳ�ʼ��
	USART1_GPIO_Init();			//�������ų�ʼ��
	USART1_NVIC_Init();			//�жϳ�ʼ��
//	USART1_DMATxd_Init();		//DMA���ͳ�ʼ��
	USART1_RX_Buffer_init();	//�����ж�����ջ�������

	USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//����ձ�־
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//���������ж�
	
	USART_Cmd(USART1, ENABLE);  								//ʹ��ʧ�ܴ�������	
}

/****************************************************************************
* ��	�ƣ�void USART1_RX_Buffer_Clear(void)	
* ��	�ܣ���ս��ջ�����
* ��ڲ�������
* ���ڲ�������
* ˵	������						
****************************************************************************/
void USART1_RX_Buffer_Clear(void)
{
	USART1_Real.QIn=USART1_Real.QStart;							//����λ��=��ʼλ��
	USART1_Real.QOut=USART1_Real.QStart;						//���λ��=��ʼλ��
	USART1_Real.QDataCount=0;									//�ռ������ݸ���
}

/****************************************************************************
* ��	�ƣ�u32 LookUSART1_GetBuffMax(void)	
* ��	�ܣ���ȡ���ջ�������С
* ��ڲ�������
* ���ڲ�������
* ˵	������			  
****************************************************************************/
u32 LookUSART1_GetBuffMax(void)
{
	return (u32)USART1_RX_BUFF_SIZEMAX;
}

/****************************************************************************
* ��	�ƣ�u32 LookUSART1_GetBuffCount(void)
* ��	�ܣ���ȡ�������н��յ������ݸ���
* ��ڲ�������
* ���ڲ�������
* ˵	������					  
****************************************************************************/
u32 LookUSART1_GetBuffCount(void)
{
	return USART1_Real.QDataCount;
}

/****************************************************************************
* ��	�ƣ�u8 USART1_GetByte(u8* Data)	
* ��	�ܣ�����������ȡ���ַ�����
* ��ڲ�����u8* Data ���յ�������
* ���ڲ�����u8	���ճɹ���־
			0 	���ճɹ�
			1 	����ʧ��
* ˵	�����ӽ��ջ������л�ȡ���ݣ���������ȴ�	 
***************************************************************************/
u8 USART1_GetByte(u8* Data)
{
	if(USART1_Real.QDataCount!=0)					//��δ�������ݴ���
	{
		*Data=*USART1_Real.QOut;					//�Ӷ�����ȡ����
		USART1_Real.QOut++;							//ָ����һ��δ�������ݵ�
		if(USART1_Real.QOut>USART1_Real.QEnd)		//�������һ���жϵ������ؿ�ʼ��
			USART1_Real.QOut=USART1_Real.QStart;	
		USART1_Real.QDataCount--;					//δ�������ݸ����ݼ�

		return 1;
	}
	return 0;
}

/****************************************************************************
* ��	�ƣ�u8 USART1_GetByte_WaitTime(u8* Data,u32 TimeLater)	
* ��	�ܣ�u8* Data ���յ�������
			u32 TimeLater	�ȴ�ʱ�� 7=1us
* ��ڲ�����u8	�Ƿ��н��յ�����
				0 û�н��յ�����
				1 ���յ�����
* ���ڲ��������յ��ĵ��ַ�����
* ˵	�����ӽ��ջ������л�ȡ���ݣ�ʱ���ȴ�
***************************************************************************/
u8 USART1_GetByte_WaitTime(u8* Data,u32 TimeLater)
{
	u8 flag=0;					//��ȡ��־
	u32 err=0;					//�����־
	u8 val=0;					//��ȡ�����ݴ�ŵ�
	while(1)
	{
		flag=USART1_GetByte(&val);
		if (flag==1)
		{
			*Data=val;
			return 1;
		}
		err++;
		if(err>=TimeLater)	//10���붼û��ȡ�����ݣ������˳�
			return 0;
	}
}

/****************************************************************************
* ��	�ƣ�u8 USART1_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater)	
* ��	�ܣ�����ʱ֮��ʼ����ָ���������µ�����
* ��ڲ�����u8* Data ���յ�������
			u32 DataLen		ϣ�����յ����ݸ���
			u32 TimeLater	���յȴ�ʱ�� 7=1us
* ���ڲ�����u32 ���յ����ݸ���	0û�н��յ�����
* ˵	������
***************************************************************************/
u32 USART1_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater)
{
	u8 G = 0;					//��õ�ֵ
	u32 i = 0;					//���յĸ���

	Delay_MS(TimeLater);	 	//���յȴ�

	if(LookUSART1_GetBuffCount() == 0)			//û�н��յ�����
		return 0;

	while(LookUSART1_GetBuffCount() != 0)		//�鿴���������ݽ��ո���
	{
		if(i >= DataLen - 1)		//����ϣ�����յĴ�С
			break;					//�˳�����

		if(USART1_GetByte(&G) == 1)		//ȡֵ�ɹ�
		{
			Data[i] = G;			//���ݻ���
			i++;
			//DebugPf("%c",G);		//�ն���ʾ
		}
	}
	return i;
}

/****************************************************************************
* ��	�ƣ�u8 USART1_GetByte_WaitLong(void)	
* ��	�ܣ���
* ��ڲ�����u8	���յ�������
* ���ڲ��������յ��ĵ��ַ�����
* ˵	�����ӽ��ջ������л�ȡ���ݣ������ȴ�
***************************************************************************/
u8 USART1_GetByte_WaitLong(void)
{
	u8 flag=0;					//��ȡ��־
	u8 val=0;					//��ȡ�����ݴ�ŵ�
	while(1)
	{
		flag=USART1_GetByte(&val);
		if (flag==1)
			return val;
	}
}

/****************************************************************************
* ��	�ƣ�void USART1_SendByte(u8 Data)
* ��	�ܣ����ַ�����
* ��ڲ�����Data 	���͵��ַ�����
* ���ڲ�������
* ˵	������				   
****************************************************************************/
void USART1_SendByte(u8 Data)		   //���ַ��������
{
	USART_SendData(USART1, Data);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/****************************************************************************
* ��	�ƣ�void USART1_SendString(u8* Data,u32 Len)
* ��	�ܣ����ַ����
* ��ڲ�����Data 	����ĵ��ַ�����
			Len		�ַ�����
* ���ڲ�������
* ˵	������					 
****************************************************************************/
void USART1_SendString(u8* Data,u32 Len)		   //���ַ����
{
	u32 i=0;
	for(i=0;i<Len;i++)
    {    
		USART_SendData(USART1, Data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
}

/****************************************************************************
* ��	�ƣ�void USART1_DMASendString(u8* Data,u32 Len)
* ��	�ܣ�DMA��ʽ���ַ����
* ��ڲ�����Data 	����ĵ��ַ�����
			Len		�ַ�����
* ���ڲ�������
* ˵	����������USART1_DMATxd_Init��ʼ��֮�����ʹ��
			DMA����CPU���� �� ��CPU���� ���з��ͳ�ͻ	 
****************************************************************************/
void USART1_DMASendString(u8* Data,u32 Len)		   //���ַ����
{
	memcpy(USART1TxBuffer, Data, Len);			   //�������ݵ����ͻ�����
    DMA1_Channel4->CNDTR = Len;					   //�����ֽ�����
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); //����
	DMA_Cmd(DMA1_Channel4, ENABLE);				   //ʼ��DMAͨ��
}

/****************************************************************************
* ��	�ƣ�void USART1_IRQHandler(void)	
* ��	�ܣ��жϻ���
* ��ڲ�������
* ���ڲ�������
* ˵	�������յ������ݴ�����ջ�����
	 USART_GetITStatus		���ָ����USART�жϷ������
	 USART_GetFlagStatus	���ָ����USART��־λ�������
****************************************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)				// ���ڽ������ݴ����ж�
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//��ս����жϱ�־
		USART1_GetChar = USART_ReceiveData(USART1);					//���յ����ַ�����
		
		if(USART1_Real.QDataCount < USART1_RX_BUFF_SIZEMAX)			//����ռ�δ��,���ձ������� 
		{
			if(USART1_Real.QIn > USART1_Real.QEnd)					//����пռ�ɴ����ݣ�������λ����ĩβλ��
				USART1_Real.QIn = USART1_Real.QStart;				//������λ��������ʼλ��
			*USART1_Real.QIn= USART1_GetChar;						//���յ����ַ����ݴ��뻺��������λ��
			USART1_Real.QIn++;										//����λ������
			USART1_Real.QDataCount++;								//�ռ������ݸ�������
		}
	}

	else if(USART_GetFlagStatus(USART1, USART_IT_ORE) == SET)		//����Ƿ��н������
    {
	    USART_ReceiveData(USART1);									//����������־��ֻ���ö����ݵķ�ʽ���������־
    }

	else if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)				//���ڷ������ݴ����ж�
	{
		;
	}
}

//�жϻ��洮������
//uint8_t U1_P = 0;
//unsigned char U1BUF[256]={0};
//extern unsigned char U1RECV[50];	//����1�յ��ĵ�����Ч���ݰ�
//extern unsigned char U1TOU2[50];	//����1�յ��ĵ�����Ч���ݰ�
//extern unsigned char id[];


//void USART1_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
//	{
//		USART_ClearFlag(USART1,USART_FLAG_RXNE);
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//		if(USART_ReceiveData(USART1)!=0x00){
//			U1BUF[U1_P] = USART_ReceiveData(USART1);
//			U1_P++;		
//		}
//	}
//	else if(USART_GetFlagStatus(USART1, USART_IT_ORE) == SET)		//����Ƿ��н������
 // {
//		USART_ReceiveData(USART1);									//����������־��ֻ���ö����ݵķ�ʽ���������־
//  }
	
//}

//��ȡ���յ������ݺͳ���
//unsigned char *get_rebuff1(uint8_t *len)
//{
//    *len = U1_P;
//    return (unsigned char *)&U1BUF;
//}

//void clean_rebuff1(void)
//{
//	uint8_t i;
//	for(i=0;i<255;i++){
//		U1BUF[i]=0x00;
//	}
//	U1BUF[255]=0x00;
//  U1_P = 0;
//}



/****************************************************************************
* ��	�ƣ�void DMA1_Channel4_IRQHandler(void)	
* ��	�ܣ�DMA�жϻ���
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)) //����������
	{
		USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);    //�ر�DMA����
		DMA_Cmd(DMA1_Channel4, DISABLE);	       			//�ر�DMAͨ��  
	}

	DMA_ClearFlag(DMA1_FLAG_GL4| DMA1_FLAG_TC4 | DMA1_FLAG_HT4 | DMA1_FLAG_TE4);  //���DMA��ر�־
}

/****************************************************************************
* ��	�ƣ�USART1_STRUCT* USART1GetDCB(void)
* ��	�ܣ���ȡ����
* ��ڲ�������
* ���ڲ��������ýṹ��
* ˵	������
****************************************************************************/
USART1_STRUCT* USART1GetDCB(void)
{
	return ((USART1_STRUCT*)(&USART1_Real));
}

/****************************************************************************
* ��	�ƣ�void Delay_MS(u32 nCount_temp)
* ��	�ܣ����뼶
* ��ڲ�����u32 nCount_temp	��ʱ��
* ���ڲ�������
* ˵	������
****************************************************************************/
void Delay_MS(u32 nCount_temp)
{
	u32 nCount=nCount_temp*8000;
	while(nCount--);
}
