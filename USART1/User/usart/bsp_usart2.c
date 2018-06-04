
#include "bsp_usart2.h"

u8 Usart2_temp_buffer[256];

//���ջ���������
#define USART2_RX_BUFF_SIZEMAX 256
#define USART2_TX_BUFF_SIZEMAX 256

//�ڲ��ñ���
static u8 USART2_GetChar=0;							//���յ��ĵ����ַ�				�ж���ʹ��					
static u8 USART2RxBuffer[USART2_RX_BUFF_SIZEMAX];	//���ڽ��ջ����� 

//�������ñ���
USART2_STRUCT USART2_Real = 
{
	115200,				//������
	
	//���տ��ƶ���
	((void*)0),			//��ʼλ��
	((void*)0),			//����λ��
	((void*)0),			//����λ��
	((void*)0),			//���λ��
	((void*)0),			//�ռ�ָ��
	0,					//�ռ������ݸ���
};

/****************************************************************************
* ��	�ƣ�void USART2_GPIO_Init(void)
* ��	�ܣ��������ų�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART2_GPIO_Init(void)			//�������ų�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;		//�������Žṹ
	
	//�������ŷ���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//���ô��� Tx (PA.02) Ϊ�����������
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;					//���ڷ�������
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//�����������
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//Ƶ��50MHz
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//��ʼ������
    
	// ���ô��� Rx (PA.03) Ϊ��������
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;					//���ڽ�������
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//��������
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//��ʼ������
}

/****************************************************************************
* ��	�ƣ�void USART2_Init(void)
* ��	�ܣ����ڳ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART2_Init(void)
{		
	USART_InitTypeDef UART_InitStructure;		//���ڽṹ

	//���ڷ���ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//���ڳ�ʼ��
	UART_InitStructure.USART_BaudRate            = USART2_Real.BaudRate;	//������
	UART_InitStructure.USART_WordLength          = USART_WordLength_8b;		//����λ8bit
	UART_InitStructure.USART_StopBits            = USART_StopBits_1;		//ֹͣλ����
	UART_InitStructure.USART_Parity              = USART_Parity_No ;		//��������żЧ��
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//RTS��CTSʹ��(None��ʹ��)
	UART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	//���ͺͽ���ʹ��
	USART_Init(USART2, &UART_InitStructure);	//��ʼ������
}

/****************************************************************************
* ��	�ƣ�void USART2_NVIC_Init(void)
* ��	�ܣ������ж��������ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART2_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 		//�жϿ���������
	
	/* ѡ�����ȼ�����1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//�򿪴����ж�
	NVIC_Init(&NVIC_InitStructure);								//��ʼ���ж�������
}

/****************************************************************************
* ��	�ƣ�void USART2_RX_Buffer_init(void)
* ��	�ܣ����ڽ��ճ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	���������ж�����ջ�������
****************************************************************************/
void USART2_RX_Buffer_init(void)
{
	USART2_Real.QStart = USART2RxBuffer;							//��ʼλ��
	USART2_Real.QEnd = &USART2RxBuffer[USART2_RX_BUFF_SIZEMAX - 1];	//����λ��
	USART2_Real.QIn = USART2_Real.QStart;							//����λ��=��ʼλ��
	USART2_Real.QOut = USART2_Real.QStart;							//���λ��=��ʼλ��
	USART2_Real.QDataBuf = USART2RxBuffer;							//ָ�����ջ������ռ�
	USART2_Real.QDataCount = 0;										//�ռ������ݸ���
}

/****************************************************************************
* ��	�ƣ�void USART2_Config(void)
* ��	�ܣ���������
* ��ڲ�������
* ���ڲ�������
* ˵	����Ĭ��Ϊ�����ݽ���					 
****************************************************************************/
void USART2_Config(void)
{
	USART2_Init();				//���ڳ�ʼ��
	USART2_GPIO_Init();			//�������ų�ʼ��
	USART2_NVIC_Init();			//�жϳ�ʼ��
//	USART2_DMATxd_Init();		//DMA���ͳ�ʼ��
	USART2_RX_Buffer_init();	//�����ж�����ջ�������

	USART_ClearITPendingBit(USART2, USART_IT_RXNE);				//����ձ�־
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);				//���������ж�
	
	USART_Cmd(USART2, ENABLE);  								//ʹ��ʧ�ܴ�������	
}

/****************************************************************************
* ��	�ƣ�void USART2_RX_Buffer_Clear(void)	
* ��	�ܣ���ս��ջ�����
* ��ڲ�������
* ���ڲ�������
* ˵	������						
****************************************************************************/
void USART2_RX_Buffer_Clear(void)
{
	USART2_Real.QIn=USART2_Real.QStart;							//����λ��=��ʼλ��
	USART2_Real.QOut=USART2_Real.QStart;						//���λ��=��ʼλ��
	USART2_Real.QDataCount=0;									//�ռ������ݸ���
}

/****************************************************************************
* ��	�ƣ�u32 LookUSART2_GetBuffMax(void)	
* ��	�ܣ���ȡ���ջ�������С
* ��ڲ�������
* ���ڲ�������
* ˵	������			  
****************************************************************************/
u32 LookUSART2_GetBuffMax(void)
{
	return (u32)USART2_RX_BUFF_SIZEMAX;
}

/****************************************************************************
* ��	�ƣ�u32 LookUSART2_GetBuffCount(void)
* ��	�ܣ���ȡ�������н��յ������ݸ���
* ��ڲ�������
* ���ڲ�������
* ˵	������					  
****************************************************************************/
u32 LookUSART2_GetBuffCount(void)
{
	return USART2_Real.QDataCount;
}

/****************************************************************************
* ��	�ƣ�u8 USART2_GetByte(u8* Data)	
* ��	�ܣ�����������ȡ���ַ�����
* ��ڲ�����u8* Data ���յ�������
* ���ڲ�����u8	���ճɹ���־
			0 	���ճɹ�
			1 	����ʧ��
* ˵	�����ӽ��ջ������л�ȡ���ݣ���������ȴ�	 
***************************************************************************/
u8 USART2_GetByte(u8* Data)
{
	if(USART2_Real.QDataCount!=0)	//��δ�������ݴ���
	{
		*Data=*USART2_Real.QOut;	//�Ӷ�����ȡ����
		USART2_Real.QOut++;			//ָ����һ��δ�������ݵ�
		if(USART2_Real.QOut>USART2_Real.QEnd)		//�������һ���жϵ������ؿ�ʼ��
			USART2_Real.QOut=USART2_Real.QStart;	
		USART2_Real.QDataCount--;	//δ�������ݸ����ݼ�

		return 1;
	}
	return 0;
}

/****************************************************************************
* ��	�ƣ�void USART2_SendByte(u8 Data)
* ��	�ܣ����ַ�����
* ��ڲ�����Data 	���͵��ַ�����
* ���ڲ�������
* ˵	������				   
****************************************************************************/
void USART2_SendByte(u8 Data)		   //���ַ��������
{
	USART_SendData(USART2, Data);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/****************************************************************************
* ��	�ƣ�void USART2_SendString(u8* Data,u32 Len)
* ��	�ܣ����ַ����
* ��ڲ�����Data 	����ĵ��ַ�����
			Len		�ַ�����
* ���ڲ�������
* ˵	������					 
****************************************************************************/
void USART2_SendString(u8* Data,u32 Len)		   //���ַ����
{
	u32 i=0;
	for(i=0;i<Len;i++)
    {    
		USART_SendData(USART2, Data[i]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}
}

void Usart2_temp_buffer_Clr()
{
	uint16_t i;
	for(i=0;i<256;i++)
	{
		Usart2_temp_buffer[i]=0x00;
	}
}

void USART2_DMAS(u8* Data)
{
	while(*Data)
	{
		USART2_SendByte(*Data++);
	}
}

int USART2_DMASS(u8* Data,uint16_t BeTime,uint16_t AfTime)
{

		int RecLen=0;
		u8 Ubyte=0;
		uint16_t aftime;
		
		Usart2_temp_buffer_Clr();		
		USART2_RX_Buffer_Clear();
		if(Data!=NULL){
			USART2_SendString(Data,strlen(Data));
		}
        
		do{
			BeTime--;
			Delay_MS(1);
		}while(!(LookUSART2_GetBuffCount()) && BeTime>0);
//??????	
		aftime=AfTime;
		do{
			Delay_MS(1);
			while(LookUSART2_GetBuffCount()){
				USART2_GetByte(&Ubyte);
				Usart2_temp_buffer[RecLen]=Ubyte;
				RecLen=RecLen+1;			
				aftime=AfTime;
			}
		
			aftime--;
			
		}while(LookUSART2_GetBuffCount() || aftime>0);//????aftime=0?LookUSART2_GetBuffCount()=1?????
					
		USART2_RX_Buffer_Clear();
						
		return RecLen;	
		
}

/****************************************************************************
* ��	�ƣ�void USART2_IRQHandler(void)	
* ��	�ܣ��жϻ���
* ��ڲ�������
* ���ڲ�������
* ˵	�������յ������ݴ�����ջ�����
	USART_GetITStatus		���ָ����USART�жϷ������
	USART_GetFlagStatus	���ָ����USART��־λ�������
****************************************************************************/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)				// ���ڽ������ݴ����ж�
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);				//��ս����жϱ�־
		USART2_GetChar = USART_ReceiveData(USART2);					//���յ����ַ�����
//		USART1_SendByte(USART2_GetChar);
		if(USART2_Real.QDataCount < USART2_RX_BUFF_SIZEMAX)			//����ռ�δ��,���ձ������� 
		{
			if(USART2_Real.QIn > USART2_Real.QEnd)					//����пռ�ɴ����ݣ�������λ����ĩβλ��
				USART2_Real.QIn = USART2_Real.QStart;				//������λ��������ʼλ��
			*USART2_Real.QIn= USART2_GetChar;						//���յ����ַ����ݴ��뻺��������λ��
			USART2_Real.QIn++;										//����λ������
			USART2_Real.QDataCount++;								//�ռ������ݸ�������
		}
	}

	else if(USART_GetFlagStatus(USART2, USART_IT_ORE) == SET)		//����Ƿ��н������
    {
	    USART_ReceiveData(USART2);									//����������־��ֻ���ö����ݵķ�ʽ���������־
    }

	else if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)				//���ڷ������ݴ����ж�
	{
		;
	}
}
