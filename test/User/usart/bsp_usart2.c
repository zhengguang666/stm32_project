
#include "USART2_init.h"
#include "USART1_init.h"

//���ջ���������
#define USART2_RX_BUFF_SIZEMAX 256
#define USART2_TX_BUFF_SIZEMAX 256

//�ڲ��ñ���
static u8 USART2_GetChar=0;							//���յ��ĵ����ַ�				�ж���ʹ��					
static u8 USART2RxBuffer[USART2_RX_BUFF_SIZEMAX];	//���ڽ��ջ����� 
static u8 USART2TxBuffer[USART2_TX_BUFF_SIZEMAX];	//���ڷ��ͻ����� 
static void Delay_MS(u32 nCount_temp);

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
* ��	�ƣ�void USART2_DMATxd_Init(void)
* ��	�ܣ�����DMA��ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void USART2_DMATxd_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 		//�жϿ���������
	DMA_InitTypeDef DMA_InitStructure;			//DMA�ṹ

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);			//ʹ��DMA1ʱ��


	/* ѡ�����ȼ�����1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//DMA�ж���������
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;	//����DMA�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//�����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//���ж�
	NVIC_Init(&NVIC_InitStructure); 

	//DMA����
	DMA_DeInit(DMA1_Channel7);  		   									//��λDMA1_Channel4ͨ��ΪĬ��ֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_BASE + 4;				//DMAͨ���������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2TxBuffer;				//DMAͨ���洢������ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMAĿ�ĵ�	(DMA_DIR_PeripheralSRCԴ)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//��ǰ����Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//��ǰ�洢�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݿ��Ϊ�ֽ�(8λ)
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݿ��Ϊ�ֽ�(8λ)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//��������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					//DMAͨ�����ȼ��ǳ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//DMAͨ��δ���ô洢�����洢������
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);							//�����������ó�ʼ��DMA
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);    						//����DMAͨ���ж�
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
* ��	�ƣ�u8 USART2_GetByte_WaitTime(u8* Data,u32 TimeLater)	
* ��	�ܣ�u8* Data ���յ�������
			u32 TimeLater	�ȴ�ʱ�� 7=1us
* ��ڲ�����u8	�Ƿ��н��յ�����
				0 û�н��յ�����
				1 ���յ�����
* ���ڲ��������յ��ĵ��ַ�����
* ˵	�����ӽ��ջ������л�ȡ���ݣ�ʱ���ȴ�
***************************************************************************/
u8 USART2_GetByte_WaitTime(u8* Data,u32 TimeLater)
{
	u8 flag=0;					//��ȡ��־
	u32 err=0;					//�����־
	u8 val=0;					//��ȡ�����ݴ�ŵ�
	while(1)
	{
		flag=USART2_GetByte(&val);
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
* ��	�ƣ�u8 USART2_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater)	
* ��	�ܣ�����ʱ֮��ʼ����ָ���������µ�����
* ��ڲ�����u8* Data ���յ�������
			u32 DataLen		ϣ�����յ����ݸ���
			u32 TimeLater	���յȴ�ʱ�� 7=1us
* ���ڲ�����u32 ���յ����ݸ���	0û�н��յ�����
* ˵	������
***************************************************************************/
u32 USART2_WaitTime_GetString(u8* Data,u32 DataLen,u32 TimeLater)
{
	u8 G = 0;					//��õ�ֵ
	u32 i = 0;					//���յĸ���

	Delay_MS(TimeLater);	 	//���յȴ�

	if(LookUSART2_GetBuffCount() == 0)			//û�н��յ�����
		return 0;

	while(LookUSART2_GetBuffCount() != 0)		//�鿴���������ݽ��ո���
	{
		if(i >= DataLen - 1)		//����ϣ�����յĴ�С
			break;					//�˳�����

		if(USART2_GetByte(&G) == 1)		//ȡֵ�ɹ�
		{
			Data[i] = G;			//���ݻ���
			i++;
			//DebugPf("%c",G);		//�ն���ʾ
		}
	}
	return i;
}

/****************************************************************************
* ��	�ƣ�u8 USART2_GetByte_WaitLong(void)	
* ��	�ܣ���
* ��ڲ�����u8	���յ�������
* ���ڲ��������յ��ĵ��ַ�����
* ˵	�����ӽ��ջ������л�ȡ���ݣ������ȴ�
***************************************************************************/
u8 USART2_GetByte_WaitLong(void)
{
	u8 flag=0;					//��ȡ��־
	u8 val=0;					//��ȡ�����ݴ�ŵ�
	while(1)
	{
		flag=USART2_GetByte(&val);
		if (flag==1)
			return val;
	}
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

/****************************************************************************
* ��	�ƣ�void USART2_DMASendString(u8* Data,u32 Len)
* ��	�ܣ�DMA��ʽ���ַ����
* ��ڲ�����Data 	����ĵ��ַ�����
			Len		�ַ�����
* ���ڲ�������
* ˵	����������USART2_DMATxd_Init��ʼ��֮�����ʹ��
			DMA����CPU���� �� ��CPU���� ���з��ͳ�ͻ	 
****************************************************************************/
void USART2_DMASendString(u8* Data,u32 Len)		   //���ַ����
{
	memcpy(USART2TxBuffer, Data, Len);			   //�������ݵ����ͻ�����
    DMA1_Channel7->CNDTR = Len;					   //�����ֽ�����
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); //����
	DMA_Cmd(DMA1_Channel7, ENABLE);				   //ʼ��DMAͨ��
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

/****************************************************************************
* ��	�ƣ�void DMA1_Channel7_IRQHandler(void)	
* ��	�ܣ�DMA�жϻ���
* ��ڲ�������
* ���ڲ�������
* ˵	������
****************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7)) //����������
	{
		USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);    //�ر�DMA����
		DMA_Cmd(DMA1_Channel7, DISABLE);	       			//�ر�DMAͨ��  
	}

	DMA_ClearFlag(DMA1_FLAG_GL7| DMA1_FLAG_TC7 | DMA1_FLAG_HT7 | DMA1_FLAG_TE7);  //���DMA��ر�־
}

/****************************************************************************
* ��	�ƣ�USART2_STRUCT* USART2GetDCB(void)
* ��	�ܣ���ȡ����
* ��ڲ�������
* ���ڲ��������ýṹ��
* ˵	������
****************************************************************************/
USART2_STRUCT* USART2GetDCB(void)
{
	return ((USART2_STRUCT*)(&USART2_Real));
}

/*	���ڲ���
	u8 c[10]="���";
	u8 l='a';
	u8 v=0;
	u8 f=0;
	u8 len=0;

	DebugPf("��ʼ\r\n");				 //printf����
	USART2_DMASendString(c,10);		 //DMA���Ͳ���
	USART2_SendString(c,10);		 //���ַ����Ͳ���
	USART2_DMASendString(&l,1);		 //DMA���͵��ַ�����

	f=1;
	while(f)						 //��������
	{
		;	
	}
	len = LookUSART2_GetBuffCount(); //�鿴���ո�������
	DebugPf("%d\r\n",len);			//printf����
	len =0;

	f=USART2_GetByte(&v);			 //ȡһ�ֽڲ���
	if(f==1)
		USART2_SendByte(v);
	f=0;

	USART2_RX_Buffer_Clear();		 //��ջ���������

	len=1;
	while(len)
	{
		f=USART2_GetByte_WaitTime(&v,7000000);		 //ʱ���ȴ�����
		if(f==1)
			USART2_SendByte(v);
		f=0;

		v=USART2_GetByte_WaitLong();				 //һֱ�ȴ�����
		USART2_SendByte(v);
	}
	
	len = LookUSART2_GetBuffMax();					 //��������С�鿴����
	DebugPf("%d\r\n",len);			//printf����
*/


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
