
#include "bsp_usart2.h"

u8 Usart2_temp_buffer[256];

//接收缓冲区容量
#define USART2_RX_BUFF_SIZEMAX 256
#define USART2_TX_BUFF_SIZEMAX 256

//内部用变量
static u8 USART2_GetChar=0;							//接收到的单个字符				中断中使用					
static u8 USART2RxBuffer[USART2_RX_BUFF_SIZEMAX];	//串口接收缓存区 

//串口配置变量
USART2_STRUCT USART2_Real = 
{
	115200,				//波特率
	
	//接收控制队列
	((void*)0),			//开始位置
	((void*)0),			//结束位置
	((void*)0),			//输入位置
	((void*)0),			//输出位置
	((void*)0),			//空间指针
	0,					//空间里数据个数
};

/****************************************************************************
* 名	称：void USART2_GPIO_Init(void)
* 功	能：串口引脚初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART2_GPIO_Init(void)			//串口引脚初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;		//串口引脚结构
	
	//串口引脚分配时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//配置串口 Tx (PA.02) 为复用推挽输出
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;					//串口发送引脚
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//复用推挽输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//频率50MHz
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//初始化引脚
    
	// 配置串口 Rx (PA.03) 为浮空输入
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;					//串口接收引脚
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//初始化引脚
}

/****************************************************************************
* 名	称：void USART2_Init(void)
* 功	能：串口初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART2_Init(void)
{		
	USART_InitTypeDef UART_InitStructure;		//串口结构

	//串口分配时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//串口初始化
	UART_InitStructure.USART_BaudRate            = USART2_Real.BaudRate;	//波特率
	UART_InitStructure.USART_WordLength          = USART_WordLength_8b;		//数据位8bit
	UART_InitStructure.USART_StopBits            = USART_StopBits_1;		//停止位个数
	UART_InitStructure.USART_Parity              = USART_Parity_No ;		//不进行奇偶效验
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//RTS和CTS使能(None不使用)
	UART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	//发送和接收使能
	USART_Init(USART2, &UART_InitStructure);	//初始化串口
}

/****************************************************************************
* 名	称：void USART2_NVIC_Init(void)
* 功	能：串口中断向量表初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART2_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 		//中断控制器变量
	
	/* 选择优先级分组1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//设置中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//主优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//设置优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//打开串口中断
	NVIC_Init(&NVIC_InitStructure);								//初始化中断向量表
}

/****************************************************************************
* 名	称：void USART2_RX_Buffer_init(void)
* 功	能：串口接收初始化
* 入口参数：无
* 出口参数：无
* 说	明：接收中断与接收缓冲区绑定
****************************************************************************/
void USART2_RX_Buffer_init(void)
{
	USART2_Real.QStart = USART2RxBuffer;							//开始位置
	USART2_Real.QEnd = &USART2RxBuffer[USART2_RX_BUFF_SIZEMAX - 1];	//结束位置
	USART2_Real.QIn = USART2_Real.QStart;							//输入位置=开始位置
	USART2_Real.QOut = USART2_Real.QStart;							//输出位置=开始位置
	USART2_Real.QDataBuf = USART2RxBuffer;							//指定接收缓冲区空间
	USART2_Real.QDataCount = 0;										//空间里数据个数
}

/****************************************************************************
* 名	称：void USART2_Config(void)
* 功	能：串口设置
* 入口参数：无
* 出口参数：无
* 说	明：默认为包数据接收					 
****************************************************************************/
void USART2_Config(void)
{
	USART2_Init();				//串口初始化
	USART2_GPIO_Init();			//串口引脚初始化
	USART2_NVIC_Init();			//中断初始化
//	USART2_DMATxd_Init();		//DMA发送初始化
	USART2_RX_Buffer_init();	//接收中断与接收缓冲区绑定

	USART_ClearITPendingBit(USART2, USART_IT_RXNE);				//清接收标志
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);				//开启接收中断
	
	USART_Cmd(USART2, ENABLE);  								//使能失能串口外设	
}

/****************************************************************************
* 名	称：void USART2_RX_Buffer_Clear(void)	
* 功	能：清空接收缓冲区
* 入口参数：无
* 出口参数：无
* 说	明：无						
****************************************************************************/
void USART2_RX_Buffer_Clear(void)
{
	USART2_Real.QIn=USART2_Real.QStart;							//输入位置=开始位置
	USART2_Real.QOut=USART2_Real.QStart;						//输出位置=开始位置
	USART2_Real.QDataCount=0;									//空间里数据个数
}

/****************************************************************************
* 名	称：u32 LookUSART2_GetBuffMax(void)	
* 功	能：获取接收缓冲区大小
* 入口参数：无
* 出口参数：无
* 说	明：无			  
****************************************************************************/
u32 LookUSART2_GetBuffMax(void)
{
	return (u32)USART2_RX_BUFF_SIZEMAX;
}

/****************************************************************************
* 名	称：u32 LookUSART2_GetBuffCount(void)
* 功	能：获取缓冲区中接收到的数据个数
* 入口参数：无
* 出口参数：无
* 说	明：无					  
****************************************************************************/
u32 LookUSART2_GetBuffCount(void)
{
	return USART2_Real.QDataCount;
}

/****************************************************************************
* 名	称：u8 USART2_GetByte(u8* Data)	
* 功	能：缓冲区中提取单字符数据
* 入口参数：u8* Data 接收到的数据
* 出口参数：u8	接收成功标志
			0 	接收成功
			1 	接收失败
* 说	明：从接收缓冲区中获取数据，不会持续等待	 
***************************************************************************/
u8 USART2_GetByte(u8* Data)
{
	if(USART2_Real.QDataCount!=0)	//有未处理数据存在
	{
		*Data=*USART2_Real.QOut;	//从队列中取数据
		USART2_Real.QOut++;			//指向下一个未处理数据点
		if(USART2_Real.QOut>USART2_Real.QEnd)		//到了最后一个判断点则跳回开始点
			USART2_Real.QOut=USART2_Real.QStart;	
		USART2_Real.QDataCount--;	//未处理数据个数递减

		return 1;
	}
	return 0;
}

/****************************************************************************
* 名	称：void USART2_SendByte(u8 Data)
* 功	能：单字符发送
* 入口参数：Data 	发送单字符数据
* 出口参数：无
* 说	明：无				   
****************************************************************************/
void USART2_SendByte(u8 Data)		   //单字符数据输出
{
	USART_SendData(USART2, Data);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/****************************************************************************
* 名	称：void USART2_SendString(u8* Data,u32 Len)
* 功	能：多字符输出
* 入口参数：Data 	输出的单字符数据
			Len		字符个数
* 出口参数：无
* 说	明：无					 
****************************************************************************/
void USART2_SendString(u8* Data,u32 Len)		   //多字符输出
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
* 名	称：void USART2_IRQHandler(void)	
* 功	能：中断机制
* 入口参数：无
* 出口参数：无
* 说	明：接收到的数据存入接收缓冲区
	USART_GetITStatus		检查指定的USART中断发生与否
	USART_GetFlagStatus	检查指定的USART标志位设置与否
****************************************************************************/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)				// 串口接收数据触发中断
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);				//清空接收中断标志
		USART2_GetChar = USART_ReceiveData(USART2);					//接收到的字符数据
//		USART1_SendByte(USART2_GetChar);
		if(USART2_Real.QDataCount < USART2_RX_BUFF_SIZEMAX)			//如果空间未满,接收保存数据 
		{
			if(USART2_Real.QIn > USART2_Real.QEnd)					//如果有空间可存数据，但输入位置在末尾位置
				USART2_Real.QIn = USART2_Real.QStart;				//将输入位置跳到开始位置
			*USART2_Real.QIn= USART2_GetChar;						//接收到的字符数据存入缓冲区输入位置
			USART2_Real.QIn++;										//输入位置增加
			USART2_Real.QDataCount++;								//空间里数据个数增加
		}
	}

	else if(USART_GetFlagStatus(USART2, USART_IT_ORE) == SET)		//检测是否有接收溢出
    {
	    USART_ReceiveData(USART2);									//清接收溢出标志，只能用读数据的方式来清溢出标志
    }

	else if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)				//串口发送数据触发中断
	{
		;
	}
}
