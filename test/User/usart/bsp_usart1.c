#include "USART1_init.h"
#include "TIMER.h"
#include "bsp_TiMbase.h" 
#include "bsp_led.h"

typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;
/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBRcvState eRcvState;

volatile uint8_t  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

static volatile uint8_t *pucSndBufferCur;
static volatile unsigned short usSndBufferCount;

static volatile uint8_t usRcvBufferPos;

static volatile uint8_t eRcvStateError = 0;

extern uint16_t TM_SEND;
extern uint16_t T_SEND;

/****************************************************************************
* 名	称：void USART1_GPIO_Init(void)
* 功	能：串口引脚初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART1_GPIO_Init(void)						//串口引脚初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;		//串口引脚结构
	
	//串口引脚分配时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//配置串口 Tx (PA.09) 为复用推挽输出
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					//串口发送引脚
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//复用推挽输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//频率50MHz
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//初始化引脚
    
	// 配置串口 Rx (PA.10) 为浮空输入
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;					//串口接收引脚
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);						//初始化引脚
}

/****************************************************************************
* 名	称：void USART1_Init(void)
* 功	能：串口初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART1_Init(void)
{		
	USART_InitTypeDef UART_InitStructure;		//串口结构

	//串口分配时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//串口初始化
	UART_InitStructure.USART_BaudRate            = USART1_BAUD_RATE;			//波特率
	UART_InitStructure.USART_WordLength          = USART_WordLength_8b;				//数据位8bit
	UART_InitStructure.USART_StopBits            = USART_StopBits_1;				//停止位个数
	UART_InitStructure.USART_Parity              = USART_Parity_No ;				//不进行奇偶效验
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//RTS和CTS使能(None不使用)
	UART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	//发送和接收使能
	USART_Init(USART1, &UART_InitStructure);										//初始化串口
}

/****************************************************************************
* 名	称：void USART1_NVIC_Init(void)
* 功	能：串口中断向量表初始化
* 入口参数：无
* 出口参数：无
* 说	明：无
****************************************************************************/
void USART1_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 						//中断控制器变量
	
	/* 选择优先级分组1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//设置中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//主优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//设置优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//打开串口中断
	NVIC_Init(&NVIC_InitStructure);								//初始化中断向量表
}

/****************************************************************************
* 名	称：void USART1_Config(void)
* 功	能：串口设置
* 入口参数：无
* 出口参数：无
* 说	明：默认为包数据接收					 
****************************************************************************/
void USART1_Config(void)
{
	USART1_Init();				//串口初始化
	USART1_GPIO_Init();			//串口引脚初始化
	USART1_NVIC_Init();			//中断初始化

	USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//清接收标志
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//开启接收中断
	
	USART_Cmd(USART1, ENABLE);  								//使能失能串口外设	
}

void
PortSerialGetByte( uint8_t * pucByte )
{
    *pucByte = USART_ReceiveData(USART1);
}

void
ReceiveFSM( void )
{
    uint8_t           ucByte;

    /* Always read the character. */
    ( void )PortSerialGetByte( ( uint8_t * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        PortTimersEnable(  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        PortTimersEnable(  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
        eRcvStateError = 0;
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        PortTimersEnable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
            eRcvStateError = 1;
        }
        PortTimersEnable(  );
        break;
    }
}

/****************************************************************************
* 名	称：void USART1_IRQHandler(void)	
* 功	能：中断机制
* 入口参数：无
* 出口参数：无
* 说	明：接收到的数据存入接收缓冲区
	 USART_GetITStatus		检查指定的USART中断发生与否
	 USART_GetFlagStatus	检查指定的USART标志位设置与否
****************************************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)				// 串口接收数据触发中断
	{
        ReceiveFSM( );
        /*
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//清空接收中断标志
		USART1_GetChar = USART_ReceiveData(USART1);					//接收到的字符数据
		
		if(USART1_Real.QDataCount < USART1_RX_BUFF_SIZEMAX)			//如果空间未满,接收保存数据 
		{
			if(USART1_Real.QIn > USART1_Real.QEnd)					//如果有空间可存数据，但输入位置在末尾位置
				USART1_Real.QIn = USART1_Real.QStart;				//将输入位置跳到开始位置
			*USART1_Real.QIn= USART1_GetChar;						//接收到的字符数据存入缓冲区输入位置
			USART1_Real.QIn++;										//输入位置增加
			USART1_Real.QDataCount++;								//空间里数据个数增加
		}*/
	}

	else if(USART_GetFlagStatus(USART1, USART_IT_ORE) == SET)		//检测是否有接收溢出
    {
	    USART_ReceiveData(USART1);									//清接收溢出标志，只能用读数据的方式来清溢出标志
    }

	else if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)				//串口发送数据触发中断
	{
		;
	}
}

void
TimerT35Expired( void )
{
    PortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;
}
