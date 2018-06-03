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
	UART_InitStructure.USART_BaudRate            = USART1_BAUD_RATE;			//������
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

	USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//����ձ�־
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);				//���������ж�
	
	USART_Cmd(USART1, ENABLE);  								//ʹ��ʧ�ܴ�������	
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
        ReceiveFSM( );
        /*
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);				//��ս����жϱ�־
		USART1_GetChar = USART_ReceiveData(USART1);					//���յ����ַ�����
		
		if(USART1_Real.QDataCount < USART1_RX_BUFF_SIZEMAX)			//����ռ�δ��,���ձ������� 
		{
			if(USART1_Real.QIn > USART1_Real.QEnd)					//����пռ�ɴ����ݣ�������λ����ĩβλ��
				USART1_Real.QIn = USART1_Real.QStart;				//������λ��������ʼλ��
			*USART1_Real.QIn= USART1_GetChar;						//���յ����ַ����ݴ��뻺��������λ��
			USART1_Real.QIn++;										//����λ������
			USART1_Real.QDataCount++;								//�ռ������ݸ�������
		}*/
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

void
TimerT35Expired( void )
{
    PortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;
}
