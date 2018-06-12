#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
	* 1-修改总线时钟的宏，uart1挂载到apb2总线，其他uart挂载到apb1总线
	* 2-修改GPIO的宏
  */
	
// 串口1-USART1
#define  WEIGHT_USARTx                   USART1
#define  WEIGHT_USART_CLK                RCC_APB2Periph_USART1
#define  WEIGHT_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  WEIGHT_USART_BAUDRATE           115200

// USART GPIO 引脚宏定义
#define  WEIGHT_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  WEIGHT_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  WEIGHT_USART_TX_GPIO_PORT       GPIOA   
#define  WEIGHT_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  WEIGHT_USART_RX_GPIO_PORT       GPIOA
#define  WEIGHT_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  WEIGHT_USART_IRQ                USART1_IRQn
#define  WEIGHT_USART_IRQHandler         USART1_IRQHandler

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

struct customer{
char type; 
char *unit_price; 
char *Block; 
char *weight; 
float total_price; 
};

static volatile eMBRcvState eRcvState;

void USART_Config(void);
void PortSerialGetByte( uint8_t * pucByte );
void ReceiveFSM( void );
void TimerT35Expired( void );
unsigned char * Get_Weight(void);

#endif /* __USART_H */
