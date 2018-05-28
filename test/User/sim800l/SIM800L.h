

#include "stm32f10x.h"
#include "static_init.h"	//���ڽṹ��


/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */



/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬







uint8_t Sim_ini(void);

unsigned char GPSVLD();
void GPSDATA(void);
void SIM_GPIO_Config(void);
uint8_t GetKeyCHG();
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin,uint8_t Down_state);
void USART2_DMAS(u8* Data);
void CBCDATA();
int USART2_DMASS(u8* Data,uint16_t BeTime,uint16_t AfTime);
void PWR_ON();
void GetSpeed();
uint8_t GetKeySTA();
uint8_t GetKeyPWR();
void SIM_RST();
uint8_t GPSINI();
uint8_t USART2_RD(uint16_t bTime,uint16_t aTime);
uint8_t Creg_CK();
uint8_t PWR_DOWN();
uint8_t TCP_SEND();
uint8_t GetGPS();
uint8_t TCP_GET();
void GET_CBC();
void PWR_OFF();
uint8_t SendGPS();	
uint8_t GetKeyCHG();
uint8_t Connect_Server(void);

