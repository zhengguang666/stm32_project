#ifndef __MP3_H
#define	__MP3_H

#include "stm32f10x.h"


/*
��������ϵͳ��ת����


��������
				column 240
			 
			 x
        _ _ _ _ _ _
       A           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |  320        
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
	(0,0)- - - - - ->  y       
				
Һ����ʾ���弰�����Ⱥ�����
				
				column 240
			 
										x
  (0,0) ----------->
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |  320        
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 V- - - - - - 

       y
			 
Һ����ʾͼ��
				
				column 240		 
									 X	
				-----------A
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |  320        
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
			 |           |
		 Y <- - - - - -(0,0) 

       

ת����ϵ��
Y�� = 320 - X����
X�� = Y����

Xͼ�� = X����
Yͼ�� = 240-Y����

Yͼ�� = 240 - X��
Xͼ�� = 320 - Y��




*/



/* WAV�ļ�ͷ����ʽ */
typedef __packed struct
{ 
	uint8_t		riff[4];					/* = "RIFF"							       				*/
	uint32_t	size_8;						/* ���¸���ַ��ʼ���ļ�β�����ֽ���		*/
	uint8_t		wave[4];					/* = "WAVE" 							   					*/
	uint8_t		fmt[4];						/* = "fmt " 							   					*/
	uint32_t	fmtSize;					/* ��һ���ṹ��Ĵ�С(һ��Ϊ16)			  */
	 
	uint16_t	wFormatTag;				/* ���뷽ʽ,һ��Ϊ1						   			*/
	uint16_t	wChannels;				/* ͨ������������Ϊ1��������Ϊ2			  */
	uint32_t	dwSamplesPerSec;	/* ������								   						*/
	uint32_t	dwAvgBytesPerSec;	/* ÿ���ֽ���(= ������ �� ÿ���������ֽ���) */
	uint16_t	wBlockAlign;			/* ÿ���������ֽ���(=����������/8*ͨ����)  	*/
	uint16_t	wBitsPerSample;		/* ����������(ÿ��������Ҫ��bit��) 		   		*/
																			   
	uint8_t		data[4];					/* = "data"; 							   					*/
	uint32_t	datasize;					/* �����ݳ��� 							   				*/
} WavHead;


/* ������״̬��־   */
typedef enum {S_READY,S_PLAY,S_STOP,S_SWITCH}PLAYER_STATE;
//׼�������š���ͣ���и�


/* �����¼���־ */
typedef enum {E_NULL,E_PLAY_STOP,E_PREV,E_NEXT,E_UP,E_DOWN,E_SELECT,E_SELT_UP,E_SELT_DOWN,E_SELT_PREV,E_SELT_NEXT,E_SELT_OK,E_LOUD_SPEAK}TOUCH_EVEN;
//���¼���������ͣ�¼�����һ�ס���һ�ס�����+����������ֱ��ѡ�����������ѡ������ѡ����һҳ����һҳ��ѡ��OK��ѡ����������


void MP3_test(void);
void player_run(void);
void touch_process (void);	/* �жϷ����� */							

#endif


