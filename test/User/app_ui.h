#ifndef __APP_UI_H
#define __APP_UI_H

#include "stm32f10x.h"

//#define LCD_DISPLAY
//#define WORD_MODE		//����ģʽ
/* ʵ�����״̬��־   */
typedef enum {SB_NULL,SB_RUN,SB_MP3,SB_FM}TESTING_STATE;
//׼�������š���ͣ���и�


/* �����¼���־ */
typedef enum {EB_NULL,EB_SELT_PREV,EB_SELT_NEXT,EB_SELECT,EB_RESET}TOUCH_EVEN_BOARDTEST;
//���¼���������ͣ�¼�����һ�ס���һ�ס�����+����������ֱ��ѡ�����������ѡ������ѡ����һҳ����һҳ��ѡ��OK��ѡ����������


//void Lcd_List(char page);
void even_process_boardtest(void);
//void Lcd_Note(uint8_t test_num);


#endif /* __APP_UI_H */

