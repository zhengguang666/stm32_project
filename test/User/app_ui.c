#include <stdio.h>
#include <string.h>
#include "app_ui.h"

#include "ff.h"
#include "diskio.h"
#include "bsp_led.h"
#include "bsp_ili9341_lcd.h"
#include "bsp_bmp.h"
#include "bsp_touch.h"

#define TEST_NUM			11
#ifdef WORD_MODE
#define PER_PAGE			8					//ÿҳ��ʾ����Ŀ	

#else

#define PER_PAGE			6					//ÿҳ��ʾ����Ŀ	
#endif

uint8_t current_page_boardtest=1;
uint8_t all_page_boardtest = (TEST_NUM+7)/8 ;		//ÿҳ��ʾ8��ʵ��,+7����Ϊʹ�ý�һ��


uint8_t select_index_boardtest=0;								//ѡ������	
//uint8_t testing_index=0;							//���ڲ��Եĳ�������

TESTING_STATE testing_state=SB_NULL;			//������״̬��־λ��ָʾ�и裬���ڲ��ţ���ͣ��״̬
TOUCH_EVEN_BOARDTEST touch_even_boardtest;					//����״̬��־λ��ָʾ����������������һ����ť��������+-��

// iso �������
extern volatile uint8_t rec_cmd;
extern void Soft_Reset(void);

static void delay(uint16_t t)
{
	uint8_t i;
	while(t--)
	{
		for(i=0;i<250;i++);
	}
}

/*ʵ�����֣�������ʾ*/
uint8_t test_name[TEST_NUM][24]={
	"��ˮ�Ʋ���",												//1
	"��������",													//2
	"EEPROM����",												//3
	"RTC������",						  			//4
	"DHT11��ʪ��",									//5
	"DS18b20�¶ȴ�����",						//6
	"����ͷ(����ov7725)",						//7
	"CAN(Loopback)",								//8
	"�����Ʋ���",										//9
	"��̫��ENC28J60",								//10
	"Usb-Mass-Storage����"					//11

};


//uint8_t test_note[TEST_NUM][250]={
//	"LED���Գ���������:��������: LED1 LE2 LED3 ������˸",
//	"�������Գ���������:��������:KEY1����LED1��ת,KEY2����LE2��ת",
//	"EEPROM���Գ���������",
//	"FLASH���Գ���������",
//	"ʵ������:Һ������ʮ�ֽ��棬�û��ʵ��ʮ�ֽ�����д�����У��",
//	"",
//	
//	
//};


/**
  * @brief  lcd_list ��ʾʵ���б�
  * @param  ʵ���б��ҳ��
  * @retval none
  */

 void Lcd_List(char page)
{	

#ifdef WORD_MODE				//���ְ�
	char i;
	char lcd_char[28];												//+4��Ϊ��Ԥ���ռ��ʵ����
	
	
	select_index_boardtest=0;														//������ʾ�µ�һҳʱ����select_index
	Lcd_GramScan( 1 );												//����ɨ�跽��
	LCD_Clear(12,86,206,145,BACKGROUND);
	
	
	sprintf(lcd_char,"�� %0d/%0d ҳ",current_page_boardtest,all_page_boardtest);		//��ʾҳ��
	LCD_DispEnCh(30,235,(const uint8_t *)lcd_char,BLUE);
		
	/* ��playlist����ȡ��Ƶ�ļ��� */
	for(i=0;(i+PER_PAGE*(page-1))<TEST_NUM && i< PER_PAGE;i++)								//����ʾ��һҳ
	{	
	
		sprintf(lcd_char,"%0d.%s",PER_PAGE*(page-1)+i+1,test_name[(PER_PAGE*(page-1) + i)]);			//ƴ�ӳ������б�
		
		LCD_DispEnCh(22,87+i*18,(const uint8_t *)lcd_char,BLACK);										//��ʾ

	}	
	
#else			//ͼ���
	char i;
	char lcd_char[28];												//+4��Ϊ��Ԥ���ռ��ʵ����
	char pic_name[50];
	
	select_index_boardtest=0;														//������ʾ�µ�һҳʱ����select_index
	Lcd_GramScan( 1 );												//����ɨ�跽��
	LCD_Clear(12,94,212,160,BACKGROUND);//BLUE BACKGROUND
	
	
	sprintf(lcd_char,"�� %0d/%0d ҳ",current_page_boardtest,all_page_boardtest);		//��ʾҳ��
	LCD_DispEnCh(30,235,(const uint8_t *)lcd_char,BLUE);
		
	/* ��playlist����ȡ��Ƶ�ļ��� */
	for(i=0;(i+PER_PAGE*(page-1))<TEST_NUM && i< PER_PAGE;i++)								//����ʾ��һҳ
	{		
		sprintf(pic_name,"/boardtest/ISO_MINI/ui_test%d.bmp",PER_PAGE*(page-1)+i+1);

		if(i<=((PER_PAGE/2)-1))
		{
				Lcd_show_bmp(160, 166-(70*i),pic_name);
		}
		else
		{
				Lcd_show_bmp(91, 166-(70*(i-(PER_PAGE/2))),pic_name);

			}
	}		
	
#endif

}



/*����ʱ��ʾ����Ч��*/
void Lcd_Touch(uint8_t test_num)
{
	char i = ((test_num-1)%PER_PAGE);
	char pic_name[50];	
	
		sprintf(pic_name,"/boardtest/ISO_MINI/ui_tch_test%d.bmp",test_num);

		if(i<=((PER_PAGE/2)-1))
		{
			Lcd_show_bmp(160, 166-(70*i),pic_name);				
		}
		else
		{
			Lcd_show_bmp(91, 166-(70*(i-(PER_PAGE/2))),pic_name);
		}
		
		
		delay(0x2FFf);
		sprintf(pic_name,"/boardtest/ISO_MINI/ui_test%d.bmp",test_num);
		
		if(i<=((PER_PAGE/2)-1))
		{
			Lcd_show_bmp(160, 166-(70*i),pic_name);				
		}
		else
		{
			Lcd_show_bmp(91, 166-(70*(i-(PER_PAGE/2))),pic_name);
		}
		delay(0xFFF);

	


}


void Lcd_Note(uint8_t test_num)
{
	uint8_t lcd_char[50];
	Lcd_GramScan( 1 );												//����ɨ�跽��
	LCD_Clear(12,86,206,165,BACKGROUND);			//�����Ļ
	
//	LCD_Clear(12,72,215,152,BACKGROUND);							//RED	 ���				
	//LCD_DispEnCh(50,90,test_name[test_num-1],RED);	
	
	sprintf(lcd_char,"%s����������:",test_name[test_num-1]);

	LCD_DispEnCh(15,90,lcd_char,BLACK);		
	
	
	LCD_DispEnCh(10,230,"�����Ӳ����λ����ѡ�����",BLACK);	
	
	



}

/**
  * @brief  touch_process �����¼������ڴ����ж�ʱ������
  * @param  none
  * @retval none
  */
void touch_process_boardtest (void)
{
	
	  if(touch_flag == 1)			/*������ʰ�����*/
    {
		
      /* ��ȡ������� */
      if(Get_touch_point(&display, Read_2046_2(), &touch_para ) !=DISABLE)      
      {		
        LED1_TOGGLE;   																//LED��ת	    
				printf("\r\nx=%d,y=%d",display.x,display.y);
				//��������x��Χ���ٸ���y�������ְ���
				if(display.x>=6 && display.x<=38)							
				{
						if(display.y >=13 &&display.y<=35)		//��һҳ
						{
							touch_even_boardtest = EB_SELT_PREV;									
							printf("\r\n ��һҳ");
						}
						else if(display.y >=48 &&display.y<=80)	//��һҳ
						{										
							touch_even_boardtest = EB_SELT_NEXT;	
							printf("\r\n ��һҳ");
						}
						else if(display.y >=180 &&display.y<=218)	//��λ
						{
							touch_even_boardtest = EB_RESET;
							printf("\r\n ��λ");
						}
							
				}
				
		#ifdef WORD_MODE	//����ģʽ
					
				//ֱ��ѡ��ʵ�飬��1.2.3.4.������8~
					
				else if(display.x>208 && display.x<226)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 1;
					printf("\r\n rec_cmd=%d",select_index_boardtest);
				}
				else if(display.x>190 && display.x<208)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 2;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>172 && display.x<190)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 3;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>154 && display.x<172)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 4;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>136 && display.x<154)
				{
					touch_even_boardtest = EB_SELECT;
					select_index_boardtest = 5;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>118 && display.x<136)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 6;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>100 && display.x<118)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 7;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
				else if(display.x>82 && display.x<100)
				{
					touch_even_boardtest = EB_SELECT;	
					select_index_boardtest = 8;
					printf("\r\n select_index_boardtest=%d",select_index_boardtest);
				}
		#else			//ͼ��ģʽ
				
				else if(display.x>=140 && display.x<=195)			//��һ��ͼ��					
				{
						if(display.y>17 && display.y<70)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 1;
							printf("\r\n rec_cmd=%d",select_index_boardtest);
						}
						else if(display.y>86 && display.y<144)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 2;
							printf("\r\n select_index_boardtest=%d",select_index_boardtest);
						}
						else if(display.y>163 && display.y<210)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 3;
							printf("\r\n select_index_boardtest=%d",select_index_boardtest);
						}

				}
					
				else if(display.x>=70 && display.x<=125)			//��һ��ͼ��					
				{
						if(display.y>17 && display.y<70)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 4;
							printf("\r\n select_index_boardtest=%d",select_index_boardtest);
						}
						else if(display.y>86 && display.y<144)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 5;
							printf("\r\n select_index_boardtest=%d",select_index_boardtest);
						}
						else if(display.y>163 && display.y<210)
						{
							touch_even_boardtest = EB_SELECT;	
							select_index_boardtest = 6;
							printf("\r\n select_index_boardtest=%d",select_index_boardtest);
						}

				}							

				
		#endif
						
      }//if(Get_touch_point(&display, Read_2046_2(), &touch_para ) !=DISABLE)      
			
    }// if(touch_flag == 1)			
	
}


/**
  * @brief  even_process �����¼���־���д���
  * @param  none
  * @retval none
  */
void even_process_boardtest(void)
{
	static uint8_t flag = 0;
	switch(touch_even_boardtest)
	{
		
		/* ���š���ͣ��	*/
		case EB_RESET:												
		
			Soft_Reset();
				
			touch_even_boardtest = EB_NULL;
			break;	
				
		/* ֱ�ӵ�ѡ����	*/
		case EB_SELECT:												
			
			rec_cmd = select_index_boardtest + ((current_page_boardtest-1)*PER_PAGE);											//���ݵ�ǰҳ��select_index_boardtestȷ��rec_cmd
			if(flag == 0)
			{
				Lcd_Touch(rec_cmd);
				flag = 1;
			}
				testing_state = SB_RUN;
		
			touch_even_boardtest = EB_NULL;	
		
			break;			
		
		/* ��һҳ				*/
		case EB_SELT_NEXT:		
			if(testing_state == SB_RUN)
				Soft_Reset();
			
			if(current_page_boardtest<all_page_boardtest)
			{
				if(current_page_boardtest<all_page_boardtest)
					current_page_boardtest++;										//���µ�ǰҳ��
				Lcd_List(current_page_boardtest);						//ˢ��LCD�б�
			}
			else
				current_page_boardtest =all_page_boardtest;
		
			touch_even_boardtest = EB_NULL;
			
			break;
		
		/* ��һҳ			*/
		case EB_SELT_PREV:	
			if(testing_state == SB_RUN)
				Soft_Reset();
		
			if(current_page_boardtest>1)					
			{
				if(current_page_boardtest>0)
					current_page_boardtest--;										//���µ�ǰҳ��
				Lcd_List(current_page_boardtest);						//ˢ��LCD�б�
			}
			else
				current_page_boardtest =1;
			
			touch_even_boardtest = EB_NULL;
			break;		
		
		default:
			//touch_even_boardtest = EB_NULL;
			break;
	}

}


