#include <stdio.h>
#include <string.h>
#include "mp3dec.h"
#include "ff.h"
#include "diskio.h"
#include "bsp_iis.h"
#include "bsp_pcm1770.h"
#include "bsp_led.h"
#include "mp3.h"
#include "bsp_ili9341_lcd.h"
#include "bsp_bmp.h"
#include "bsp_touch.h"

#define FILE_NAME_LEN 	50			//�ļ������ȣ������⵽�ļ�������50 ��������ļ� 
#define MUSIC_NAME_LEN 	24			//LCD��ʾ���ļ�������󳤶�
#define _DF1S	0x81


/* ��־λ */
uint8_t select_index = 0;				//�������ѡ���־
uint8_t speaker_flag = 0;				//ѡ��ʹ���������ı�־��Ұ��logo��
uint16_t play_index = 0;				//�洢���ڻ�׼�����ŵĸ�������

PLAYER_STATE player_state;			//������״̬��־λ��ָʾ�и裬���ڲ��ţ���ͣ��״̬
TOUCH_EVEN touch_even;					//����״̬��־λ��ָʾ����������������һ����ť��������+-��

/* ������ȫ�ֱ��� */
uint32_t  file_num = 0;					//�洢ɨ�赽��MP3+WAV��Ƶ�ļ����� 
char current_page = 1;					//��ǰ������ҳ��
char all_page = 1;							//�ܹ��Ĳ�����ҳ������
extern uint8_t voice;

/* �ļ�ϵͳȫ�ֱ��� */
static FATFS fs;								//�ļ�ϵͳ�����ռ�
static FIL	file;								//�ļ����		
static FRESULT fres;						//�ļ�ϵͳAPI��������ֵ
static unsigned int rw_num;			//�Ѷ�����д���ֽ���

char path[500]="0:";						//ɨ��SD���ĳ�ʼĿ¼��ɨ���ǵݹ麯�����ݹ�Խ�������ȡֵҲҪԽ��(����RAM���� �˴�ȡ�ٵ�)	

/* MP3����ʱ�õ��ı��� */
uint8_t bufflag=0;										//�����л�buffer�ı�־
static MP3FrameInfo		Mp3FrameInfo;		//mP3֡��Ϣ  
static uint8_t  buffer[1024*4]; 			// �ļ�������
static short   outBuf[2][2500];		    // PCM�����壬ʹ������������ 

/* WAV�����õ��ı��� */
								

/* �ڲ��������� */
static void file_scan(char* path);							//ɨ���ļ�������ǵݹ����
static FRESULT scan_files (char* path);					//ɨ���ļ�������ݹ����
static void mp3_player(const char *filename);		//mp3������
static void wav_player(const char *filename);		//wav������	
static void lcd_list(char page);								//��ʾ�赥��LCD
static void even_process(void);									//��������־�¼�


/**
  * @brief  player_run ����mp3���������̣��ڲ���ѭ��
  * @param  none
  * @retval none
  */
void player_run(void)
{
		char music_name[FILE_NAME_LEN];	
		
		/* ע�Ṥ������ԭsd���ֿ�Ҳʹ����f_mount����mount(0)���ļ�ϵͳָ�벻ͬ,
		����mount��0�̷��Ļ�������Ұ�sd��GetGBKCode_from_sd�����е�f_mountע�ᵽ��1	*/		
		f_mount(0, &fs);	
	
		file_scan(path);							//ɨ���ļ�
	
		if(file_num == 0)
		{
			printf("\r\n no mp3 file ! ");
			return; 										//����������
		}			
			
		player_state = S_READY;				//��ʼ��״̬
		touch_even = E_NULL;					//��ʼ���¼���־
		
		
		all_page = (file_num+7)/8 ;		//ÿҳ��ʾ8���ļ�,+7����Ϊʹ�ý�һ��
		current_page = 1;
		printf("\r\n file_num =%d,all_page=%d",file_num,all_page);
		
		
	//	PCM1770_VolumeSet(0);				//���ڳ�ʼ����	
		PCM1770_VolumeSet(28);				//��֪��Ϊʲô��ʼ����������Ч����һ���ϵ��ʱ�����ֻ�ܴ������ڲ��Ÿ��������������������
																	//�ڲ��Ÿ���ǰ����������ʱ���ᵼ�º��沥��û������~	
		
		
		lcd_list(current_page);				//��ʾ�����б�,��һҳ
	
		while(1)											//������ѭ��������״̬�л�				
		{		
			if(play_index >= file_num-1)//���play_index	
				play_index = file_num-1;	//indexָ�����һ���ļ�
			else if(play_index<=0)				
				play_index =0;				
			
			even_process();							//�����¼�
				
			switch(player_state)
			{
				case S_PLAY:							//����״̬	
												
					//��ʼplay����						
					//��ȡ��Ƶ�ļ�����
					//��playlist����ȡ��Ƶ�ļ���
					fres = f_open (&file, "0:mp3player/playlist.txt", FA_READ);
					fres = f_lseek (&file, play_index*FILE_NAME_LEN);
					fres = f_read(&file, music_name, FILE_NAME_LEN, &rw_num);
					fres = f_close (&file);
			
					//��ȡ�ļ�����׼������					
					printf("׼������:%s ",music_name);
				
					if(strstr(music_name,".mp3")||strstr(music_name,".MP3"))	//MP3��ʽ
					{				
						//��ʼmp3����
						mp3_player(music_name);
					}
					else																											//wav��ʽ
					{			
						//��ʼwav�ļ�����
						wav_player(music_name);
					}	
					
				break;
						
				case S_SWITCH:				 //�и�״̬
						player_state = S_PLAY;																	//���±�־λ
						
						/* ���Ҫ�л��ĸ����Ƿ��ڲ����б����һҳ */
						if((play_index+8)/8 < current_page) //play_index���㿪ʼ��������+1��+7��ʹ�ý�һ���� +1+7 =+8
						{
							current_page--;										//���µ�ǰҳ��
							lcd_list(current_page);						//ˢ��LCD�б�	
						}
				
						/* ���Ҫ�л��ĸ����Ƿ��ڲ����б����һҳ */
						if((play_index+8)/8 >current_page)  //play_index���㿪ʼ��������+1��+7��ʹ�ý�һ���� +1+7 =+8
						{
							current_page++;										//���µ�ǰҳ��
							lcd_list(current_page);						//ˢ��LCD�б�
						}
						
					break;				

				
				default:break;

				}
		}

}



/**
  * @brief  file_scan����ǵݹ���̣�
	*					ɾ���ɵ�playlist�����ڴ洢����·���������ļ���λ����lcdlist�����ڴ洢�����б�������ʾ��
  * @param  path:��ʼɨ��·��
  * @retval none
  */
static void file_scan(char* path)
{ 
	
  fres = f_unlink("0:mp3player/playlist.txt");//ɾ���ɵ�playlist
	fres = f_unlink("0:mp3player/lcdlist.txt");	//ɾ���ɵ�playlist
	fres = scan_files(path);				//�ݹ�ɨ������ļ�
}

/**
  * @brief  scan_files �ݹ�ɨ��sd���ڵĸ����ļ�
  * @param  path:��ʼɨ��·��
  * @retval result:�ļ�ϵͳ�ķ���ֵ
  */
static FRESULT scan_files (char* path) 
{ 
    FRESULT res; 		//�����ڵݹ���̱��޸ĵı���������ȫ�ֱ���	
    FILINFO fno; 
    DIR dir; 
    int i; 
    char *fn; 
		char file_name[FILE_NAME_LEN];	
	
#if _USE_LFN 
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1]; 	//���ļ���֧��
    fno.lfname = lfn; 
    fno.lfsize = sizeof(lfn); 
#endif 

    res = f_opendir(&dir, path); //��Ŀ¼
    if (res == FR_OK) 
			{ 
        i = strlen(path); 
        for (;;) 
				{ 
            res = f_readdir(&dir, &fno); 										//��ȡĿ¼�µ�����
            if (res != FR_OK || fno.fname[0] == 0) break; 	//Ϊ��ʱ��ʾ������Ŀ��ȡ��ϣ�����
#if _USE_LFN 
            fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
            fn = fno.fname; 
#endif 
            if (*fn == '.') continue; 											//���ʾ��ǰĿ¼������			
            if (fno.fattrib & AM_DIR) 
						{ 																							//Ŀ¼���ݹ��ȡ
                sprintf(&path[i], "/%s", fn); 							//�ϳ�����Ŀ¼��
                res = scan_files(path);											//�ݹ���� 
                if (res != FR_OK) 
									break; 																		//��ʧ�ܣ�����ѭ��
                path[i] = 0; 
            } 
						else 
						{ 
              printf("%s/%s\r\n", path, fn);								//����ļ���

								if(strstr(fn,".mp3")||strstr(fn,".MP3")||strstr(fn,".wav")||strstr(fn,".WAV"))//�ж��Ƿ�mp3��wav�ļ�
								{
									if (strlen(path)+strlen(fn)<FILE_NAME_LEN)
									{
										sprintf(file_name, "%s/%s", path, fn); 
									
										//�洢��Ƶ�ļ�����playlist(��·��)
										res = f_open (&file, "0:mp3player/playlist.txt", FA_READ|FA_WRITE|FA_CREATE_ALWAYS );
										res = f_lseek (&file, file_num*FILE_NAME_LEN);  
										res = f_write (&file, file_name, FILE_NAME_LEN, &rw_num);	
										res = f_close (&file);
										
										//�洢��Ƶ�ļ�����·����lcdlist�����ļ�����
										res = f_open (&file, "0:mp3player/lcdlist.txt", FA_READ|FA_WRITE|FA_CREATE_ALWAYS );
										res = f_lseek (&file, file_num*FILE_NAME_LEN);  
										res = f_write (&file, fn, FILE_NAME_LEN, &rw_num);	
										res = f_close (&file);

										
										file_num++;//��¼�ļ�����									
							
									}
								}//if mp3||wav
            }//else
        } //for
    } 

    return res; 
} 




/**
  * @brief  mp3_player ����mp3�ļ����롢����
  * @param  filename:Ҫ���ŵ��ļ�·��
  * @retval none
  */
static void mp3_player(const char *filename)
{
	int err, i, outputSamps, current_sample_rate = 0;	

	int						read_offset = 0;				/* ��ƫ��ָ��				*/
	int						bytes_left = 0;					/* ʣ���ֽ���				*/	
	unsigned long	Frames = 0;							/* mP3֡����				*/
	unsigned char	*read_ptr = buffer;			/* ������ָ��				*/
	HMP3Decoder		Mp3Decoder;						  /* mp3������ָ��		*/
	

	
	//����Ƶ�ļ�
	fres = f_open (&file, filename, FA_READ );
	
	//��ʧ��
	if (fres!=FR_OK)
	{
		printf("read file %s error  ! open another file\r\n",filename);
		fres = f_close (&file);
		
		if (++play_index>=file_num)	//����ֵ��1
		{
				play_index=0;						//��0��������������ļ�����ʧ�ܻ�һֱѭ��
		}				
		return ;										//�ļ��޷��򿪣���ֹ���롣������һ��ѭ������ȡ��һ���ļ�
	}
		
	//�򿪳ɹ�
	//��ʼ��MP3������
	Mp3Decoder = MP3InitDecoder();	
	
	//��ȡ����������������helix����룬���PCM���ݣ�Լ20ms���һ��ѭ��
	//��ʼ���벥��״̬,�ڼ��жϻ��޸�touch_even״̬
	while(player_state != S_SWITCH)	//ѭ��1�� ���touch_even�����и�״̬���������ѭ������ 
	{
		//��ʱ���ֽ�����󣬴��������ڱ�ѭ�����ڣ���������
		
		//��ʾ����ͼ��
		Lcd_GramScan(1);
		LCD_Clear(12,88,8,145,BACKGROUND);
		Lcd_show_bmp(320-(103+((play_index-((current_page-1)*8))*18)),240-20,"/mp3player/ui_playing.bmp");
		
		//��ȡmp3�ļ�
		fres = f_read(&file, buffer, sizeof(buffer), &rw_num);
		if(fres != FR_OK)
		{
			printf("��ȡ%sʧ�ܣ� %d\r\n",filename,fres);
			break;
			//return;
		}
		read_ptr = buffer;									//ָ��mp3������
		bytes_left = rw_num;								//ʵ�ʶ�������������С��С

		//��֡����	
		while(player_state != S_SWITCH)			//ѭ��2��ѭ�������̲�����Ƶ��ֱ��������һ�ס���һ��	
		{
			if (player_state == S_STOP)
			{								
				even_process();									//����Ƿ����¼���Ҫ����
				continue;												//��ͣ��ʱ���������ѭ��
			}
				
			player_state = S_PLAY;						//״̬����Ϊ���ڲ���
			
			read_offset = MP3FindSyncWord(read_ptr, bytes_left);	//Ѱ��֡ͬ�������ص�һ��ͬ���ֵ�λ��
			if(read_offset < 0)																		//û���ҵ�ͬ����
			{
				break;																							//����ѭ��2���ص�ѭ��1	
			}
			
			read_ptr += read_offset;					//ƫ����ͬ���ֵ�λ��
			bytes_left -= read_offset;				//ͬ����֮������ݴ�С	
			if(bytes_left < 1024)							//��������
			{
				/* ע������ط���Ϊ���õ���DMA��ȡ������һ��Ҫ4�ֽڶ���  */
				i=(uint32_t)(bytes_left)&3;									//�ж϶�����ֽ�
				if(i) i=4-i;														//��Ҫ������ֽ�
				memcpy(buffer+i, read_ptr, bytes_left);	//�Ӷ���λ�ÿ�ʼ����
				read_ptr = buffer+i;										//ָ�����ݶ���λ��
				fres = f_read(&file, buffer+bytes_left+i, sizeof(buffer)-bytes_left-i, &rw_num);//��������
				bytes_left += rw_num;										//��Ч��������С
			}
			err = MP3Decode(Mp3Decoder, &read_ptr, &bytes_left, outBuf[bufflag], 0);					//��ʼ���� ������mp3����ṹ�塢������ָ�롢��������С�������ָ�롢���ݸ�ʽ
			Frames++;			
			
			if (err != ERR_MP3_NONE)									//������
			{
				switch (err)
				{
					case ERR_MP3_INDATA_UNDERFLOW:
						printf("ERR_MP3_INDATA_UNDERFLOW\r\n");
						read_ptr = buffer;
						fres = f_read(&file, read_ptr, sizeof(buffer), &rw_num);
						bytes_left = rw_num;
						break;
			
					case ERR_MP3_MAINDATA_UNDERFLOW:
						/* do nothing - next call to decode will provide more mainData */
						printf("ERR_MP3_MAINDATA_UNDERFLOW\r\n");
						break;
			
					default:
						printf("UNKNOWN ERROR:%d\r\n", err);
			
						// ������֡
						if (bytes_left > 0)
						{
							bytes_left --;
							read_ptr ++;
						}	
						break;
				}
			}
			else		//�����޴���׼�������������PCM
			{
				MP3GetLastFrameInfo(Mp3Decoder, &Mp3FrameInfo);		//��ȡ������Ϣ				

		    /* ���ݽ�����Ϣ���ò����� */
				if (Mp3FrameInfo.samprate != current_sample_rate)	//������ 
				{
					current_sample_rate = Mp3FrameInfo.samprate;

					printf(" \r\n Bitrate       %dKbps", Mp3FrameInfo.bitrate/1000);
				  printf(" \r\n Samprate      %dHz", current_sample_rate);
					printf(" \r\n BitsPerSample %db", Mp3FrameInfo.bitsPerSample);
					printf(" \r\n nChans        %d", Mp3FrameInfo.nChans);
					printf(" \r\n Layer         %d", Mp3FrameInfo.layer);
					printf(" \r\n Version       %d", Mp3FrameInfo.version);
					printf(" \r\n OutputSamps   %d", Mp3FrameInfo.outputSamps);

					if(current_sample_rate >= I2S_AudioFreq_Default)	//I2S_AudioFreq_Default = 2��������֡��ÿ�ζ�Ҫ������
					{
						I2S_Freq_Config(current_sample_rate);						//���ݲ������޸�iis����
					}
				}
				
				/* �����DAC */
				outputSamps = Mp3FrameInfo.outputSamps;							//PCM���ݸ���
				
				if (outputSamps > 0)
				{
					if (Mp3FrameInfo.nChans == 1)	//������
					{
						//������������Ҫ����һ�ݵ���һ������
						for (i = outputSamps - 1; i >= 0; i--)
						{
							outBuf[bufflag][i * 2] = outBuf[bufflag][i];
							outBuf[bufflag][i * 2 + 1] = outBuf[bufflag][i];
						}
						outputSamps *= 2;
					}
				
					//�ǵ��������ݿ�ֱ����DMA���䵽IIS����DAC
					/* �ȴ�DMA�����꣬���ʱ�����ǿ��Ը��������£�ɨ���¼����д��� */
					while((DMA1_Channel5->CCR&DMA_CCR1_EN) && !(DMA1->ISR&DMA1_IT_TC5))
					{
							even_process();							
						}
						
					/*DMA�������*/
			    DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5);
			    DMA_I2S_Configuration((uint32_t)outBuf[bufflag], outputSamps);
					bufflag = 1 -bufflag;																			//�л�buffer

				}//if (outputSamps > 0)
			}//else ��������
			
		
		if(file.fptr==file.fsize) 		//���ָ��ָ�����ļ�β����ʾ����ȫ������
		{
			printf("END\r\n");
			if(play_index<file_num-1)		//�Զ���ʼ��һ�׸���
			{
				play_index++;
				player_state = S_SWITCH;	//�����и�״̬������
			}
			else
			{
				play_index = 0;
				player_state = S_SWITCH;
			}
					
			break; 										//�������׸�Ĳ���״̬	while break; 
		}
  	
	}//ѭ��2		�� while(player_state != S_SWITCH)	
	
 }//ѭ��1  	�� while(player_state != S_SWITCH)

	f_close(&file);							//�������ű��������ر��ļ�
	MP3FreeDecoder(Mp3Decoder);
	I2S_Stop();
	
}


/**
  * @brief  wav_player ����wav�ļ�����
	*					wav��ʽ�洢�ľ���PCM���ݣ�����Ҫ����
  * @param  filename:Ҫ���ŵ��ļ�·��
  * @retval none
  */
static void wav_player(const char *filename)
{
	short *p;
	WavHead *wav;	

	//����Ƶ�ļ�
	fres = f_open (&file, filename, FA_READ );
	
	//��ʧ��
	if (fres!=FR_OK)
	{
		printf("read file %s error  ! open another file\r\n",filename);
		fres = f_close (&file);
		
		if (++play_index>=file_num)	//����ֵ��1
		{
				play_index=0;						//��0��������������ļ�����ʧ�ܻ�һֱѭ��
		}				
		return ;										//�ļ��޷��򿪣���ֹ���롣������һ��ѭ������ȡ��һ���ļ�
	}
	
		//��ʾ����ͼ��
	Lcd_GramScan(1);
	LCD_Clear(12,88,8,145,BACKGROUND);
	Lcd_show_bmp(320-(103+((play_index-((current_page-1)*8))*18)),240-20,"/mp3player/ui_playing.bmp");	
	
	
	fres = f_read(&file,buffer,512,&rw_num);							//��ȡ�ļ�ͷ
	
	wav = (WavHead *)buffer;															//�����ʽ
		
	printf("\r\n samprate: %dHz", wav->dwSamplesPerSec);	//������

	if(wav->dwSamplesPerSec >= I2S_AudioFreq_Default)
		I2S_Freq_Config(wav->dwSamplesPerSec);							//���ò�����

		//����ѭ��	
		while(player_state != S_SWITCH)			//ѭ�������̲�����Ƶ��ֱ���и�
		{
			if (player_state == S_STOP)
				{			
					even_process();
					continue;											//��ͣ��ʱ��������ѭ��
				}
				
			player_state = S_PLAY;						//״̬����Ϊ���ڲ���
		
				
			//��ȡwav�ļ�
			p = (short *)(buffer+sizeof(buffer)/2*bufflag);		
			fres = f_read(&file, p, sizeof(buffer)/2, &rw_num);
			if(fres != FR_OK)
			{
				printf("��ȡ%sʧ�ܣ� %d\r\n",filename,fres);
				break;
			}
			
			/* �ȴ�DMA�����꣬���ʱ�����ǿ��Ը��������£��¼����� */
			while((DMA1_Channel5->CCR & DMA_CCR1_EN) && !(DMA1->ISR&DMA1_IT_TC5))
			{
					even_process();
			}
        DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_TE5);
        DMA_I2S_Configuration((uint32_t)p, rw_num/2);
				bufflag = 1 -bufflag;				//�л�buffer																	
							
		
			if(file.fptr==file.fsize) 		//���ָ��ָ�����ļ�β����ʾ����ȫ������
			{
				printf("END\r\n");
				if(play_index<file_num-1)		//�Զ���ʼ��һ�׸���
				{
					play_index++;
					player_state = S_SWITCH;	
				}
				else
				{
					play_index = 0;
					player_state = S_SWITCH;
				}
						
				break; 									//�������׸�Ĳ���״̬	while break; 
			}  	
	}
	
	f_close(&file);								//�������ű��������ر��ļ�
	I2S_Stop();
}


/**
  * @brief  lcd_list ��ʾ�����б�
  * @param  �����б��ҳ��
  * @retval none
  */
static void lcd_list(char page)
{	
	char i;
	char music_name[MUSIC_NAME_LEN];	
	char lcd_char[MUSIC_NAME_LEN+4];					//+4��Ϊ��Ԥ���ռ�ĸ������
	FIL name_file;
	
	
	select_index=0;														//������ʾ�µ�һҳʱ����select_index
	Lcd_GramScan( 1 );												//����ɨ�跽��
	LCD_Clear(12,86,206,145,BACKGROUND);
	
	
	sprintf(lcd_char,"�� %0d/%0d ҳ",current_page,all_page);		//��ʾҳ��
	LCD_DispEnCh(30,235,(const uint8_t *)lcd_char,BLUE);
	

	sprintf(lcd_char,"����:%2d",voice);													//��ʾ����
	LCD_DispEnCh(155,235,(const uint8_t *)lcd_char,BLUE);
	
	
	/* ��ɨ���¼���ĸ赥 */ 
	fres = f_open (&name_file, "0:mp3player/lcdlist.txt", FA_READ);	
	
	/* ��playlist����ȡ��Ƶ�ļ��� */
	for(i=0;(i+8*(page-1))<file_num && i< 8;i++)								//����ʾ��һҳ
	{	
		fres = f_lseek (&name_file, (8*(page-1) + i)*FILE_NAME_LEN);			//�ļ��еĸ�������FILE_NAME_LEN��ƫ��
		fres = f_read(&name_file, music_name, MUSIC_NAME_LEN, &rw_num);		//ֻ��ȡMUSIC_NAME_LEN���ȵ��ַ���̫���Ļ�LCD�����ռ���ʾ
		
		music_name[MUSIC_NAME_LEN-1]='\0';												//�����һ��Ԫ������Ϊ'\0'��ֹû�н������Ŷ����
		
		sprintf(lcd_char,"%0d.%s",8*(page-1)+i+1,music_name);			//ƴ�ӳ������б�
		
		LCD_DispEnCh(22,87+i*18,(const uint8_t *)lcd_char,RED);										//��ʾ

	}
	
	fres = f_close (&name_file);
}


/**
  * @brief  even_process �����¼���־���д���
  * @param  none
  * @retval none
  */
static void even_process(void)
{
	switch(touch_even)
	{
		/* ������				 */
		case E_UP:														
			
			Volume_Add();		
			
			touch_even = E_NULL;
		
			break;
		
		/* ������ 			*/
		case E_DOWN:													
			
			Volume_Dec();	
			
			touch_even = E_NULL;	
		
			break;
		
		/* ���š���ͣ��	*/
		case E_PLAY_STOP:												
		
			if(player_state == S_PLAY)					//����ǰ״̬Ϊ����
			{
				player_state = S_STOP;						//�л�Ϊ��ͣ״̬
			}		
			else																//����ǰ״̬Ϊ��ͣ
				player_state =S_PLAY;							//�л�Ϊ������ͣ
				
			touch_even = E_NULL;
			break;	

		/* ��һ��			*/	
		case E_PREV:														
			if(play_index <= 0)
			{
				play_index = 0;
			}
			else
			{
				play_index--;
				#if 0																//����player_run ������ͳһ������	
				/* ���Ҫ�л��ĸ����Ƿ��ڲ����б����һҳ */
				if((play_index+8)/8 < current_page) //play_index���㿪ʼ��������+1��+7��ʹ�ý�һ���� +1+7 =+8
				{
					current_page--;										//���µ�ǰҳ��
					lcd_list(current_page);						//ˢ��LCD�б�	
				}
				#endif

			}
			
			touch_even = E_NULL;						
			player_state = S_SWITCH;							//�����и�״̬
			
			break;
		
		/* ��һ��	 	 */			
		case E_NEXT:														
			
			if(play_index < file_num-1)
			{
				play_index++;	
				#if 0															//����player_run ������ͳһ������
				/* ���Ҫ�л��ĸ����Ƿ��ڲ����б����һҳ */
				if((play_index+8)/8 >current_page)  //play_index���㿪ʼ��������+1��+7��ʹ�ý�һ���� +1+7 =+8
				{
					current_page++;										//���µ�ǰҳ��
					lcd_list(current_page);						//ˢ��LCD�б�
				}
				#endif
			}									
			else
			{
				play_index = file_num-1;						//play_index����Ϊ���һ�������ļ�
			}		
				touch_even = E_NULL;	
				player_state = S_SWITCH;						//�����и�״̬
		
			break;
		
		/* ����			*/
		case E_SELT_UP:													
			if(select_index>0)
				select_index--;
			
			Lcd_GramScan(1);
			LCD_Clear(12,88,8,145,BACKGROUND);		//���״̬��
			Lcd_show_bmp(320-(103+(select_index*18)),240-20,"/mp3player/ui_select.bmp");			//��ʾ���򹴡���ǩ
		
			touch_even = E_NULL;
			
			break;
		
		/* ���� 		*/ 
		case E_SELT_DOWN:												
			if(select_index<8-1 && (select_index+8*(current_page-1))<file_num-1)	//�ж��Ƿ������������8����ָ�����ļ�����λ��
				select_index++;			
			
			Lcd_GramScan(1);
			LCD_Clear(12,88,8,145,BACKGROUND);	//���״̬��
			Lcd_show_bmp(320-(103+(select_index*18)),240-20,"/mp3player/ui_select.bmp");			//��ʾ���򹴡���ǩ
		
			touch_even = E_NULL;	
					
			break;
		
		/* ֱ�ӵ�ѡ����	*/
		case E_SELECT:												
			
			play_index = select_index + ((current_page-1)*8);											//���ݵ�ǰҳ��select_indexȷ��play_index
			player_state = S_SWITCH;
		
			touch_even = E_NULL;	
		
			break;			
		
		/* ��һҳ				*/
		case E_SELT_NEXT:												
			
			if(current_page<all_page)
			{
				current_page++;										//���µ�ǰҳ��
				lcd_list(current_page);						//ˢ��LCD�б�
			}
			else
				current_page =1;
		
			touch_even = E_NULL;
			
			break;
		
		/* ��һҳ			*/
		case E_SELT_PREV:											
			if(current_page>1)					
			{
				current_page--;										//���µ�ǰҳ��
				lcd_list(current_page);						//ˢ��LCD�б�
			}
			else
				current_page =1;
			
			touch_even = E_NULL;
			break;
		
		/* OK					*/
		case E_SELT_OK:												
			play_index = select_index+8*(current_page-1);	//���ݵ�ǰҳ��select_indexȷ��play_index
			
			touch_even = E_NULL;	
			player_state = S_SWITCH;
			
			break;
		
		/* ������ 		*/
		case E_LOUD_SPEAK:										
			if(speaker_flag == 0)								//speaker��״̬
			{
				Loud_Speaker_ON();								//����speaker	
				speaker_flag =1;
				printf("\r\n loud on");
			}
			else																//speakerΪ��״̬
			{
				Loud_Speaker_OFF();								//�ر�speaker
				speaker_flag=0;
				printf("\r\n loud off");
			}
			
			touch_even = E_NULL;
			break;
		
		default:
			//touch_even = E_NULL;
			break;
	}

}

/**
  * @brief  touch_process �����¼������ڴ����ж�ʱ������
  * @param  none
  * @retval none
  */
void touch_process (void)
{
	
	  if(touch_flag == 1)			/*������ʰ�����*/
    {
		
      /* ��ȡ������� */
      if(Get_touch_point(&display, Read_2046_2(), &touch_para ) !=DISABLE)      
      {		
        LED1_TOGGLE;   																//LED��ת	    
				
				//MP3��������x��Χ���ٸ���y�������ְ���
				if(display.x>=13 && display.x<=31)							
				{
						if(display.y >=177 &&display.y<=194)			//���� +
						{
							touch_even = E_UP;
						}
						else if(display.y >=199 &&display.y<=214)	//���� -
						{
							touch_even = E_DOWN;
						}
						else if(display.y >=98 &&display.y<=131)	//������ͣ
						{
							touch_even = E_PLAY_STOP;
							printf("\r\n ������ͣ");
						}
						else if(display.y >=65 &&display.y<=93)		//��һ��
						{
							touch_even = E_PREV;									
							printf("\r\n ��һ��");
						}
						else if(display.y >=139 &&display.y<=167)	//��һ��
						{										
							touch_even = E_NEXT;	
							printf("\r\n ��һ��");
						}
						else if(display.y >=43 &&display.y<=57)		//��һҳ
						{
								touch_even = E_SELT_NEXT;
								printf("\r\n ��һҳ");
						}
						else if(display.y >=24 &&display.y<=43)		//OK��
						{
							touch_even = E_SELT_OK;
							printf("\r\n OK��");
						}
						else if(display.y >=10 &&display.y<=24)		//��һҳ
						{
							touch_even = E_SELT_PREV;					
							printf("\r\n ��һҳ");
						}
							
				}
				/* ���Ϻ����µĿ��ư�ťy��Χ	*/		
				else if(display.y >=24 && display.y<=43)
				{
						if(display.x>=31 && display.x <=43)				//����
						{
								touch_even = E_SELT_UP;
								printf("\r\n����");
							}
						else if(display.x<=13 && display.x>0)		//����
						{
								touch_even = E_SELT_DOWN;
								printf("\r\n����");
							}
				}		
				
				else if(display.x>260 && display.x< 300 &&display.y>47 && display.y<107)	//������
				{
					touch_even = E_LOUD_SPEAK;
					printf("\r\n ������");
				}		
					
				//ֱ��ѡ���������1.2.3.4.������8��~
					
				else if(display.x>208 && display.x<226)
				{
					touch_even = E_SELECT;	
					select_index = 0;
					printf("\r\n play_index=%d",select_index);
				}
				else if(display.x>190 && display.x<208)
				{
					touch_even = E_SELECT;	
					select_index = 1;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>172 && display.x<190)
				{
					touch_even = E_SELECT;	
					select_index = 2;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>154 && display.x<172)
				{
					touch_even = E_SELECT;	
					select_index = 3;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>136 && display.x<154)
				{
					touch_even = E_SELECT;
					select_index = 4;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>118 && display.x<136)
				{
					touch_even = E_SELECT;	
					select_index = 5;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>100 && display.x<118)
				{
					touch_even = E_SELECT;	
					select_index = 6;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>82 && display.x<100)
				{
					touch_even = E_SELECT;	
					select_index = 7;
					printf("\r\n select_index=%d",select_index);
				}
				else if(display.x>64 && display.x<82)
				{
					touch_even = E_SELECT;	
					select_index = 8;
					printf("\r\n select_index=%d",select_index);
				}						
						
      }//if(Get_touch_point(&display, Read_2046_2(), &touch_para ) !=DISABLE)      
			
    }// if(touch_flag == 1)			
	
}


void MP3_test(void)
{
			/* ��ʼ������I2S */
		I2S_Bus_Init();
		
		/* ��ʼ��PCM1770 */
		PCM1770Init();
	
			/* ��ʾMP3����ͼƬ */
		Lcd_show_bmp(0, 0,"/mp3player/ui_window.bmp");

		/* ����MP3������ */	
		player_run();
}



