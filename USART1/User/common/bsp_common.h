#ifndef _STATIC_INIT_H_	
#define _STATIC_INIT_H_

#ifdef __cplusplus		   //�����CPP����C���� //��ʼ����
extern "C" {
#endif
    
#include "stm32f10x.h"
#include "String.h"	

void Delay_US(u32 nCount_temp);
void Delay_MS(u32 nCount_temp);
void Delay_S(u32 nCount_temp);

#ifdef __cplusplus		   //�����CPP����C���� //��������
}
#endif

#endif
