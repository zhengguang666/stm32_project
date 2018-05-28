#ifndef __FLASH_H
#define	__FLASH_H

#include "stm32f10x.h"

uint8_t Readflash(uint32_t StartAddr);
void Writeflash(uint32_t StartAddr,char *src,uint8_t n);


#endif /* __FLASH_H */
