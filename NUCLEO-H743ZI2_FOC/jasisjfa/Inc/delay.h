
#ifndef STM32_DELAY_H
#define STM32_DELAY_H
/******************************************************************************/

#include "stm32h7xx.h"

void delay_us(unsigned long nus);
void delay_ms(unsigned short nms);
void systick_CountMode(void);
/******************************************************************************/

#endif

