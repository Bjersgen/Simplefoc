#ifndef __ADC_H
#define __ADC_H	
/******************************************************************************/	
#include "stm32f10x.h"
/******************************************************************************/	
void ADC_Init_(void);
unsigned short analogRead(unsigned char ch);
float _readADCVoltage(unsigned char ch);
/******************************************************************************/	

#endif 
