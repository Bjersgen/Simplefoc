#ifndef __ADC_H
#define __ADC_H	
/******************************************************************************/	
#include "stm32h7xx.h"
/******************************************************************************/	
#include "main.h"

extern ADC_HandleTypeDef hadc1;

void ADC_Init_(void);
unsigned short analogRead(void);
float _readADCVoltage(void);
/******************************************************************************/	

#endif 
