

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_it.h" 
#include "usart.h"
#include "delay.h"
#include "tim.h"

#include "foc_utils.h" 
#include "FOCMotor.h" 
#include "BLDCmotor.h" 

#define M1_Enable    GPIO_SetBits(GPIOB,GPIO_Pin_9);          //高电平使能
#define M1_Disable   GPIO_ResetBits(GPIOB,GPIO_Pin_9);        //低电平解除

#endif

