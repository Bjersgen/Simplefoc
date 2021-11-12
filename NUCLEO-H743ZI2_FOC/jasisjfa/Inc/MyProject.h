

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

#define M1_Enable    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //设置A2引脚输出高电平;          //高电平使能
#define M1_Disable   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);        //低电平解除

#endif

