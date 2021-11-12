
#include "delay.h"


/******************************************************************************/
//延时nus
void delay_us(unsigned long nus)
{
	unsigned long temp;
	
//	RCC_ClocksTypeDef clocks;
//	RCC_GetClocksFreq(&clocks);
	
	SysTick->LOAD =nus*9;   //9=针对72MHz
	SysTick->VAL =0; 
	SysTick->CTRL =SysTick_CTRL_ENABLE_Msk;   //HCLK/8
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));

	SysTick->CTRL=0;
	SysTick->VAL =0; 
}
/******************************************************************************/
//延时nms
//最大延时时间=0xFFFFFF/9MHz=1864ms
void delay_ms(unsigned short nms)
{
	unsigned long temp;

	SysTick->LOAD=(u32)nms*9000;   //9=针对72MHz
	SysTick->VAL =0;	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //HCLK/8
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));

	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL =0;
}
/******************************************************************************/
//0xFFFFFF到0循环计数 
void systick_CountMode(void)
{
	SysTick->LOAD = 0xFFFFFF-1;      //set reload register
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; //Enable SysTick Timer
}
/******************************************************************************/
