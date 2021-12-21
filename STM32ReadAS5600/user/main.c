
#include "stm32f10x.h"
#include "config.h"
#include "sys.h"
#include "usart.h"

#include "timer.c"
#include "i2c.c"

/************************************************
读取AS5600的角度(模拟I2C)
PB10--SCL
PB11--SDA
=================================================

/******************************************************************************/
void delay_s(u32 i)
{
	while(i--);
}
/******************************************************************************/
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;//PB10 PB11 I2C接口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //通用推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);                //对选中管脚初始化
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);          //高电平
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;        //LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);            //对选中管脚初始化
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);                //上电点亮LED
}
/******************************************************************************/
/******************************************************************************/
int main(void)
{
	GPIO_Config();
	uart_init(115200);
	
	delay_s(0x5fffff);       //延时等待系统稳定
	printf("Initial OK!\r\n");
	TIM2_Init(999,71);       //1ms定时，中断函数中累加计数器
	
	while(1)
	{
		if(time1_cntr>=200)   //每0.2S灯亮灭一次
			{
				time1_cntr=0;
				LED_blink;
			}
			if(time2_cntr>=200)  //每0.2S打印
			{
				time2_cntr=0;
				printf("Angle_I2C=%.4f\r\n",AS5600_ReadRawAngleTwo()*0.08789);
			}
	}
}
/******************************************************************************/

