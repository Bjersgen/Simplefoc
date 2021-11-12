
#include "stm32f10x.h"
#include <stdlib.h>
#include "MyProject.h"

float target;
void commander_run(void);
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13是LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //对选中管脚初始化
	GPIO_SetBits(GPIOC,GPIO_Pin_13);                   //上电点亮LED
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //PB9是motor1的使能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);                  //低电平解除
}
/******************************************************************************/
int main(void)
{
	GPIO_Config();
	uart_init(115200);
	TIM2_PWM_Init();
	TIM4_1ms_Init();           //interrupt per 1ms
	
	delay_ms(1000);            //Wait for the system to stabilize
	
	voltage_power_supply=12;   //V
	voltage_limit=1;           //V，最大值需小于12/1.732=6.9，值越大电机电流越大
	velocity_limit=20;         //rad/s angleOpenloop() use it
	controller=Type_velocity_openloop;  //Type_angle_openloop;  //
	pole_pairs=7;              //极对数
	
	M1_Enable;
  printf("Motor ready.\r\n");
	
	systick_CountMode();   //不能再调用delay_us()和delay_ms()函数
	
	while(1)
	{
		if(time1_cntr>=200)  //0.2s
		{
			time1_cntr=0;
			//LED_blink;
		}
		move(target);
		commander_run();
	}
}
/******************************************************************************/
void commander_run(void)
{
	if((USART_RX_STA&0x8000)!=0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'T':   //T6.28
				target=atof((const char *)(USART_RX_BUF+1));
				printf("RX=%.4f\r\n", target);
				break;
		}
		USART_RX_STA=0;
	}
}
/******************************************************************************/




