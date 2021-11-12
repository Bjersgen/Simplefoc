
#include "usart.h"

#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
	
/******************************************************************************/
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲，usart.h中定义长度
//接收状态
//bit15  接收完成标志
//bit14  接收到0x0D
//bit13~0  接收的字节数
unsigned short USART_RX_STA=0;       //接收状态标志
/******************************************************************************/
void uart_init(unsigned long bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  
	//USART1_TX   GPIOA.9 = TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  //USART1_RX	  GPIOA.10 = RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;    //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
  
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);       //串口1初始化
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //使能接收中断
  USART_Cmd(USART1, ENABLE);                      //使能串口1

}
/******************************************************************************/
void USART1_IRQHandler(void)   //串口1中断程序
{
	unsigned char Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的字节
		
		if((USART_RX_STA&0x8000)==0)    //接收未完成
		{
			if(USART_RX_STA&0x4000)       //接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;   //接收错误，重新开始
				else 
				{
					USART_RX_STA|=0x8000;	       //接收完成
					USART_RX_BUF[USART_RX_STA&0X3FFF]='\0';   //最后一个字节放'0’，方便判断
				}
			}
			else //还没收到0x0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  //接收错误，重新开始  
				}		 
			}
		}
  }
}
/******************************************************************************/


