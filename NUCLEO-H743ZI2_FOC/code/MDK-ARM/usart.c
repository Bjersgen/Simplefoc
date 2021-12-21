#include "usart.h"
#include "stdio.h"

#if 1
#pragma import(__use_no_semihosting)
UART_HandleTypeDef huart3;
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;


int fputc(int ch, FILE *f)
{      
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart3,temp,1,2);
		return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart3, &ch, 1, 0xffff);
  return ch;
}


#endif 
