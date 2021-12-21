

#include "spi2.h"


/******************************************************************************/
void SPI2_Init_(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_12;  //CS--推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_8|GPIO_Pin_12);  //CS_H
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//PA5--CLK,PA6--MISO,PA7--MOSI,
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI1--双线全双工！！
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;   //16位数据
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      //CPOL=0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;    //CPHA=1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;       //NSS 信号由硬件管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //4--9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;   //CRC 值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
}
/******************************************************************************/


