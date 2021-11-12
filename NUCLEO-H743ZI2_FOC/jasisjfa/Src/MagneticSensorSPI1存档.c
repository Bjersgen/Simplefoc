
#include "MyProject.h"

/******************************************************************************/
//#define  AS5047P_DIAAGC    0xFFFC   
//bit11:MAGL, Magnetic field strength too low, AGC=0xFF
//bit10:MAGH, Magnetic field strength too high,AGC=0x00
//bit9:COF, CORDIC overflow
//bit8:LF, offset compensation
//bit7-0:AGC, value
//#define  AS5047P_ANGLECOM  0xFFFF
/******************************************************************************/
//TLE5012B
#define READ_STATUS       0x8001 //8000
#define READ_ANGLE_VALUE  0x8021 //8020
/***************************************************************************/
#define SPI_CS_L  GPIO_ResetBits(GPIOA, GPIO_Pin_4)      //CS_L
#define SPI_CS_H GPIO_SetBits(GPIOA, GPIO_Pin_4)         //CS_H

#define SPI_TX_OFF {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xF0000000;}  //把PA7(MOSI)复用开漏输出(50MHz)
#define SPI_TX_ON  {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xB0000000;}  //把PA7(MOSI)复用推挽输出(50MHz)
/***************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/***************************************************************************/
unsigned short ReadTLE5012B(unsigned short Comm);
/***************************************************************************/
void delay_s(unsigned long i)
{
	while(i--);
}
/***************************************************************************/
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  //PA4--CS--推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	SPI_CS_H;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5--CLK,PA6--MISO,PA7--MOSI,
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI1--双线全双工！！
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;   //16位数据
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;      //CPOL=0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;    //CPHA=1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;       //NSS 信号由硬件管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //8--9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;   //CRC 值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}
/***************************************************************************/
void MagneticSensorSPI_Init(void)
{
	cpr=32768;   //for TLE5012B
	full_rotation_offset = 0;
	angle_data_prev = ReadTLE5012B(READ_ANGLE_VALUE)&0x7FFF;
	velocity_calc_timestamp=0;
}
/***************************************************************************/
unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
	unsigned short retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(++retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, byte);
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) 
	{
		if(++retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1);
}
/***************************************************************************/
unsigned short ReadTLE5012B(unsigned short Comm)
{
	unsigned short u16Data;
	
	SPI_CS_L;
	SPIx_ReadWriteByte(Comm);
	SPI_TX_OFF;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min
	u16Data = SPIx_ReadWriteByte(0xffff);
	SPIx_ReadWriteByte(0xffff);   //safety_word
	SPI_CS_H;
	SPI_TX_ON;
	
	return(u16Data);
}
/***************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
	angle_data = ReadTLE5012B(READ_ANGLE_VALUE)&0x7FFF;
	
	// tracking the number of rotations 
  // in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(fabs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;
  // return the full angle 
  // (number of full rotations)*2PI + current sensor angle 
  return  (full_rotation_offset + ( angle_data * _2PI / cpr)) ;
}
/***************************************************************************/
// Shaft velocity calculation
float getVelocity(void)
{
	unsigned long now_us;
	float Ts, angle_c, vel;
	
  // calculate sample time
  now_us = SysTick->VAL; //_micros();
	if(now_us<velocity_calc_timestamp)Ts = (float)(velocity_calc_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/9*1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3;
	
  // current angle
  angle_c = getAngle();
  // velocity calculation
  vel = (angle_c - angle_prev)/Ts;
  
  // save variables for future pass
  angle_prev = angle_c;
  velocity_calc_timestamp = now_us;
  return vel;
}
/***************************************************************************/

