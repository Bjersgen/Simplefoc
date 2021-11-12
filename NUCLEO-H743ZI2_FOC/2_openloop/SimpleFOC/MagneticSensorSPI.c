
#include "MyProject.h"

/***************************************************************************/
//long  cpr;
//float full_rotation_offset;
//long  angle_data_prev;
//unsigned long velocity_calc_timestamp;
//float angle_prev;
/******************************************************************************/
//TLE5012B
#define READ_STATUS       0x8001 //8000
#define READ_ANGLE_VALUE  0x8021 //8020
/***************************************************************************/
#define SPI2_CS1_L  GPIO_ResetBits(GPIOB, GPIO_Pin_8)      //CS1_L
#define SPI2_CS1_H  GPIO_SetBits(GPIOB, GPIO_Pin_8)        //CS1_H
#define SPI2_CS2_L  GPIO_ResetBits(GPIOB, GPIO_Pin_12)     //CS2_L
#define SPI2_CS2_H  GPIO_SetBits(GPIOB, GPIO_Pin_12)       //CS2_H

#define SPI2_TX_OFF {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xF0000000;}  //把PB15(MOSI)复用开漏输出(50MHz)
#define SPI2_TX_ON  {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
/***************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm);
/***************************************************************************/
void MagneticSensorSPI_Init(void)
{
	cpr=32768;   //for TLE5012B
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/***************************************************************************/
unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
	unsigned short retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(++retry>200)return 0;
	}
	SPI_I2S_SendData(SPI2, byte);
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) 
	{
		if(++retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI2);
}
/***************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm)
{
	unsigned short u16Data;
	
	SPI2_CS1_L;
	SPIx_ReadWriteByte(Comm);
	SPI2_TX_OFF;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min
	u16Data = SPIx_ReadWriteByte(0xffff);
	SPIx_ReadWriteByte(0xffff);   //safety_word
	SPI2_CS1_H;
	SPI2_TX_ON;
	
	return(u16Data);
}
/***************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;

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
	return  (full_rotation_offset + ( angle_data * _2PI / cpr));
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

