

#include "MyProject.h"

/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

//#if AS5600
/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
/******************************************************************************/
unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx)
{
	uint32_t Timeout;
	unsigned short temp;
	unsigned char dh,dl;	
	
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0)return 0;
	}
	/* Send the slave address, Reset the address bit0 for write*/
	I2Cx->DR = AS5600_Address<<1;
	Timeout = 0xFFFF;
	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1 &0x0002) != 0x0002)
	{
		if (Timeout-- == 0)return 0;
	}
	/* Clear ADDR flag by reading SR2 register */
	temp = I2Cx->SR2;
	/* Write the first data in DR register (EV8_1) */
	I2Cx->DR = RAW_Angle_Hi;
	/* EV8_2: Wait until BTF is set before programming the STOP */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	
	/////////////////////////////////////////////////////////////////////////
	/* Set POS bit */
	I2Cx->CR1 |= 0x0800;//CR1_POS_Set;
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0)return 0;
	}
	Timeout = 0xFFFF;
	/* Send slave address */
	I2Cx->DR = (AS5600_Address<<1)+1;

	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1&0x0002) != 0x0002)
	{
		if (Timeout-- == 0)return 0;
	}
	/* EV6_1: The acknowledge disable should be done just after EV6,
	that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and 
	ACK clearing */
	__disable_irq();
	/* Clear ADDR by reading SR2 register  */
	temp = I2Cx->SR2;
	/* Clear ACK */
	I2Cx->CR1 &= 0xFBFF;//CR1_ACK_Reset;
	/*Re-enable IRQs */
	__enable_irq();
	/* Wait until BTF is set */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	/* Disable IRQs around STOP programming and data reading because of the limitation ?*/
	__disable_irq();
	/* Program the STOP */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	/* Read first data */
	dh = I2Cx->DR;
	/* Re-enable IRQs */
	__enable_irq();
	/**/
	/* Read second data */
	dl = I2Cx->DR;
	/* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
	while ((I2Cx->CR1&0x200) == 0x200);
	/* Enable Acknowledgement to be ready for another reception */
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
	/* Clear POS bit */
	I2Cx->CR1  &= 0xF7FF;//CR1_POS_Reset;
	
	temp++;  //useless,otherwise warning
	return ((dh<<8)+dl);
}
/******************************************************************************/
//#endif

//#if TLE5012B
/******************************************************************************/
//TLE5012B
#define READ_STATUS       0x8001 //8000
#define READ_ANGLE_VALUE  0x8021 //8020
#define TLE5012B_CPR      32768
/******************************************************************************/
#define SPI2_CS1_L  GPIO_ResetBits(GPIOB, GPIO_Pin_8)      //CS1_L
#define SPI2_CS1_H  GPIO_SetBits(GPIOB, GPIO_Pin_8)        //CS1_H
#define SPI2_CS2_L  GPIO_ResetBits(GPIOB, GPIO_Pin_12)     //CS2_L
#define SPI2_CS2_H  GPIO_SetBits(GPIOB, GPIO_Pin_12)       //CS2_H

#define SPI2_TX_OFF {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0x40000000;}  //F,把PB15(MOSI)复用开漏输出(50MHz)
#define SPI2_TX_ON  {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
/******************************************************************************/
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
/******************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm)
{
	unsigned short u16Data;
	
	SPI2_CS1_L;
	SPIx_ReadWriteByte(Comm);
	SPI2_TX_OFF;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min
	u16Data = SPIx_ReadWriteByte(0xffff);
	SPIx_ReadWriteByte(0xffff);   //safety_word,useless
	SPI2_CS1_H;
	SPI2_TX_ON;
	
	return(u16Data);
}
/******************************************************************************/
//#endif

/******************************************************************************/
void MagneticSensor_Init(void)
{
#if AS5600
	cpr=AS5600_CPR;
	angle_data_prev = I2C_getRawCount(I2C1);  
#else
	cpr=TLE5012B_CPR;
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
#if AS5600
	angle_data = I2C_getRawCount(I2C1);
#else
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
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
	return  (full_rotation_offset + ( angle_data / (float)cpr) * _2PI) ;
}
/******************************************************************************/
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
/******************************************************************************/



