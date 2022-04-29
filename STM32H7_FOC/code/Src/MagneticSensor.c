

#include "MyProject.h"


/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
#define state_Address 0x0B
#define raw_angle 0x0D
/******************************************************************************/
/******************************************************************************/



/******************************************************************************/
//TLE5012B
#define READ_ANGLE_VALUE  0x8020
#define TLE5012B_CPR      32768
/******************************************************************************/
#define SPI2_CS1_L  GPIO_ResetBits(GPIOB, GPIO_Pin_8)      //CS1_L
#define SPI2_CS1_H  GPIO_SetBits(GPIOB, GPIO_Pin_8)        //CS1_H

#define SPI2_TX_OFF {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0x40000000;}  //F,把PB15(MOSI)复用开漏输出(50MHz)
#define SPI2_TX_ON  {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
/******************************************************************************/

/******************************************************************************/
//#endif

/******************************************************************************/
uint8_t as_zt(){
	
	uint8_t data = 0xff;                  
	HAL_I2C_Mem_Read(&hi2c4,AS5600_Address,state_Address, I2C_MEMADD_SIZE_16BIT, &data , 1, 0xfff);
	data &= 0x38;
	return data ; //中间三位
	
}

float I2C_getRawCount()//获取角度
{ 
  float angle=0.0;
  uint8_t buf[2];
  HAL_I2C_Mem_Read(&hi2c4, AS5600_Address, raw_angle, I2C_MEMADD_SIZE_8BIT, buf, 2, 0xfff);
  angle=((short)buf[1]<<8|buf[0])*0.0878;
  
  return angle;	
}
void MagneticSensor_Init(void)
{
#if M1_AS5600
	cpr=AS5600_CPR;
	angle_data_prev = I2C_getRawCount();  
#elif M1_TLE5012B
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
	
#if M1_AS5600
	angle_data = I2C_getRawCount();
#elif M1_TLE5012B
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



