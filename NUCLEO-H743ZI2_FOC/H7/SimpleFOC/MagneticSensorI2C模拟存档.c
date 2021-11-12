

#include "MyProject.h"

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
/***************************************************************************/
#define SDA_IN()  {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=0x00008000;}
#define SDA_OUT() {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=0x00005000;}
#define READ_SDA  (GPIOB->IDR&(1<<11))
#define IIC_SCL_1  GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define IIC_SCL_0  GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define IIC_SDA_1  GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define IIC_SDA_0  GPIO_ResetBits(GPIOB,GPIO_Pin_11)
/***************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/***************************************************************************/
unsigned char AS5600_ReadOneByte(unsigned char addr);
unsigned short I2C_getRawCount(void);
/***************************************************************************/
void delay_s(unsigned long i)
{
	while(i--);
}
/***************************************************************************/
void I2C2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//GPIOB
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //GPIO_Mode_AF_OD
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}
/***************************************************************************/
void MagneticSensorI2C_Init(void)
{
	cpr=AS5600_CPR;
	full_rotation_offset = 0;
	angle_data_prev = I2C_getRawCount();  
	velocity_calc_timestamp=0;
}
/***************************************************************************/
void IIC_Start(void)
{
	IIC_SDA_1;
	IIC_SCL_1;
	delay_s(20);
	IIC_SDA_0;
	delay_s(20);
	IIC_SCL_0;
}
/***************************************************************************/
void IIC_Stop(void)
{
	IIC_SCL_0;
	IIC_SDA_0;
	delay_s(20);
	IIC_SCL_1;
	IIC_SDA_1;
	delay_s(20);
}
/***************************************************************************/
//1-fail,0-success
unsigned char IIC_Wait_Ack(void)
{
	unsigned char ucErrTime=0;
	
	SDA_IN();
	IIC_SDA_1;
	IIC_SCL_1;
	delay_s(10);
	while(READ_SDA!=0)
	{
		if(++ucErrTime>250)
			{
				SDA_OUT();
				IIC_Stop();
				return 1;
			}
	}
	SDA_OUT();
	IIC_SCL_0;
	return 0; 
}
/***************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_0;
	IIC_SDA_0;
	delay_s(20);
	IIC_SCL_1;
	delay_s(20);
	IIC_SCL_0;
}
/***************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_0;
	IIC_SDA_1;
	delay_s(20);
	IIC_SCL_1;
	delay_s(20);
	IIC_SCL_0;
}
/***************************************************************************/
void IIC_Send_Byte(unsigned char txd)
{
	unsigned long i;
	
	IIC_SCL_0;
	for(i=0;i<8;i++)
	{
		if((txd&0x80)!=0)IIC_SDA_1;
		else
			IIC_SDA_0;
		txd<<=1;
		delay_s(20);
		IIC_SCL_1;
		delay_s(20);
		IIC_SCL_0;
		delay_s(20);
	}
}
/***************************************************************************/
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,rcv=0;
	
	SDA_IN();
	for(i=0;i<8;i++)
	{
		IIC_SCL_0; 
		delay_s(20);
		IIC_SCL_1;
		rcv<<=1;
		if(READ_SDA!=0)rcv++;
		delay_s(10);
	}
	SDA_OUT();
	if(!ack)IIC_NAck();
	else
		IIC_Ack();
	return rcv;
}
/***************************************************************************/
unsigned short I2C_getRawCount(void)
{
	unsigned char dh,dl;		  	    																 
	
	IIC_Start();
	IIC_Send_Byte(AS5600_Address<<1);
	IIC_Wait_Ack();
	IIC_Send_Byte(RAW_Angle_Hi);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte((AS5600_Address<<1)+1);
	IIC_Wait_Ack();
	dh=IIC_Read_Byte(1);   //1-ack for next byte
	dl=IIC_Read_Byte(0);   //0-end trans
	IIC_Stop();
	
	return ((dh<<8)+dl);
}
/***************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
	angle_data = I2C_getRawCount();
	
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

