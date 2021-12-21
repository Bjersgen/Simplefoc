


/************************************************
读取AS5600的角度(模拟I2C)
PB10--SCL
PB11--SDA
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120431638
          《SimpleFOC移植STM32（三）—— 角度读取》
创建日期：20210909
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
#define  RAW_Angle_Hi    0x0C
#define  RAW_Angle_Lo    0x0D
#define  AS5600_Address  0x36
/***************************************************************************/
#define SDA_IN()  {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=0x00008000;}
#define SDA_OUT() {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=0x00003000;}
#define READ_SDA  (GPIOB->IDR&(1<<11))
#define IIC_SCL_1  GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define IIC_SCL_0  GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define IIC_SDA_1  GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define IIC_SDA_0  GPIO_ResetBits(GPIOB,GPIO_Pin_11)
/***************************************************************************/
void delay_s(u32 i);
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
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	
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
void IIC_Send_Byte(u8 txd)
{
	u32 i;
	
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
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,rcv=0;
	
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
u8 AS5600_ReadOneByte(u8 addr)
{
	u8 temp;		  	    																 
	
	IIC_Start();
	IIC_Send_Byte(AS5600_Address<<1);
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte((AS5600_Address<<1)+1);
	IIC_Wait_Ack();	 
	temp=IIC_Read_Byte(0);		   
	IIC_Stop();
	
	return temp;
}
/***************************************************************************/
u16 AS5600_ReadRawAngleTwo(void)
{
	u8 dh,dl;		  	    																 
	
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


