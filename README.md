# Simplefoc
Feature——MAX_120W(24V*5A)——Faster(1KHZ for closed loop)

       SimpleFOC——
                        | ——BLDCmotor(self made DIY motor)
                        | ——Documents
                        | ——STM32H7_FOC
                        	|——circle(PCBs & libraries)
                        		|——Shell(a PCB shell)
                                |——code(using Keil5 & STM32CubeMX)
                        		|——Inc
                        		|——Src
                        		|——Drivers
                        		|——MDK-ARM
                                |——datasheet(ICs datasheets)
                                |——image
                                |——readme
                        | ——pico-drv8313
                        | ——pico-drv8313 with foc
                        | ——readme
                        


Most projects are based on Arduino, and one project is based on stm32h743.

# FOC document 2021.12.15


## 目录

### 一、FOC所需原理（快速了解）

#### 1.1 简介

### 二、FOC小圆片IC选型

#### 2.1 电流采样

#### 2.2 驱动芯片

#### 2.3 磁编码器

### 三、FOC-demo

#### 3.1 Arduino(MCU)----SimpleFOC-Shield(driver)

#### 3.2 Arduino(MCU)----Drv8305(driver)

#### 3.3 STM32H743ZI----SimpleFOC-Shield(driver)

### 四、FOC代码及调试的注意事项

### 五、AS5600调试与执行





















### 一、FOC所需原理（快速了解）

#### 1.1 简介



1、正弦波控制实现了电压矢量的控制，间接实现了电流大小的控制，但是无法控制电流的方向。FOC控制方式可以认为是正弦波控制的升级版本，实现了电流矢量的控制，也即实现了电机定子磁场的矢量控制。

2、由于控制了电机定子磁场的方向，所以可以使电机定子磁场与转子磁场时刻保持在90°，实现一定电流下的最大转矩输出。FOC控制方式的优点是：转矩波动小、效率高、噪声小、动态响应快。

3、低转速下控制

由于控制原理的区别，无刷电调只能控制电机工作在高转速下，低速下无法控制；而FOC控制器则完全没有这个限制，不论在什么转速下都可以实现精确控制。

4、电机换向

同上面的理由，由于电调无法反馈转子位置，因此很难实现电机正反转的换向（当然有感电调可以实现）；而FOC驱动器的换向性能极其优秀，最高转速下正反转切换可以非常顺畅；此外FOC还可以以能量回收的形式进行刹车控制。

5、力矩控制

普通电调都只能控制电机转速，而FOC可以进行电流（力矩）、速度、位置三个闭环控制。



![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/7.png)




### 二、FOC小圆片IC选型

#### 2.1 电流采样

##### 1）低侧电流采样

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/1.png)

     低侧电流检测可能是最常见的电流检测技术，主要是因为它既不需要高性能的PWM抑制运放，也不需要支持高压的运放，采样电阻在低侧MOS和GND之间，确保了运放输入端的电压非常低，**所以这种方法对运放没有什么要求**。这种方法的缺点是，必须在下桥臂MOS打开时检测电流，PWM频率通常为20k～50khz，这意味着低侧MOS的开关频率为每秒20k～50k次，因此PWM设置与ADC采集之间的同步非常重要。



##### 2）高侧电流采样

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/2.png)

​    高侧电流检测可能是最不常见的电流检测技术，因为它需要支持高压的运放，采样电阻在高侧MOS和直流电源电压之间，使放大器的输入端始终有高电压。 这种方法的另一个缺点和低侧电流采样一样，需要同步PWM和ADC。**优点：可以检测区分负载是否短路，无地电平干扰**。



##### 3）内置电流采样

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/3.png)

​        内置电流检测（InlineCurrentSense）是使用起来最简单但是最精准的技术。 采样电阻串联在电机相线上，检测的电流始终都是电机相电流，因为电感中的电流不会突变，所以无论PWM占空比的状态如何，采样到的电流都是连续稳定的。**内置电流检测的缺点主要在于芯片，需要比常规放大器更好的PWM抑制功能的高精度双向运放，简单的说就是硬件成本高。**



##### 4）电流采样芯片的选择

经考虑，选择INA240，INA240分为四个型号，主要区别在于集成运放的放大倍数（INA240A1-20V/V，**INA240A2-50V/V**，INA240A3-100V/V,INA240A4-200V/V），下图为电流采样原理图，其中为了使负半轴期的电流信号能被采样到，增加了REF1和REF2两个电阻，为集成运放的正脚提供一个偏置电压V，使其不直接接地，左边的MOS管阵列负责选相，在时序中按顺序读出电流，在程序中只读出了两相，第三项通过基尔霍夫定律iA+iB+iC = 0


![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/4.png)





**下面附上选型INA240A2-50V/V原理图**（SimpleFOCShield所用电流检测芯片）

其中VS对应上图的Supply，PH_B_CS对应B相的片选信号，C2_OUT即为放大后的电流采样结果


![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/5.png)


- 采样电阻0.01 Ω
- 背面需分别短接A0/A2至输出
- C1_OUT=INA_VCC/2 + 0.01 * I * 50
  如果INA_VCC=3.3V，C1_OUT=1.65 + 0.01 * I * 50，电流范围（-3.3A,3.3A）
  **如果INA_VCC=5.0V，C1_OUT=2.50 + 0.01 * I * 50，电流范围（-5A,5A）**



#### 2.2驱动芯片

##### 1）DRV8305

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/6.png)





优点：功能强大，有1-PWM，2-PWM，3PWM，6PWM多种模式，芯片内部的保护做的比较出色。

缺点：需要通过SPI进行模式的配置，无论是走线还是简洁程度上都不够。

Peak current:

##### 2）DRV8313

电流不够

##### 3）L6234D

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/8.png)

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/9.png)

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/10.png)

**Peak current : 5A**

#### 2.3 磁编码器（AS5600）

##### 安装：

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/13.png)

​        AS5600是一种易于编程的具有12位高分辨率模拟或PWM输出 的磁性旋转位置传感器。这个非接触式模块可以检测出磁铁 径向磁轴转动的绝对角度。AS5600是为非接触式电位计应用 而设计的，其稳健的设计消除了外部杂散磁场的影响.。 工业标准的I²C 接口支持用户对非易失性参数进行简单的编程 而不需要专门的程序员来进行。 默认情况下可以输出0到360度的变化范围. 它同样可以通过编 程设定0度（开始位置）和最大角度（终止位置）来定义一个 较小的输出范围。 AS5600配备了智能低功耗功能，以自动降低功耗 。 输入引脚 (DIR) 根据旋转方向选择输出极性。如果DIR 接地， 那么输出值将随顺时针旋转而增加. 如果DIR接至VDD, 那么输 出值将随着逆时针旋转而增加。

##### AS5600霍尔元件位置：

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/14.png)

##### AS5600安装允许位移：

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/15.png)



##### AS5600 IIC通信连接方式：

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/16.png)



### 三、FOC-demo

#### 3.1 Arduino(MCU)----SimpleFOC-Shield(driver)

#### 3.2 Arduino(MCU)----Drv8305(driver)

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/12.gif)



#### 3.3 STM32H743ZI----SimpleFOC-Shield



### 四、FOC代码及调试的注意事项

2021.11.10   开环FOC代码完成BLDCmotor()、FOCmotor()

2021.11.16  21：17 进行编码器代码调试

2021.11.17  20：31 修改工程文件重大bug，编译生成.o文件重定义问题

2021.11.17  21：34 增加了currentsense()子函数，完成电流采样，增加PID(float error)，对误差进行调节

2021.11.21  15：00 keil5的32代码全部完成，但只调试过开环

2021.11.22  23：00 小圆片选型完成

2021.11.23  22：48 小圆片原理图画完且无bug 

2021.12.01  15：00第一版小圆片Over

### 五、AS5600调试与执行

time 5  初始值200，period:2000，作为定时器读取time1-4的编码器计数值

time1-4 编码器(Encoder mode，占用TIM1-4全部)

差分走线(A-, A；B-, B)





AS5600 I2C编程时序图

![](https://github.com/Bjersgen/Simplefoc/blob/main/STM32H7_FOC/image/17.png)

方案1：

time 口用GPIO模拟I2C

以time1 CH1、CH2为例

```c
###########I2C.c文件#############
//AS5500的角度信息地址
#define  RAW_Angle_Hi    0x0C
#define  RAW_Angle_Lo    0x0D

//从机地址
#define  AS5600_Address  0x36
//清零并设置PE9的GPIO为INPUT
#define SDA_IN()  {GPIOE->CRH&=0xFFFF0FFF;GPIOE->CRH|=0x00008000;}
//清零并设置PE11的GPIO为OUTPUT
#define SDA_OUT() {GPIOE->CRH&=0xFFFF0FFF;GPIOE->CRH|=0x00003000;}
//读取PE11即SDA引脚的点平状态
#define READ_SDA  (GPIOE->IDR&(1<<11)) 
//设置GPIO引脚的开关
#define IIC_SCL_1  GPIO_SetBits(GPIOE,GPIO_Pin_9)
#define IIC_SCL_0  GPIO_ResetBits(GPIOE,GPIO_Pin_9)
#define IIC_SDA_1  GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define IIC_SDA_0  GPIO_ResetBits(GPIOE,GPIO_Pin_11)
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

#########main.c中断执行函数######
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim5.Instance)
	{
        #ifdef ENCODER_4096_MAGA
        Encode_count_Motor1 = AS5600_ReadRawAngleTwo()*0.08789
        Encode_count_Motor2 = AS5600_ReadRawAngleTwo()*0.08789
        Encode_count_Motor3 = AS5600_ReadRawAngleTwo()*0.08789
        Encode_count_Motor4 = AS5600_ReadRawAngleTwo()*0.08789
        #endif
            
    }
}


```

