
#include "adc.h"

/******************************************************************************/															   
void ADC_Init_(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   // 72M/6=12MHz,ADC时钟不能超过14M
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      //单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//软件触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	                //规则转换的ADC通道数目
	ADC_Init(ADC1,&ADC_InitStructure);     //初始化
	ADC_Cmd(ADC1, ENABLE);                 //使能ADC1
	
	ADC_ResetCalibration(ADC1);	                //开启复位校准 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	ADC_StartCalibration(ADC1);	                //开启校准
	while(ADC_GetCalibrationStatus(ADC1));	    //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/******************************************************************************/	
unsigned short analogRead(unsigned char ch)   
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_55Cycles5 );	//ADC1,ADC通道,顺序值,采样周期	  			    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		      //使能软件触发
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); //等待转换结束
	return ADC_GetConversionValue(ADC1);	          //返回最近一次的转换结果
}
/******************************************************************************/	
// function reading an ADC value and returning the read voltage
float _readADCVoltage(unsigned char ch)
{
  uint32_t raw_adc = analogRead(ch);
  return (float)raw_adc*3.3/4096;
}
/******************************************************************************/	


