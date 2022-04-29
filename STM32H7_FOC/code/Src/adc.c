
#include "adc.h"

/******************************************************************************/															   


/******************************************************************************/	
uint16_t ADC_Value=0;
unsigned short analogRead(void){
    //开启ADC1
    HAL_ADC_Start(&hadc1);
    //等待ADC转换完成，超时为100ms
    HAL_ADC_PollForConversion(&hadc1,100);
    //判断ADC是否转换成功
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
         //读取值
       return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}
/******************************************************************************/	
// function reading an ADC value and returning the read voltage
float _readADCVoltage(void)
{
  unsigned short raw_adc = analogRead();
  return (float)raw_adc*3.3/4096;
}
/******************************************************************************/	


