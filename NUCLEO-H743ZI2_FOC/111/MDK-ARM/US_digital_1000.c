#include "Myproject.h"

long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp=0;
float angle_prev;
float Vel_Now_Motor1;
float Encoder_count_Motor1 = 0;

float getVelocity(void);
float getAngle(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim5.Instance)
	{
			//���̶������ʱ������  �������
            
     
            Encoder_count_Motor1 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
            __HAL_TIM_SET_COUNTER(&htim4,0);
           

          
			//��������ת��
			Vel_Now_Motor1 = Encoder_count_Motor1;

	}
	
}

float getVelocity(void){
	
	  return Vel_Now_Motor1;
	
}

float getAngle(void){
	
	return 0;
}


	
