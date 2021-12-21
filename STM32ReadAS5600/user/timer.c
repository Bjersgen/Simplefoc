

/***************************************************************************/
//arr���Զ���װֵ�� psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void TIM2_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //ʹ��TIM2ʱ��
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;       //��ʱ��2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;    //��Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);                     //��ʼ�� NVIC
	
	TIM_TimeBaseInitStructure.TIM_Period = arr;         //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;        //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);  //��ʼ����ʱ��TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);            //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE);                               //ʹ�ܶ�ʱ��2
}
/***************************************************************************/
//��ʱ��2�жϷ�����
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
		{
			time1_cntr++;
			time2_cntr++;
		}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //����жϱ�־λ
}
/***************************************************************************/



