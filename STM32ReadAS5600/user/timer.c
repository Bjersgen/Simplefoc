

/***************************************************************************/
//arr：自动重装值。 psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM2_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //使能TIM2时钟
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;       //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;    //响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);                     //初始化 NVIC
	
	TIM_TimeBaseInitStructure.TIM_Period = arr;         //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;        //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);  //初始化定时器TIM2
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);            //允许定时器2更新中断
	TIM_Cmd(TIM2,ENABLE);                               //使能定时器2
}
/***************************************************************************/
//定时器2中断服务函数
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
		{
			time1_cntr++;
			time2_cntr++;
		}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位
}
/***************************************************************************/



