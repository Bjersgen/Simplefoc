
#include "MyProject.h"

/******************************************************************************/
#define DEF_CURR_FILTER_Tf 0.005 //!< default currnet filter time constant
#define DEF_VEL_FILTER_Tf  0.005 //!< default velocity filter time constant
/******************************************************************************/
//unsigned long lpf_vel_timestamp;
float y_vel_prev=0;
float y_current_q_prev=0;
float y_current_d_prev=0;
/******************************************************************************/
/*
float LPF_velocity(float x)
{
	unsigned long now_us;
	float Ts, alpha, y;
	
	now_us = SysTick->VAL;
	if(now_us<lpf_vel_timestamp)Ts = (float)(lpf_vel_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + lpf_vel_timestamp)/9*1e-6f;
	
	lpf_vel_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f; 
	
	alpha = DEF_VEL_FILTER_Tf/(DEF_VEL_FILTER_Tf + Ts);
	y = alpha*y_prev + (1.0f - alpha)*x;
	y_prev = y;
	
	return y;
}
*/
float LPF_velocity(float x)
{
	float y = 0.9*y_vel_prev + 0.1*x;
	
	y_vel_prev=y;
	
	return y;
}
/******************************************************************************/
float LPF_current_q(float x)
{
	float y = 0.9*y_current_q_prev + 0.1*x;
	
	y_current_q_prev=y;
	
	return y;
}
/******************************************************************************/
float LPF_current_d(float x)
{
	float y = 0.9*y_current_d_prev + 0.1*x;
	
	y_current_d_prev=y;
	
	return y;
}
/******************************************************************************/
