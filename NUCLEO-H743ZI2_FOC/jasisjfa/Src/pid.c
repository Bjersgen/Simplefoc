

#include "MyProject.h"

/************************************************
本程序仅供学习，未经作者许可，不得用于其它任何用途
PID参数初始化、角度PID、速度PID、电流PID
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210801
************************************************/
/******************************************************************************/
float pid_vel_P, pid_ang_P, pid_cur_P;
float pid_vel_I, pid_ang_D, pid_cur_I;
float integral_vel_prev, integral_cur_q_prev, integral_cur_d_prev;
float error_vel_prev, error_ang_prev, error_cur_q_prev, error_cur_d_prev;
float output_vel_ramp,output_cur_ramp;
float output_vel_prev,output_cur_q_prev, output_cur_d_prev;
unsigned long pid_vel_timestamp, pid_ang_timestamp, pid_cur_q_timestamp, pid_cur_d_timestamp;
/******************************************************************************/
void PID_init(void)
{
	pid_vel_P=0.1;  //0.2
	pid_vel_I=2;    //20
	output_vel_ramp=100;       // output derivative limit [volts/second]
	integral_vel_prev=0;
	error_vel_prev=0;
	output_vel_prev=0;
	pid_vel_timestamp=SysTick->VAL;
	
	pid_ang_P=10;
	pid_ang_D=0.5;
	error_ang_prev=0;
	pid_ang_timestamp=SysTick->VAL;
	
	pid_cur_P=2;   
	pid_cur_I=1; 
	output_cur_ramp=10;
	integral_cur_q_prev=0;
	error_cur_q_prev=0;
	output_cur_q_prev=0;
	pid_cur_q_timestamp=SysTick->VAL;
	
	integral_cur_d_prev=0;
	error_cur_d_prev=0;
	output_cur_d_prev=0;
	pid_cur_d_timestamp=SysTick->VAL;
}
/******************************************************************************/
//just P&I is enough,no need D
float PID_velocity(float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,integral,output;
	float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us<pid_vel_timestamp)Ts = (float)(pid_vel_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + pid_vel_timestamp)/9*1e-6f;
	pid_vel_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = pid_vel_P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = integral_vel_prev + pid_vel_I*Ts*0.5*(error + error_vel_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -voltage_limit, voltage_limit);
	
	// sum all the components
	output = proportional + integral;
	// antiwindup - limit the output variable
	output = _constrain(output, -voltage_limit, voltage_limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - output_vel_prev)/Ts;
	if(output_rate > output_vel_ramp)output = output_vel_prev + output_vel_ramp*Ts;
	else if(output_rate < -output_vel_ramp)output = output_vel_prev - output_vel_ramp*Ts;
	
	// saving for the next pass
	integral_vel_prev = integral;
	output_vel_prev = output;
	error_vel_prev = error;
	
	return output;
}
/******************************************************************************/
//P&D for angle_PID
float PID_angle(float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,derivative,output;
	//float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us<pid_ang_timestamp)Ts = (float)(pid_ang_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + pid_ang_timestamp)/9*1e-6f;
	pid_ang_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = pid_ang_P * error;
	// u_dk = D(ek - ek_1)/Ts
	derivative = pid_ang_D*(error - error_ang_prev)/Ts;
	
	output = proportional + derivative;
	output = _constrain(output, -velocity_limit, velocity_limit);
	
	// limit the acceleration by ramping the output
//	output_rate = (output - output_ang_prev)/Ts;
//	if(output_rate > output_ang_ramp)output = output_ang_prev + output_ang_ramp*Ts;
//	else if(output_rate < -output_ang_ramp)output = output_ang_prev - output_ang_ramp*Ts;
	
	// saving for the next pass
//	output_ang_prev = output;
	error_ang_prev = error;
	
	return output;
}
/******************************************************************************/
//P&I for current_PID
float PID_current_q(float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,integral,output;
	float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us<pid_cur_q_timestamp)Ts = (float)(pid_cur_q_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + pid_cur_q_timestamp)/9*1e-6f;
	pid_cur_q_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = pid_cur_P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = integral_cur_q_prev + pid_cur_I*Ts*0.5*(error + error_cur_q_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -voltage_power_supply, voltage_power_supply); 
	
	// sum all the components
	output = proportional + integral;
	// antiwindup - limit the output variable
	output = _constrain(output, -voltage_limit, voltage_limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - output_cur_q_prev)/Ts;
	if(output_rate > output_cur_ramp)output = output_cur_q_prev + output_cur_ramp*Ts;
	else if(output_rate < -output_cur_ramp)output = output_cur_q_prev - output_cur_ramp*Ts;
	
	// saving for the next pass
	integral_cur_q_prev = integral;
	output_cur_q_prev = output;
	error_cur_q_prev = error;
	
	return output;
}
/******************************************************************************/
//P&I for current_PID
float PID_current_d(float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,integral,output;
	float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us<pid_cur_d_timestamp)Ts = (float)(pid_cur_d_timestamp - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + pid_cur_d_timestamp)/9*1e-6f;
	pid_cur_d_timestamp = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = pid_cur_P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = integral_cur_d_prev + pid_cur_I*Ts*0.5*(error + error_cur_d_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -voltage_power_supply, voltage_power_supply); 
	
	// sum all the components
	output = proportional + integral;
	// antiwindup - limit the output variable
	output = _constrain(output, -voltage_limit, voltage_limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - output_cur_d_prev)/Ts;
	if(output_rate > output_cur_ramp)output = output_cur_d_prev + output_cur_ramp*Ts;
	else if(output_rate < -output_cur_ramp)output = output_cur_d_prev - output_cur_ramp*Ts;
	
	// saving for the next pass
	integral_cur_d_prev = integral;
	output_cur_d_prev = output;
	error_cur_d_prev = error;
	
	return output;
}
/******************************************************************************/

