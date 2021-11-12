
#include "MyProject.h"

/************************************************
本程序仅供学习，未经作者许可，不得用于其它任何用途
FOC变换
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210801
************************************************/
/******************************************************************************/
// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided
float getDCCurrent(float motor_electrical_angle)
{
	PhaseCurrent_s current;
	float sign=1;       // currnet sign - if motor angle not provided the magnitude is always positive
	float i_alpha, i_beta;
	
	// read current phase currents
	current = getPhaseCurrents();
	
	// calculate clarke transform
	i_alpha = current.a;
	i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	
	// if motor angle provided function returns signed value of the current
	// determine the sign of the current
	// sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
	if(motor_electrical_angle)sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
	// return current magnitude
	return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
}
/******************************************************************************/
// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents internally
DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// read current phase currents
	current = getPhaseCurrents();
	
	// calculate clarke transform
	i_alpha = current.a;  
	i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	
	// calculate park transform
	ct = _cos(angle_el);
	st = _sin(angle_el);
	
	ret.d = i_alpha * ct + i_beta * st;
	ret.q = i_beta * ct - i_alpha * st;
	return ret;
}
/******************************************************************************/
