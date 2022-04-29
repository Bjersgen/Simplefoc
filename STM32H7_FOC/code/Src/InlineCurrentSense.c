
#include "MyProject.h"


int pinA,pinB;
float gain_a,gain_b;
float offset_ia,offset_ib;
/******************************************************************************/
void Current_calibrateOffsets(void);
/******************************************************************************/
void InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB)
{
	float volts_to_amps_ratio;
	
	pinA = _pinA;
	pinB = _pinB;
	//pinC = _pinC;
	
	volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
	
	gain_a=volts_to_amps_ratio;
	gain_b=-gain_a;     //B相电流检测反相
	
	//printf("gain_a:%.4f,gain_b:%.4f.\r\n",gain_a,gain_b);
}
/******************************************************************************/
void InlineCurrentSense_Init(void)
{
	Current_calibrateOffsets();   //检测偏置电压，也就是电流0A时的运放输出电压值，理论值=1.65V
}
/******************************************************************************/
// Function finding zero offsets of the ADC
void Current_calibrateOffsets(void)
{
	int i;
	
	offset_ia=0;
	offset_ib=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<1000; i++)
	{
		offset_ia += _readADCVoltage();
		offset_ib += _readADCVoltage();
		HAL_Delay(1); //delay_s(10000);  //555.9us
	}
	// calculate the mean offsets
	offset_ia = offset_ia/1000;
	offset_ib = offset_ib/1000;
	
	//printf("offset_ia:%.4f,offset_ib:%.4f.\r\n",offset_ia,offset_ib);
}
/******************************************************************************/
// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	
	current.a = (_readADCVoltage() - offset_ia)*gain_a;// amps
	current.b = (_readADCVoltage() - offset_ib)*gain_b;// amps
	current.c = 0; // amps
	
	return current;
}
/******************************************************************************/
/*
// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int driverAlign(float voltage)
{
	int exit_flag = 1;
	int i;
	PhaseCurrent_s c,c1;
	float ab_ratio,ba_ratio;
	int tmp_pinA;
	
	// set phase A active and phases B and C down
	setPwm(voltage, 0, 0);
	delay_ms(200); 
	c = getPhaseCurrents();
	// read the current 100 times ( arbitrary number )
	for(i=0; i<100; i++)
	{
		c1 = getPhaseCurrents();
		c.a = c.a*0.6 + 0.4*c1.a;
		c.b = c.b*0.6 + 0.4*c1.b;
		//c.c = c.c*0.6 + 0.4*c1.c;
		delay_ms(3);
	}
	setPwm(0, 0, 0);
	// align phase A
	ab_ratio = fabs(c.a / c.b);
			printf("phasseA,cur_a:%f,cur_b:%f.\r\n",c.a,c.b);
	if( ab_ratio > 1.5 )gain_a *= _sign(c.a); // should be ~2    
	else 
		if( ab_ratio < 0.7 ) // should be ~0.5
		{
			// switch phase A and B
			tmp_pinA = pinA;
			pinA = pinB; 
			pinB = tmp_pinA;
			gain_a *= _sign(c.b);
			exit_flag = 2; // signal that pins have been switched
    }
		else  // error in current sense - phase either not measured or bad connection
			return 0;
	
	//delay_ms(3000); 
	// set phase B active and phases A and C down
	setPwm(0, voltage, 0);
	delay_ms(200); 
	c = getPhaseCurrents();
	// read the current 100 times
	for(i=0; i<100; i++)
	{
		c1 = getPhaseCurrents();
		c.a = c.a*0.6 + 0.4*c1.a;
		c.b = c.b*0.6 + 0.4*c1.b;
		//c.c = c.c*0.6 + 0.4*c1.c;
		delay_ms(3);
	}
	setPwm(0, 0, 0);
	ba_ratio = fabs(c.b/c.a);
			printf("phasseB,cur_a:%f,cur_b:%f.\r\n",c.a,c.b);
	if( ba_ratio > 1.5 )gain_b *= _sign(c.b); // should be ~2
	else 
		if( ba_ratio < 0.7 ) // it should be ~0.5
		{
			// switch phase A and B
			tmp_pinA = pinB;
			pinB = pinA; 
			pinA = tmp_pinA;
			gain_b *= _sign(c.a);
			exit_flag = 2; // signal that pins have been switched
    }
		else  // error in current sense - phase either not measured or bad connection
			return 0;
	
	if(gain_a < 0 || gain_b < 0) exit_flag +=2;
	// exit flag is either
  // 0 - fail
  // 1 - success and nothing changed
  // 2 - success but pins reconfigured
  // 3 - success but gains inverted
  // 4 - success but pins reconfigured and gains inverted
	printf("pinA:%d,pinB:%d.\r\n",pinA,pinB);
	
  return exit_flag;
}
*/
/******************************************************************************/

