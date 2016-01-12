#include "pid.h"



int16_t delta_prev;

uint8_t PID (int16_t delta)
	{
		double_t U, I;
		
		U  = delta * Kp;
		
		I += delta * Ki;
			if (I>INTEGRAL_MAX) I=INTEGRAL_MAX;
			if (I<INTEGRAL_MIN) I=INTEGRAL_MIN;
		
		D = (delta - delta_prev)*Kp;
		
		U += I + D;
		
		delta_prev=delta;
	}
