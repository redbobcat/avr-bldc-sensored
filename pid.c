#include "pid.h"

volatile int16_t delta_prev;

int16_t PID (int16_t delta)
	{
		double_t U, I, D;
		
		U  = delta * Kp;
		
		I += delta * Ki;
			if (I>INTEGRAL_MAX) I=INTEGRAL_MAX;
			if (I<INTEGRAL_MIN) I=INTEGRAL_MIN;
		
		D = (delta - delta_prev)*Kp;
		
		U += I + D;
		
		delta_prev=delta;
		
		return (U/CHANGE_STEP);
	}
