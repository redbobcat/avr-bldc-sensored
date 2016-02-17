#include "pid.h"

volatile uint16_t delta_prev;
int16_t I;

int16_t PID (int16_t delta)
	{
		int32_t U;
		
		U  = delta * Kp;
		
		I += delta * Ki;
			if (I>INTEGRAL_MAX) I=INTEGRAL_MAX;
			if (I<INTEGRAL_MIN) I=INTEGRAL_MIN;
		
		U += (delta - delta_prev)*Kp;
		
		U += I;
		
		delta_prev=delta;
		
		return (U/CHANGE_STEP);
	}
