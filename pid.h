#ifndef PID_H
#define PID_H

#define Kp 1
#define Ki 0.1
#define Kd 0.1

#define SPEED_MIN 300 //y min
#define SPEED_MAX 20000 //y max

#define PWM_MIN 70 //u min
#define PWM_MAX 255 //u max

#define DELTA_MAX SPEED_MAX //max error
#define INTEGRAL_MAX 20000
#define INTEGRAL_MIN -(INTEGRAL_MAX)

#define Y_MAX (DELTA_MAX*Kp)+(INTEGRAL_MAX)+(DELTA_MAX*Kd)
#define Y_MIN -(U_MAX)

#define CHANGE_STEP (Y_MAX/255)



int16_t PID (int16_t delta);



#endif
