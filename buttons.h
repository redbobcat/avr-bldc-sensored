#ifndef BUTTONS_H
#define BUTTONS_H

#include <avr/io.h>

#define BUTT_PORT PORTD //port for button
#define BUTT_DDR DDRD //ddr for button
#define BUTT_PIN PIND //pin for button
#define BUTT_1 2 //first button bit
#define BUTT_2 5 //second button bit
#define BUTT_3 7 //third button bit
//#define BUTT_4 3 //four button bit





uint8_t buttons (void); //returns mask of pressed buttons 0b 8th 7th 6th 5th 4th 3d 2d 1st

void buttons_init (void); //configuring IO ports to in, pull_up to vcc


#endif
