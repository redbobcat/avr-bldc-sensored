

#include "buttons.h"

volatile uint8_t buttons_prev=0b10100100; //init buttons state


uint8_t buttons (void) //returns mask of pressed buttons 0b 8th 7th 6th 5th 4th 3d 2d 1st
{
uint8_t b_temp; //temp for return result
b_temp = ~buttons_prev & BUTT_PIN; //mask for result
buttons_prev=BUTT_PIN; //save prev


return b_temp & 0b10100100;
}


void buttons_init (void) //configuring IO ports to in, pull_up to vcc
{
BUTT_DDR &=~((1<<BUTT_1)|(1<<BUTT_2)|(1<<BUTT_3)); //ddr to IN
BUTT_PORT |= (1<<BUTT_1)|(1<<BUTT_2)|(1<<BUTT_3); //port to pull up
}
