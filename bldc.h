#ifndef BLDC_H
#define BLDC_H

#include <avr/io.h>
/*
 * BLDC sensored driver.
 * Be careful with all defines, this is for my attached board only,
 * damage may occurs in another schematics
 */
volatile uint16_t ticks;
volatile uint8_t dir;
//out regs for power stage, UH UL VH VL WH WL
//now C port, from 0 to 5
#define PHASE_DDR DDRC
#define PHASE_PORT PORTC
#define PHASE_PIN PINC
#define PHASE_MASK 0b00111111 //may be usefull, MUST BE CHANGED IF SCHEME CHANGED
#define PHASE_STOP_MASK 0b00010101
//in pins for hall sensors outbut. With amp. 
//now PB, 0 to 2 "U V W"
#define HALL_DDR DDRB
#define HALL_PORT PORTB
#define HALL_PIN PINB
#define HALL_MASK 0b00000111 //my be usefull, MUST BE CHANGED IF SCHEME CHANGED

#define PWM_POWER 	OCR0A


//top funcs for bldc, only this must be used in main
void 	bldc_start (uint8_t); //2 for CCW, 0 for none, 1 for CW
void 	bldc_stop (void);
void 	bldc_all_init (void); //all init in one




#endif
