/*
 * 01.2016 
 * 
 * 
 * 
 * 
 */
#include "bldc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

//funcs for bldc drive, static
static inline uint8_t bldc_hall_state (void); //returns hall sensor state
//static void			 bldc_out_set (uint8_t); //closes all gates and sets all to next
static uint8_t 		 bldc_switch (uint8_t hallin, uint8_t direction); //2 for CCW, 0 for none, 1 for CW
static const uint8_t bldc_state[] = {0b00100001, 0b00001001, 0b00011000, 0b00010010, 0b00000110, 0b00100100}; //output table for switching
static inline void 	 bldc_out_init (void); //bldc out port configure, output, sets to 0
static inline void 	 bldc_hall_init (void); //bldc hall in configure, input, hi-z
static void 		 bldc_pwm_enable (void); //pwm enabling on PD6 pin, ~31KHz


	//enable pinchange interrupt for PB0-PB2
static inline void		bldc_interupt_enable (void);
static inline void		bldc_interupt_disable (void);
	// ISR of pinchange interupt is in bldc.c



/*
 * BLDC sensored
 * 
 * 01.2016
 * redbobcat
 */

static inline void bldc_out_init (void)
	{
		PHASE_DDR |= PHASE_MASK;
		PHASE_PORT &= ~(PHASE_MASK);
	}
	
static inline void bldc_hall_init (void)
	{
		HALL_DDR &= ~(HALL_MASK);
	}
	
static inline uint8_t bldc_hall_state (void)
	{
	return HALL_PIN & HALL_MASK;
	}
/*void bldc_out_set (uint8_t in)
	{
	PHASE_PORT &= ~(PHASE_MASK);
	PHASE_PORT |= in;
	}
*/
static uint8_t bldc_switch (uint8_t hallin, uint8_t direction)
	{	uint8_t state_temp=0;
		
		ticks++;
		
		if (direction==0)
			{
				return 0;
			}
		if (direction==1) //CW
			{
		
		switch (hallin)
				{
				case 0b00000101: {state_temp=bldc_state[3+direction-1]; break;}
				case 0b00000001: {state_temp=bldc_state[4+direction-1]; break;}
				case 0b00000011: {state_temp=bldc_state[5+direction-1]; break;}
				case 0b00000010: {state_temp=bldc_state[0+direction-1]; break;}
				case 0b00000110: {state_temp=bldc_state[1+direction-1]; break;}
				case 0b00000100: {state_temp=bldc_state[2+direction-1]; break;}
				}
			}
			
		if (direction==2) //CCW must be changed!!! CHANGED, test it!
			{
				switch (hallin)
				{
				case 0b00000101: {state_temp=bldc_state[5+direction-1]; break;}
				case 0b00000001: {state_temp=bldc_state[0+direction-1]; break;}
				case 0b00000011: {state_temp=bldc_state[1+direction-1]; break;}
				case 0b00000010: {state_temp=bldc_state[2+direction-1]; break;}
				case 0b00000110: {state_temp=bldc_state[3+direction-1]; break;}
				case 0b00000100: {state_temp=bldc_state[4+direction-1]; break;}
				}
			}
				
		return state_temp;	
	}


inline void bldc_start (uint8_t cwccw)
	{
			PHASE_PORT=bldc_switch(bldc_hall_state(),cwccw);
			PWM_POWER = 255;
			bldc_interupt_enable();
			sei();
	}
	
inline void bldc_stop(void)
	{
			
			bldc_interupt_disable();
			PHASE_PORT &= ~(PHASE_MASK);
			//PHASE_PORT |= PHASE_STOP_MASK;
	}

static inline void bldc_interupt_enable (void)
	{
			PCICR |= 1<<PCIE0;
			PCMSK0|= HALL_MASK;
	}
	
static inline void	bldc_interupt_disable (void)
	{
			PCICR &= ~(1<<PCIE0);
			PCMSK0 &= ~(HALL_MASK);
	}

static void bldc_pwm_enable (void)
	{
			TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01); //set fast-pwm, non-inverting out
			DDRD |= 1<<PD6;
			TCCR0B |= 1<<CS00; //start at full speed
			
	}

inline void bldc_all_init(void)
	{
		bldc_hall_init();
		bldc_out_init();
		bldc_pwm_enable();
		
	}
	
ISR(PCINT0_vect)
	{		
			PHASE_PORT=bldc_switch(bldc_hall_state(), dir);
			//USART_Transmit(0x30+bldc_hall_state());
			sei();
	}
