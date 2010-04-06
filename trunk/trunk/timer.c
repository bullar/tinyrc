/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief TinyRC - Smart RC controller
 *
 * Hausbus Controller for Hut-Montage
 *
 * - File:               timer
 * - Compiler:           AVRGCC
 * - Supported devices:  ATTiny45
 *
 * \author               Stephan Harms
 *                       email: avr@stephanharms.de
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.0 $
 * $RCSfile: timer,v $
 * $Date: 2010/04/05 $
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "tinyrc.h"

// the values are calculated with AvrWiz for ATmega88 @ 20 mhz
// Timer0 settings: 0 ticks (= 0 us)


extern volatile uint8_t *outPort[MAX_OUT_CH];
extern uint8_t           outBit[MAX_OUT_CH];


extern uint8_t  outState;
extern uint16_t interChDelay;
extern uint16_t sequDelay;

uint16_t lastTimerVal;
//uint16_t outData[MAX_OUT_CH] = {100,200,300,400,500,600,700,800};

extern uint16_t outData[MAX_OUT_CH];
extern uint8_t  curCh;


uint8_t timer0Cycle = 0;


void InitTimer(void)
{
  	Timer0_init();
	//Timer1_init();

}



// Timer0 Overflow
ISR(TIMER0_OVF_vect)
{
	// reinit counter
	//TCNT0 = 0;

  timer0Cycle++;

  if(timer0Cycle == 1)
  {
    // disable timer0 compare A and B
	TCCR0A &= ~((1 << COM0A1)|(1 << COM0A0)|(1 << COM0B1)|(1 << COM0B0));


  }

  if(timer0Cycle >= TIMER_0_MAX_CYCLE)
  {
    // enable timer0 compare A and B
    TCCR0A |= (1 << COM0A1)|(1 << COM0A0)|(1 << COM0B1)|(1 << COM0B0);


    // reset counter 
    timer0Cycle = 0;
  }

}

// Timer0 Compare A
ISR(TIMER0_COMPA_vect)
{
}

// Timer0 Compare B
ISR(TIMER0_COMPB_vect)
{
}

void Timer0_init()
{
  // set timer0 to mode 3
  // enable set bits on output compare
  TCCR0A = (1 << COM0A1)|(1 << COM0A0)|(1 << COM0B1)|(1 << COM0B0)|(1 << WGM01)|(1 << WGM00);


	// init counter
	TCNT0 = 0x00;

    // init output compare
	OCR0A = 0x40; // 1.5 ms
    OCR0B = 0x40; // 1.5 ms

    //OCR0B = 0xC0; // 1 ms

    // enable interrupts
 	TIMSK |= (1 << TOIE0);

     // start Timer0 
    TCCR0B = (1 << CS01)|(1 << CS00); // prescaler = 64

}





// Timer1 Overflow
ISR(TIMER1_OVF_vect)
{

    


	// reinit counter
//	TCNT1H = 0;
//	TCNT1L = 0;

	// toggle bit
//    TEST_PORT ^= (1<<TEST_BIT);
//	TIFR1   |= (1<<TOV1);


}



// Timer1 Compare A
ISR(TIMER1_COMPA_vect)
{
  uint16_t nextTimerVal;
  static uint16_t lastTimerVal;

  volatile uint8_t *thisOutPort = outPort[curCh];
  uint8_t           thisOutBit  = outBit[curCh];

  // toggle bit
  //TEST_PORT ^= (1<<TEST_BIT);

  
  switch(outState)
  {
  case PIN_HI:
    // set pin low
    *thisOutPort &= ~(1<<thisOutBit);
    
	curCh++;
	//if(curCh>=MAX_OUT_CH)
	if(curCh>=7)
	{
	  curCh = 0;
	  outState = SEQU_DELAY;
	  nextTimerVal = lastTimerVal + sequDelay;
    }
	else	
	{
	  outState = CH_DELAY;
	  // set next comepare macht to channel delay
      nextTimerVal = lastTimerVal + interChDelay;
	}

	break;
  case CH_DELAY:
  case SEQU_DELAY:
    // set pin hi
    *thisOutPort |= (1<<thisOutBit);

	// set timer to 0
	//TCNT1H = 0;
	//TCNT1L = 0;


    nextTimerVal = lastTimerVal + (uint16_t) outData[curCh];


	outState = PIN_HI;

   
  }
  

  // save last timer value
  lastTimerVal = nextTimerVal;

//  OCR1AH = 0xFF & (nextTimerVal >> 8);
//  OCR1AL = 0xFF & nextTimerVal; 

  OCR1A = nextTimerVal;



}



// Timer1 Compare B
ISR(TIMER1_COMPB_vect)
{


  volatile uint8_t *gyroOutPort = outPort[gyroCh];


  // set gyro pin low
  *gyroOutPort *= ~(1 << outBit[gyroCh]);

}

void Timer1_init()
{

	// init counter
//	TCNT1H = 0;
//	TCNT1L = 0;

   // OCR1AH = 0xFF & (outData[0] >> 8);
   // OCR1AL = 0xFF &  outData[0]; 


//	TIMSK1 = (1<<OCIE1A) | (1<<OCIE1B); // Timer1 CompareA + CompareB Interrupt Enable

//	TIMSK1 = (1<<TOIE1); // Timer1 CompareA Interrupt Enable

    // start timer
//	TCCR1B = (1<<CS11) ; // prescaler = 8



}



