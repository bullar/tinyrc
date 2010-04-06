/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief PinChange Interrupt handler
 *
 * TinyRC is a smart RC controller board for mixing signal at the receiver side
 *
 * - File:               pinchange.c
 * - Compiler:           AVRGCC
 * - Supported devices:  AVR devices with pin change interrupts
 *
 * \author               Stephan Harms
 *                       email: avr@stephanharms.de
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.0 $
 * $RCSfile: pincange,v $
 * $Date: 2010/04/05 $
 *****************************************************************************/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/interrupt.h>
//#include <util/delay.h>

#include "tinyrc.h"
#include "timer.h"


//! lastPCTime
uint16_t lastPCTime;

//! the current output channel
uint8_t  curCh = 0;


//! the order of input channels
uint8_t ch_order_in[MAX_IN_CH];

//! the order channels to output
uint8_t ch_order_out[MAX_OUT_CH];

//! state machine status for writing output
uint8_t outState = SEQU_DELAY;



//! channel intermediate delay
uint16_t interChDelay = 500;
uint16_t sequDelay =  10000;


//! input channel data
uint16_t inData[MAX_OUT_CH]= {1000,1000,1000,1000,1000,1000,1000,1000};

//! the current input channel
uint8_t  curInCh = 0;



uint16_t timeBuffer[BUFFER_IN_SIZE];
uint8_t  pinBuffer[BUFFER_IN_SIZE][2];
uint8_t  writeBufferPos = 0;
uint8_t  readBufferPos = 0;

uint8_t timerHi;

uint8_t inState = IN_SEQU_DELAY;

uint16_t lastTime;


volatile uint8_t *outPort[MAX_OUT_CH];
uint8_t           outBit[MAX_OUT_CH];

volatile uint8_t *inPin[MAX_IN_CH];
uint8_t           inBit[MAX_IN_CH];

uint16_t lastPCTime;

uint16_t outData[MAX_OUT_CH] = {1000,1000,1000,1000,1000,1000,1000,1000};



uint8_t inSequ[MAX_IN_CH]      = {4,0,3,2,5,1,7,6};
uint8_t outSequ[MAX_OUT_CH]    = {0,1,2,3,4,5,6,7};




//#define OUTPUT_DDR  DDRB
//#define OUTPUT_PORT PORTB
//#define OUTPUT_BIT  PB0


/*! \brief Load settings from flasch
 *
 * the function 'LoadSettings' is executed during startup 
 * it reads settings from flash memory
 *
 *  \param none
 *  \return  none
 */
void LoadSettings(void)
{
  // read order of input ch from eprom
  //eeprom_read_block(ch_order_in,  (void *) CH_ORDER_IN_ADR,  MAX_OUT_CH);
  //eeprom_read_block(ch_order_out, (void *) CH_ORDER_OUT_ADR, MAX_OUT_CH);

}




/*! \brief Enables the Pin Change Interrupt
 *
 * the function 'InitPinChange' is executed during startup 
 * it enables the pin change interrupt
 *
 *  \param none
 *  \return  none
 */
void InitPinChange(void)
{

  // set pin change mask
  PCMSK1 = (1<<PCINT10)|(1<<PCINT11)|(1<<PCINT12)|(1<<PCINT13);
  PCMSK2 = (1<<PCINT16)|(1<<PCINT17)|(1<<PCINT18)|(1<<PCINT19);

 // PCMSK2 = (1<<PCINT17);

  // enable pin change interrupt
  PCICR =  (1<<PCIE2)|(1<<PCIE1);

}

void PinChangeInterrupt(void)
{
  uint16_t time;
  uint16_t timeDiff;
  uint8_t pinStatus;

  time = TCNT1;

  // calc time difference
  timeDiff = time - lastTime;

  //curInCh = 4; // trottle


  pinStatus = *inPin[inSequ[curInCh]];
  pinStatus >>= inBit[inSequ[curInCh]];
  pinStatus &= 1; 


  // check if first pulse in sequence
  if(timeDiff >= MAX_PULSE_TIME)
  {
    inState = IN_SEQU_DELAY;
	curInCh = 0;
  }
  //else
  //  inData[curInCh] = timeDiff;



  if(pinStatus) // pin hi
  {
    // set status
    inState = IN_PIN_HI;
  }
  else // pin low 
  { 

    // save time
    inData[curInCh] = timeDiff;
	inState = IN_CH_DELAY;

    // go to next ch
	curInCh++;
  }

//  if(curInCh >= MAX_IN_CH)
  if(curInCh >= 6)
  {
    curInCh = 0;
    inState = IN_SEQU_DELAY;

  }

  //  for(i=0;i<6;i++)
  //    inData[i] = 5000;


  //inData[0]++;  

  // save time
  lastTime = time;
}



// pin change interrupt
/*! \brief pin change interrupt
 *
 * the ISR 'PinCange0' is executed on a pin change 
 * of all pin of port ? on which the ?? is set
 *
 *  \param none
 *  \return  none
 */
ISR(PCINT0_vect)
{
  //PinChangeInterrupt();

  // save time
  timeBuffer[writeBufferPos] = TCNT1;
  
  // save pin state
//  pinBuffer[writeBufferPos][0] = PINC; 
//  pinBuffer[writeBufferPos][1] = PIND;
  
  // update writeBuffer pos
  writeBufferPos++;

  // set new buffer pos
  if(writeBufferPos >= BUFFER_IN_SIZE)
    writeBufferPos = 0;

	
}




/*! \brief main function 
 *
 *  The main loop function loads settings from flash,
 *  sets the DD of all pins and finally enters the
 *  forever loop
 *
 */
int main(void)
{
  uint16_t i;


  //LoadSettings();
  
  InitPort();

  InitTimer();

//  InitPinChange();

  // global interrupt enable 
  sei();

  // while forever
  while(1)
  {

//    delay(1);
  /*
    for(i=0;i<6;i++)
	{
	  //cli();
	  outData[i] = inData[i];
	  
      //outData[i] = 1500;
	  //sei();

	  ReadInBuffer();

	  GyroOut();

	  }
  */
	
	//*outPort[0] ^= (1<<outBit[0]);

  }
  
  // this function will never been executed
  return(0);

}

/*
void ReadInBuffer(void)
{

  uint16_t timeDiff;
  uint8_t pinStatus;


  // new trigger events
  if(writeBufferPos != readBufferPos)
  {
      
	// calc time difference
    timeDiff = timeBuffer[readBufferPos] - lastTime;
    
	
	if(inSequ[curInCh] < 4)
	  pinStatus = pinBuffer[readBufferPos][0]; // PORTC
	else
	  pinStatus = pinBuffer[readBufferPos][1]; //PORTD

    switch(curInCh)
	{
	case 



	}
*/
/*
    pinStatus >>= inBit[inSequ[curInCh]];
    pinStatus &= 1; 


    // check if first pulse in sequence
  if(timeDiff >= MAX_PULSE_TIME)
  {
    inState = IN_SEQU_DELAY;
	curInCh = 0;
  }
  //else
  //  inData[curInCh] = timeDiff;



  if(pinStatus) // pin hi
  {
    // set status
    inState = IN_PIN_HI;
  }
  else // pin low 
  { 

    // save time
    inData[curInCh] = timeDiff;
	inState = IN_CH_DELAY;

    // go to next ch
	curInCh++;
  }

//  if(curInCh >= MAX_IN_CH)
  if(curInCh >= 6)
  {
    curInCh = 0;
    inState = IN_SEQU_DELAY;

  }

  //  for(i=0;i<6;i++)
  //    inData[i] = 5000;


  //inData[0]++;  

// OPTIMIZE IT!!
  // save time
  lastTime = timeBuffer[readBufferPos];

  readBufferPos++;
  // set new buffer pos
    if(readBufferPos >= BUFFER_IN_SIZE)
      readBufferPos = 0;


  }



}
*/
/*

void GyroOut(void)
{
  uint16_t time;


  volatile uint8_t *gyroOutPort = outPort[gyroCh];


  // set gyro pin hi
  *gyroOutPort |= (1 << outBit[gyroCh]);

  // read time
  time = TCNT1;

  // calc gyro time
  time += 1500;

  // set output compare1B
  OCR1B = time;

  // start timer

}

*/








