/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for tinyrc.c.
 *
 * - File:               tinyrc.h
 * - Compiler:           GCC 4
 * - Supported devices:  ATTiny45 ATTiny85
 *
 * \author               Stephan Harms: http://www.stephanharms.de \n
 *                       Support email: avr@stephanharms.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.2 $
 * $RCSfile: timer.h,v $
 * $Date: 2010/04/05 12:25:58 $
 *****************************************************************************/




#ifndef TINYRC_H
#define TINYRC_H


#define CH_ORDER_IN_ADR  0
#define CH_ORDER_OUT_ADR 8
#define MAX_OUT_CH       8
#define MAX_IN_CH        8

#define SEQU_DELAY 0
#define CH_DELAY   1
#define PIN_HI     2

#define IN_SEQU_DELAY  0
#define IN_CH_DELAY    1
#define IN_PIN_HI      2

#define BUFFER_IN_SIZE 4

#define OUTPUT_DDR_1  DDRB
#define OUTPUT_DDR_2  DDRB
#define OUTPUT_DDR_3  DDRB
#define OUTPUT_DDR_4  DDRB
#define OUTPUT_DDR_5  DDRB


#define OUTPUT_PORT_1 PORTB
#define OUTPUT_PORT_2 PORTB
#define OUTPUT_PORT_3 PORTB
#define OUTPUT_PORT_4 PORTB
#define OUTPUT_PORT_5 PORTB

#define OUTPUT_BIT_1  PB1
#define OUTPUT_BIT_2  PB2
#define OUTPUT_BIT_3  PB3
#define OUTPUT_BIT_4  PB4
#define OUTPUT_BIT_5  PB5


#define INPUT_BIT_1   PB1
#define INPUT_BIT_2   PB2
#define INPUT_BIT_3   PB3
#define INPUT_BIT_4   PB4
#define INPUT_BIT_5   PB5

#define INPUT_PIN_1   PINB
#define INPUT_PIN_2   PINB
#define INPUT_PIN_3   PINB
#define INPUT_PIN_4   PINB
#define INPUT_PIN_5   PINB






#define INPUT_PORT_1   PORTB
#define INPUT_PORT_2   PORTB
#define INPUT_PORT_3   PORTB
#define INPUT_PORT_4   PORTB
#define INPUT_PORT_5   PORTB


#define TEST_DDR      DDRB
#define TEST_PORT     PORTB
#define TEST_BIT      PB1

#define gyroCh 7


#define MAX_PULSE_TIME 3000


#define TIMER_0_MAX_CYCLE 10


void ReadInBuffer(void);

void GyroOut(void);



#endif // TINYRC_H
