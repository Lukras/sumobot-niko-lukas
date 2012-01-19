/********* Line detection *********
 *
 * Functions for using the line
 * sensors.
 *
 * W is connected to Vcc
 * R is read as an input
 * B is connected to Vss
 *
 **********************************/

#ifndef _QTI_
#define _QTI_ 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define QTI_PORT		PORTB
#define QTI_DDR			DDRB
#define QTI_PIN			PINB

#define RR				PB0
#define LR				PB1

#define BLACK_THRES		300

/*** Functions ***/

void initialize_QTI();
uint32_t right_raw();
uint32_t left_raw();

/* Returns positive if sensor is outside the arena */
uint8_t right_outside();
uint8_t left_outside();

#endif
