/********* Servo control **********
 *
 * A relatively low level driver
 * for controlling servos in a
 * simple manner.
 *
 * For SumoBot movement use
 * "drive.h" instead.
 *
 **********************************/

#ifndef _SERVOS_
#define _SERVOS_ 1

#include <avr/io.h>
#include <avr/interrupt.h>

#define NUM_SERVOS			2

/* Calculated for 8 MHz with prescaler 8 (See Theory in the end) */
#define ICR_VALUE			10000
#define MIN_PULSE_OCR		500			/* 1ms = 1ms/20ms*ICR_VALUE */
#define MAX_PULSE_OCR		1000		/* 2ms = 2ms/20ms*ICR_VALUE */

#define OCR_MIDDLE			750
#define OCR_RANGE			500

/*** Functions ***/

/* Setup for the timer that generates a PWM waveform */
void initialize_servos();

/* Set speed of a specific servo from 0-100 */
void set_speed(uint8_t servo, int8_t speed);

#endif


/* Theory:
 * 
 * Output frequency (fout) is calculated with the formula:
 *
 * fout = clk/(2*N*TOP)
 *
 * Where clk is the system clock, N is the prescaler and
 * TOP is the maximum timer value.
 *
 * The TOP can be any 16-bit value, but the prescaler is always
 * a power of two AND is one the following: 1,8,16,32,64,128.
 * 
 * Ex.  fout = 8000000/(2*8*10000) [Hz]
 *		fout = 50 [Hz]
 */
