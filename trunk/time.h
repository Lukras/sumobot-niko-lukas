/********* System time ************
 *
 * A NOT completely inaccurate way
 * of keeping time. This has enough
 * precision for simple timing.
 *
 * Both timer 0 and 2 can be used
 * for this, but timer 2 can be
 * prescaled with an asynchronous
 * clock source, making it slightly
 * more accurate.
 *
 **********************************/

#ifndef _TIME_
#define _TIME_ 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*** Functions ***/

/* Set the timer to count milliseconds */
void initialize_time();

/* Delay for given milliseconds */
void delay(uint32_t ms);

void start_timer();
void stop_timer();

/* Returns the difference between the "start_o" timestamp
 * and the current time. Updates the parameter "start_o" to
 * the current system time automatically
 */
uint32_t time_delta(uint32_t* start_o);

/* Returns the time elapsed since t_start
 * without updating t_start
 */
uint32_t time_since(uint32_t t_start);

#endif
