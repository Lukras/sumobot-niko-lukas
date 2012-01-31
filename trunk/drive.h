/********* Motor control **********
 *
 * Functions for controlling the
 * SumoBot movement.
 *
 * Note that the right servo is
 * installed mirror to the left
 * servo. The values of the right
 * servo are therefore inversed
 * when they are to be the same
 * as the left uninversed.
 *
 **********************************/

#ifndef _DRIVE_
#define _DRIVE_ 1

#include <avr/io.h>
#include "servos.h"

/* Hardware device IDs */
#define RIGHT_WHEEL		0
#define LEFT_WHEEL		1

/*** Macros ***/

/* Reverse curving to a direction; Define the turn steepness from 0-100 */
#define reverse_left(SPEED, STEEP) (turn_right(-SPEED, STEEP))
#define reverse_right(SPEED, STEEP) (turn_left(-SPEED, STEEP))

/*** Functions ***/

/* Move in a straight line */
void drive_forward(int8_t speed);
void drive_backward(int8_t speed);

void stop();

/* Spin in place to a direction with a given speed from 0-100 */
void spin_right(int8_t speed);
void spin_left(int8_t speed);

/* Turn to a direction; Define the turn steepness from 0-100 */
void turn_right(int16_t speed, int16_t steep);
void turn_left(int16_t speed, int16_t steep);

#endif
