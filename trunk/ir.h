/********* IR detection ***********
 *
 * IR LED and detector driver.
 *
 * Samples the IR detector
 * IR_SAMPLES times in IR_INTERVAL
 * ms intervals. If IR_THRESHOLD hits
 * out of IR_SAMPLES is achieved,
 * obstacle functions return true.
 *
 **********************************/

#ifndef _IR_
#define _IR_ 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include "time.h"

#define DET_PORT		PORTD
#define DET_DDR			DDRD
#define DET_PIN			PIND
#define DET_RIGHT		PD0
#define DET_LEFT		PD1

/* Original is 104 */
/* Good = 90, 93, 100 */
#define IR_SENSITIVITY	93

#define IR_SAMPLES		3
#define IR_INTERVAL		1	// ms
#define IR_THRESHOLD	2

void initialize_IR_LEDs();
void initialize_IR_detectors();
uint8_t obstacle_right();
uint8_t obstacle_left();

#endif
