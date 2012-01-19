/********* SumoBot ***************
 *
 * The Bot.
 *
 * Drivers:
 *    drive.h		Movement
 *       servos.h	Servo control
 *    qti.h			Line detection
 *    time.h		System time
 *
 * TODO:
 * Infrared receiver/transmitter
 * Movement patterns
 * QTI line sensing
 *
 * Authors: Niko Nieminen
 *			Lukas Kern
 *
 *********************************/

#include <avr/io.h>
#include "time.h"
#include "drive.h"
#include "qti.h"

#define LED_PORT 	PORTD
#define LED_DDR		DDRD
#define LD1			PD0
#define LD2			PD1

#define ON			1
#define OFF			0

/********* Main functions **********
 *
 * Main level functions that do not
 * have their own driver
 *
 *********************************/

void initialize_CPU();
void initialize_LEDs();
void turn_LED1(uint8_t value);
void turn_LED2(uint8_t value);

/********* Main program **********/

int main(){
	initialize_CPU();
	initialize_LEDs();
	initialize_servos();
	initialize_QTI();
	initialize_time();

	const uint16_t stage=100;

	while (1){
		if (left_outside()){
			spin_right(10);
			delay(1000);
		} else if (right_outside()){
			spin_left(10);
			delay(1000);
		} else {
			drive_forward(10);
		}

		delay(stage);
	}
}

/*** Function implementations ***/

/* Set clock frequency to 8 MHz */
void initialize_CPU(){
	cli();

	CLKPR = 0;

	/* Enable changing the clock prescaler */
	CLKPR |= (1<<CLKPCE);

	/* No scaling; This is only for reference */
	CLKPR |= (0<<CLKPS3)|(0<<CLKPS2)|(0<<CLKPS1)|(0<<CLKPS0);

	sei();
}

void initialize_LEDs(){
	LED_DDR |= (1<<LD1)|(1<<LD2);
	LED_PORT &= ((1<<LD1)|(1<<LD2));
}

void turn_LED1(uint8_t value){
	LED_PORT = (value) ? LED_PORT|(1<<LD1) : LED_PORT&(~(1<<LD1));
}

void turn_LED2(uint8_t value){
	LED_PORT = (value) ? LED_PORT|(1<<LD2) : LED_PORT&(~(1<<LD2));
}
