/********* SumoBot ***************
 *
 * The Bot.
 *
 * Drivers:
 *    drive.h		Movement
 *       servos.h	Servo control
 *    qti.h			Line detection
 *    time.h		System time
 *    ir.h			Obstacle detect
 *
 * Authors: Niko Nieminen
 *			Lukas Kern
 *
 *********************************/

#include <avr/io.h>
#include "time.h"
#include "drive.h"
#include "qti.h"
#include "ir.h"

#define LED_PORT 	PORTB
#define LED_DDR		DDRB
#define LD1			PB2
#define LD2			PB3

#define ON			1
#define OFF			0

/* Function array mode id index values */
#define SEARCH		0
#define ATTACK		1
#define BORDER		2

/* Position constants */
#define FRONT		0
#define RIGHT		1
#define LEFT		2

/* Attack PID constants */
#define KP			60
#define KI			7
#define MIN			25
#define TIMEOUT		1200

/* Search constants */
#define SWEEP_MS	600

/********* Main functions **********
 *
 * Main level functions that do not
 * have their own driver.
 *
 *********************************/

void initialize_CPU();
void initialize_LEDs();
void turn_LED1(uint8_t value);
void turn_LED2(uint8_t value);
inline uint32_t sat_substract(uint32_t a, uint32_t b);

/********* Logic patterns *********
 *
 * Different modes of operation for
 * the bot. The modes must return
 * one of the defined mode IDs.
 *
 *********************************/

uint8_t search();
uint8_t attack();
uint8_t border();

/********* Main program **********/

int main(){
	initialize_CPU();
	initialize_LEDs();
	initialize_time();
	initialize_servos();
	initialize_QTI();
	initialize_IR_LEDs();
	initialize_IR_detectors();

	stop();
	delay(1000);

	/* The Sumobot must initially be on black playarea */
	calibrate_QTI();

	uint8_t current_stage=SEARCH;
	uint8_t (*action[])() = {search, attack, border};

	/* The current stage action recommends (returns) a new
	 * stage (id) for the next turn.
	 */
	start_timer();
	while (1){
		current_stage = action[current_stage]();
	}
}

/*** Function implementations ***/

uint8_t search(){
	uint32_t start_time=0, stage_duration=50;

	uint32_t t=0;
	uint32_t timestamp=0;
	uint8_t dir = RIGHT;

	time_delta(&start_time);
	while (1){
		t += (timestamp != 0) ? time_delta(&timestamp) : 0&time_delta(&timestamp);
		if (t > SWEEP_MS){
			t = 0;
			dir = (dir==RIGHT) ? LEFT : RIGHT; 
		}
		if (dir==RIGHT)
			turn_right(100, 100);
		else
			turn_left(100, 100);
	
		if (left_outside() || right_outside())
			return BORDER;

		if (obstacle_right() || obstacle_left())
			return ATTACK;

		delay(sat_substract(stage_duration, time_delta(&start_time)));
	}

	return SEARCH;
}

uint8_t attack(){
	const uint16_t stage_duration=50; // ms
	int32_t p=0, i=0, u=0;
	uint32_t t=0, timestamp=0, start_time=0;

	/* Where the opponent is seen or was last seen */
	uint8_t opp_right=0, opp_left=0, opp_last=FRONT;

	time_delta(&start_time);
	while (1){
		if (left_outside() || right_outside()){
			i = 0;
			t = 0;
			timestamp = 0;
			opp_last = FRONT;
			return BORDER;
		}

		/********* PID control ****************
		 *
		 * Describes changes depending on:
		 * p = the magnitude of the error
		 * i = the duration of the error
		 * (d) = the speed of worsening error
		 *
		 * u = The weighted sum of the changes
		 *
		 **************************************/

		/* Sensor */
		opp_right = obstacle_right();
		opp_left = obstacle_left();

		/* Act on suspicion when unknown */
		if (!opp_right && !opp_left){
			opp_right = (opp_last == RIGHT || opp_last == FRONT);
			opp_left = (opp_last == LEFT || opp_last == FRONT);

			/* Track how many milliseconds we have not seen anything */
			t += (timestamp != 0) ? time_delta(&timestamp) : 0&time_delta(&timestamp);
		
			/* Reset attack on timeout */
			if (t>TIMEOUT){
				timestamp = 0;
				t = 0;
				i = 0;
				return SEARCH;
			}
		} else {
			timestamp = 0;
			t = 0;
		}

		/* Adjustments */
		if (opp_right && opp_left){
			p = 0;
			i = 0;
			opp_last = FRONT;
		} else if (opp_right){
			p = 1;
			if (opp_last == RIGHT) ++i;
			opp_last = RIGHT;
		} else if (opp_left){
			p = -1;
			if (opp_last == LEFT) --i;
			opp_last = LEFT;
		}

		/* Weighted sum */
		u = KP*p+KI*i;
		if (u>200) u = 200;
		if (u<-200) u = -200;

		/* Control */
		if (u==0)
			drive_forward(100);
		if (u>0)
			turn_left(100, u+MIN);
		else if (u<0)
			turn_right(100, (-u)+MIN);

		/* Debug */
		if (opp_right && opp_left){
			turn_LED1(ON);
			turn_LED2(ON);
		} else if (opp_right){
			turn_LED1(ON);
			turn_LED2(OFF);
		} else if (opp_left){
			turn_LED1(OFF);
			turn_LED2(ON);
		} else {
			turn_LED1(OFF);
			turn_LED2(OFF);
		}

		delay(sat_substract(stage_duration, time_delta(&start_time)));
	}

	return ATTACK;
}

uint8_t border(){
	while (1){
		if (right_outside()){
			spin_left(100);
			delay(300);
		} else if (left_outside()){
			spin_right(100);
			delay(300);
		} else {
			return SEARCH;
		}
	}

	return BORDER;
}

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

inline uint32_t sat_substract(uint32_t a, uint32_t b){
	return (b>a) ? 0 : a-b;
}
