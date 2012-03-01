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

/* Attack PID constants LE
#define KP			60
#define KI			7
#define MIN			25
#define TIMEOUT		1200
*/

/* Attack PID constants HE */
#define KP			50			// Constant reaction
#define KI			4			// Incremental reaction
#define MIN			25			// Minimum turn
#define TIMEOUT		2500		// Timeout on target lost (ms)

/* Search constants */
#define SWEEP_MS	1200

/* Border constants */
#define DET_TRESH_MS 250
#define DEG_45_MS   250
#define DEG_135_MS   650
#define DEG_90_MS   500
#define DEG_180_MS  800 

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
uint32_t limit(int32_t a, int32_t max);

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
	turn_LED1(OFF);
	turn_LED2(OFF);

	uint32_t start_time=0, stage_duration=50;

	uint32_t t=0;
	uint32_t timestamp=0;
	static uint8_t dir = RIGHT;
    uint8_t just_been_in_b_mode = 1;

	time_delta(&start_time);
	while (1){
		t += (timestamp != 0) ? time_delta(&timestamp) : 0&time_delta(&timestamp);
		if (t > SWEEP_MS){
			t = 0;
			dir = (dir==RIGHT) ? LEFT : RIGHT; 
            just_been_in_b_mode = 0;
		}
		if (dir==RIGHT){
			turn_right(100, 100);
		}else{
			turn_left(100, 100);
		}

        if (!just_been_in_b_mode){
    		if (left_outside()){
                dir = RIGHT;
                return BORDER;
            }else if (right_outside()){
                dir = LEFT;
                return BORDER;
            }
        }
			

		if (obstacle_right() || obstacle_left())
			return ATTACK;

		delay(sat_substract(stage_duration, time_delta(&start_time)));
	}

	return SEARCH;
}

uint8_t attack(){
	const uint16_t stage_duration=50; // ms
	int32_t p=0, i=0, u=0;
	uint32_t timeout=0, start_time=0;

	/* Where the opponent is seen or was last seen */
	uint8_t opp_right=0, opp_left=0, opp_last=FRONT;

	time_delta(&start_time);
	while (1){
		if (left_outside() || right_outside())
			return BORDER;

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

		/* Read sensors */
		opp_right = obstacle_right();
		opp_left = obstacle_left();

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

		/* Act on suspicion if target lost */
		if (!opp_right && !opp_left){
			opp_right = (opp_last == RIGHT || opp_last == FRONT);
			opp_left = (opp_last == LEFT || opp_last == FRONT);
		
			/* Reset attack on timeout unless we are charging */
			if (timeout == 0) timeout = time_since(0);
			if (time_since(timeout)>TIMEOUT && opp_last != FRONT)
				return SEARCH;
		} else {
			timeout = 0;
		}

		/* Calculate the turn components */
		if (opp_right && opp_left){
			p = 0;
			i = 0; // allows for sharp reactions, no movement learning
			opp_last = FRONT;
		} else if (opp_right){
			p = 1;
			if (timeout==0) ++i;
			opp_last = RIGHT;
		} else if (opp_left){
			p = -1;
			if (timeout==0) --i;
			opp_last = LEFT;
		}

		/* Weighted sum of the turn components */
		u = limit(KP*p+KI*i, 200);

		/* Actually control the turn */
		if (u==0)
			drive_forward(100);
		if (u>0)
			turn_left(100, u+MIN);
		else if (u<0)
			turn_right(100, (-u)+MIN);

		delay(sat_substract(stage_duration, time_delta(&start_time)));
	}

	return ATTACK;
}

uint8_t border(){
	turn_LED1(OFF);
	turn_LED2(OFF);

    uint16_t spin_delay;
    uint32_t t_start = time_since(0);
    uint32_t t_elapsed;
    drive_forward(100);
    if (right_outside()){
        while(((t_elapsed = time_since(t_start)) < DET_TRESH_MS) && !left_outside());
        /* bot was rather parallel to border so turn 90° */ 
        if (t_elapsed >= DET_TRESH_MS){
		    spin_delay = 250;                     
        /* other qti detected border before timeout
         * so turn according to how long it took the other qti
         * to find the border
         */
        }else{
            //turn between 90° and 180°
            spin_delay = 550;
        }
        spin_left(100);
        delay(spin_delay);
    }else if (left_outside()){
        while(((t_elapsed = time_since(t_start)) < DET_TRESH_MS) && !right_outside());
        /* bot was rather parallel to border so turn 90° */ 
        if (t_elapsed >= DET_TRESH_MS){
            spin_delay = 250;                     
        /* other qti detected border before timeout
         * so turn according to how long it took the other qti
         * to find the border
         */
        }else{
            //turn between 90° and 180°
            spin_delay = 550;
        }
        spin_right(100);
		delay(spin_delay);
    }else{
        //this must not happen!
    }
    return SEARCH;
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

uint32_t limit(int32_t a, int32_t max){
	if (a>max) return max;
	if (a<-max) return -max;
	return a;
}
