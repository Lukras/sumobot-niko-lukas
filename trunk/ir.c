#include "ir.h"

void initialize_IR_LEDs(){
	cli();

	TCCR0A = 0;

	/* Should have no prescaler, but default as off
	TCCR0A |= 1<<CS00; */

    DDRB |= 1<<PB4; // OC0A as output for IR LEDs
    TCCR0A |= 1<<WGM01 | 1<<COM0A0; // CTC mode and toggle OC0A on compare match

	/* Original is 104 */
	/* Good = 90, 93, 100 */
    OCR0A = 101;

	sei();
}

void stop_IR_LEDs(){
	TCCR0A &= ~(1<<CS02|1<<CS01|1<<CS00); // Timer 0 off
}

void start_IR_LEDs(){
	TCNT0 = 0;
	TCCR0A |= 1<<CS00; // no presc
}

void initialize_IR_detectors(){
    DET_DDR &= ~(1<<DET_RIGHT|1<<DET_LEFT); // IR detectors as input    
}

static uint8_t raw_obstacle_right(){
	start_IR_LEDs();
	delay(1);
	stop_IR_LEDs();

    return bit_is_clear(DET_PIN, DET_RIGHT);
}

static uint8_t raw_obstacle_left(){
	start_IR_LEDs();
	delay(1);
	stop_IR_LEDs();
	
    return bit_is_clear(DET_PIN, DET_LEFT);
}

uint8_t obstacle_right(){
	uint16_t hits=0;
	for (uint8_t i=0; i<IR_SAMPLES; ++i){
		if (raw_obstacle_right()) hits++;
		delay(IR_INTERVAL);
	}
	return hits>=IR_THRESHOLD;
}

uint8_t obstacle_left(){
	uint16_t hits=0;
	for (uint8_t i=0; i<IR_SAMPLES; ++i){
		if (raw_obstacle_left()) hits++;
		delay(IR_INTERVAL);
	}
	return hits>=IR_THRESHOLD;
}
