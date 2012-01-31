#include "qti.h"

static uint32_t _black=DEFAULT_BLACK;

void initialize_QTI(){
	QTI_DDR |= (1<<LR);
	QTI_PORT &= ~(1<<LR);

	QTI_DDR |= (1<<RR);
	QTI_PORT &= ~(1<<RR);
}

void calibrate_QTI(){
	_black = MAX_BLACK;
	_black = (right_raw()+left_raw())/10;
}

/*** Left side ***/

static inline void discharge_left(){
	/* R HIGH output */
	/* 1 ms pause */
	QTI_DDR |= (1<<LR);
	QTI_PORT |= (1<<LR);
	delay(5);
}

static inline void start_charge_left(){
	/* R LOW input */
	QTI_PORT &= ~(1<<LR);
	QTI_DDR &= ~(1<<LR);
}

static inline uint8_t read_left(){
	/* R input value */
	return bit_is_set(QTI_PIN, LR);
}

uint32_t left_raw(){
	uint32_t t=0;

	discharge_left();
	
	t = 0;
	cli();
	start_charge_left();
	while (read_left() && t<_black) ++t;
	sei();
	
	return t;
}

uint8_t left_outside(){
	return left_raw()<_black;
}

/*** Right side ***/

static inline void discharge_right(){
	/* R HIGH output */
	/* 1 ms pause */
	QTI_DDR |= (1<<RR);
	QTI_PORT |= (1<<RR);
	delay(5);
}

static inline void start_charge_right(){
	/* R LOW input */
	QTI_PORT &= ~(1<<RR);
	QTI_DDR &= ~(1<<RR);
}

static inline uint8_t read_right(){
	/* R input value */
	return bit_is_set(QTI_PIN, RR);
}

uint32_t right_raw(){
	uint32_t t=0;

	discharge_right();
	
	t = 0;
	cli();
	start_charge_right();
	while (read_right() && t<_black) ++t;
	sei();
	
	return t;
}

uint8_t right_outside(){
	return right_raw()<_black;
}
