#include "servos.h"

const uint8_t _num_servos = NUM_SERVOS;

/*** Private functions ***/

static void set_duty_cycle(uint8_t servo, uint16_t duty){
	if (servo>=_num_servos)
		return;

	if (duty>MAX_PULSE_OCR)
		duty = MAX_PULSE_OCR;
	if (duty<MIN_PULSE_OCR)
		duty = MIN_PULSE_OCR;

	switch (servo){

		/* Timer 1 */
		case 0:
			OCR1AH = duty>>8;
			OCR1AL = duty&(0xFF);
			break;
		case 1:
			OCR1BH = duty>>8;
			OCR1BL = duty&(0xFF);
			break;
	}

	return;
}

/*** Public functions ***/

void set_speed(uint8_t servo, int8_t speed){
	if (speed>100)
		speed = 100;
	if (speed<-100)
		speed = -100;

	set_duty_cycle(servo, OCR_MIDDLE+(((int32_t)speed*OCR_RANGE)/200));
}

void initialize_servos(){

	/* Timer 1 */

	TCCR1A = 0;
	TCCR1B = 0;

	DDRB |= (1<<PB5)|(1<<PB6);

	/* Cycle lasts 20 ms with the chosen pre-scaler (50 Hz) */
	ICR1H = ICR_VALUE>>8;
	ICR1L = ICR_VALUE&(0xFF);
	
	/* Set compare registers */
	set_speed(0, 0);
	set_speed(1, 0);

	/* A register */
	TCCR1A |= (1<<COM1A1);						/* PFCM, set up, clear up */
	TCCR1A |= (1<<COM1B1);						/* PFCM, set up, clear up */

	/* B register */
	TCCR1B |= (1<<WGM13);						/* PFCM with TOP = ICR1 */
	TCCR1B |= ((0<<CS12)|(1<<CS11)|(0<<CS10));	/* Pre-scaler is 8 */
}
