#include "drive.h"

/*** Private functions ***/

static inline void set_right_wspeed(int8_t speed){
	set_speed(RIGHT_WHEEL, speed);
}

static inline void set_left_wspeed(int8_t speed){
	set_speed(LEFT_WHEEL, -speed);
}

/*** Public functions ***/

void drive_forward(int8_t speed){
	set_right_wspeed(speed);
	set_left_wspeed(speed);
}

void stop(){
	drive_forward(0);
}

void drive_backward(int8_t speed){
	set_right_wspeed(-speed);
	set_left_wspeed(-speed);
}

void spin_right(int8_t speed){
	set_right_wspeed(speed);
	set_left_wspeed(-speed);
}

void spin_left(int8_t speed){
	set_right_wspeed(-speed);
	set_left_wspeed(speed);
}

void turn_right(int16_t speed, int16_t steep){
	set_right_wspeed(speed);
	set_left_wspeed(speed-steep);
}

void turn_left(int16_t speed, int16_t steep){
	set_right_wspeed(speed-steep);
	set_left_wspeed(speed);
}
