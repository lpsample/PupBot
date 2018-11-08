// Created Nick Livingston 2018
#include "sensors.h"
//*************************************************** Function Declarations ***********************************************************//
//Behavior Primitive Functions
void vision();
int check_escape_conditions();
int check_escape_back_conditions();
int check_seek_light_conditions();
int check_seek_dark_conditions();
int check_avoid_conditions();
int check_approach_conditions();
void escape();
void escape_back();
void seek_light();
void seek_dark();
void avoid();
void approach();
void straight_cruise();
void arc_cruise();
void stop();
int min(int first, int second);
int max(int first, int second);
//Motor Control Function
void drive(float left, float right, float delay_seconds);

//*************************************************** Global Constant Declarations ****************************************************//
const int FRONT_BUMP = 0;
const int BACK_BUMP = 1;//15;
const int LEFT_PHOTO = 2;
const int RIGHT_PHOTO = 3;
const int LEFT_IR = 4;
const int RIGHT_IR = 5;
const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 3;

const int YELLOW_CH = 0;
const int RED_CH = 1;
const int BLUE_CH = 2;
const int GREEN_CH = 3;

const float SERVO_MAX = 2047.0;
const float TOP_SPEED = 1023.0;//100;
//Boolean Constants
//This dialect of C does not have boolean data types, so we're faking it
//according to convention: true is non-zero, usually 1, and false is 0.
const int TRUE = 1;
const int FALSE = 0;

//*************************************************** Function Definitions ****************************************************//
int main() 
{
	int i;
	enable_servos();
		
	while(1){
		if(check_escape_conditions()){
escape();
}
else if(check_avoid_conditions()){
avoid();
}
else if(check_seek_light_conditions()){
seek_light();
}
else{straight_cruise();}

	}
	
	return 0;
}
/******************************************************/
void vision(){
	int num_red;
	int num_yellow;
	int num_green;
	int num_blue;
	
	int area_red;
	int area_yellow;
	int area_green;
	int area_blue;
	
	int width = 160;
	int height = 120;
	int screen_area = width * height;
	
	camera_update();
	num_red = get_object_count(RED_CH);
	num_yellow = get_object_count(YELLOW_CH);
	num_green = get_object_count(GREEN_CH);
	num_blue = get_object_count(BLUE_CH);
	
	//printf("red = %d, yellow = %d, green = %d, blue = %d\n", num_red, num_yellow, num_green, num_blue);
	
	if(num_red > 0){
		area_red = get_object_area(RED_CH, 0);
		printf("red_ratio = %f\n", (float)area_red/screen_area * 100.0);
	}
	if(num_yellow > 0){
		area_yellow = get_object_area(YELLOW_CH, 0);
		printf("yellow_ratio = %f\n", (float)area_yellow/screen_area * 100.0);
	}
	if(num_green > 0){
		area_green = get_object_area(GREEN_CH, 0);
		printf("green_ratio = %f\n", (float)area_green/screen_area * 100.0);
	}
	if(num_blue > 0){
		area_blue = get_object_area(BLUE_CH, 0);
		printf("blue_ratio = %f\n", (float)area_blue/screen_area * 100.0);
	}
	//printf("AREA... red = %d, yellow = %d, green = %d, blue = %d\n", area_red, area_yellow, area_green, area_blue);	
}
/******************************************************/
float check_frontbump_conditions(){
	float left_angle 			 = 0.0;
	float right_angle 		 = 0.0;
	float total_angle         = 0.0;
	int bump_midpoint    = 200;
	int bump_max	        = 400;
	int front_bump_value = analog10(FRONT_BUMP);
	printf("front_bump_value = %d\n", front_bump_value);
	if(front_bump_value < bump_midpoint){
		left_angle  = ((float)front_bump_value/(float)bump_midpoint) * 90.0;
		right_angle = 0.0;
	}else if(front_bump_value > bump_midpoint && front_bump_value <= bump_max){
		left_angle  = 0.0;
		right_angle = 90.0 - ((float)(bump_max - front_bump_value)/(float)bump_midpoint) * 90.0;
	}
	
	total_angle = left_angle - right_angle;
	//printf("left_angle = %f\n", left_angle);
	//printf("right_angle = %f\n", right_angle);
	//printf("total_angle = %f\n", total_angle);
	return total_angle;
}
/******************************************************/
int check_escape_conditions(){
	int bump_threshold = 0;
	int bump_max = 400;
	int front_bump_value = analog10(FRONT_BUMP);
	
	if(front_bump_value >= bump_threshold && front_bump_value <= bump_max){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int check_escape_back_conditions(){
	int bump_threshold = 250;
	int bump_max = 400;
	
	int back_bump_value = analog10(BACK_BUMP);
	
	if(back_bump_value < bump_threshold){
		return TRUE;
	}
	else if(back_bump_value >= bump_threshold && back_bump_value <= bump_max)
	{
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int check_seek_light_conditions(){
	int left_photo_raw;
	int right_photo_raw;
	float right_coeff;
	float left_coeff;
	left_photo_raw    = (analog_et(LEFT_PHOTO) - photo_offset);
	right_photo_raw   = (analog_et(RIGHT_PHOTO));
	//printf("left_photo_raw = %d\nright_photo_raw = %d\n", left_photo_raw, right_photo_raw);
	
	left_coeff         = (photo_coeff * (photo_max - (float)left_photo_raw)/photo_max);
	right_coeff       = (photo_coeff * (photo_max - (float)right_photo_raw)/photo_max);
	
	//printf("left_coeff = %f\nright_coeff = %f\n", left_coeff, right_coeff);
	
	left_photo_value  = left_photo_raw * left_coeff;
	right_photo_value = right_photo_raw * right_coeff;
	
	if(left_photo_value > photo_max){
		left_photo_value = photo_max;
	}
	if(right_photo_value > photo_max){
		right_photo_value = photo_max;
	}
	
	photo_difference  = left_photo_value - right_photo_value;

	//printf("left_photo = %f\nright_photo = %f\n", left_photo_value, right_photo_value);
	//printf("photo_difference = %f\n", photo_difference);
	if(abs(photo_difference) > photo_threshold){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

/******************************************************/
int check_seek_dark_conditions(){
	int left_photo_raw;
	int right_photo_raw;
	float right_coeff;
	float left_coeff;
	left_photo_raw    = (analog_et(LEFT_PHOTO) - photo_offset);
	right_photo_raw   = (analog_et(RIGHT_PHOTO));
	//printf("left_photo_raw = %d\nright_photo_raw = %d\n", left_photo_raw, right_photo_raw);
	
	left_coeff         = (photo_coeff * (photo_max - (float)left_photo_raw)/photo_max);
	right_coeff       = (photo_coeff * (photo_max - (float)right_photo_raw)/photo_max);
	
	//printf("left_coeff = %f\nright_coeff = %f\n", left_coeff, right_coeff);
	
	left_photo_value  = left_photo_raw * left_coeff;
	right_photo_value = right_photo_raw * right_coeff;
	
	if(left_photo_value > photo_max){
		left_photo_value = photo_max;
	}
	if(right_photo_value > photo_max){
		right_photo_value = photo_max;
	}
	
	photo_difference  = left_photo_value - right_photo_value;

	//printf("left_photo = %f\nright_photo = %f\n", left_photo_value, right_photo_value);
	//printf("photo_difference = %f\n", photo_difference);
	if(abs(photo_difference) > photo_threshold){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

/******************************************************/
int check_avoid_conditions(){
	int left_ir_value;
	int right_ir_value;
	
	left_ir_value = analog_et(LEFT_IR);
	right_ir_value = analog_et(RIGHT_IR);
	
	if(left_ir_value > avoid_threshold || right_ir_value > avoid_threshold){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
int check_approach_conditions(){
	int left_ir_value;
	int right_ir_value;
	int approach_threshold = 300;
	
	left_ir_value = analog_et(LEFT_IR);
	right_ir_value = analog_et(RIGHT_IR);
	
	if(left_ir_value > approach_threshold || right_ir_value > approach_threshold){
		return TRUE;
	}else{
		return FALSE;
	}
}

/******************************************************/
void escape(){
	int bump_threshold = 250;
	int bump_max = 400;
	int front_bump_value = analog10(FRONT_BUMP);
	//printf("EscapeFrontBump()\n");
	if(front_bump_value < bump_threshold){
		//drive(-0.25, 0.75, 0.50);
		drive(-0.0, -0.90, 0.50);
		//printf("drive(-0.25, -0.75, 1.5);\n");
	}
	else if(front_bump_value >= bump_threshold && front_bump_value <= bump_max){
		//drive(0.75, -0.25, 0.50);
		drive(-0.90, -0.0, 0.50);
		//printf("drive(-0.75, -0.25, 1.5);\n");
	}

}

/******************************************************/
void escape_back(){
	int bump_threshold = 250;
	int bump_max = 400;
	
	int back_bump_value = analog10(BACK_BUMP);
	
	if(back_bump_value < bump_threshold){
		//drive(0.25,-0.75, 0.30);
		drive(0.9, 0.0, 0.50);
	}
	else if(back_bump_value >= bump_threshold && back_bump_value <= bump_max)
	{
		//drive(-0.75, 0.25, 0.30);
		drive(0.0, 0.90, 0.50);
	}
}

/******************************************************/
void seek_light(){
	float left_servo;
	float right_servo;
	
	if(fabs(photo_difference) > photo_threshold){
		right_servo = photo_difference/photo_max;//(float)(photo_max - right_photo_value)/(float)photo_max;
		left_servo  = -right_servo;//(float)(photo_max - left_photo_value)/(float)photo_max;
	}

	//printf("left_servo = %f, right_servo = %f\n", left_servo, right_servo);
	drive(left_servo, right_servo, 0.25);
}

/******************************************************/
void seek_dark(){
	float left_servo;
	float right_servo;
	
	if(fabs(photo_difference) > photo_threshold){
		left_servo  = photo_difference/photo_max;
		right_servo = -left_servo;
	}

	//printf("left_servo = %f, right_servo = %f\n", left_servo, right_servo);
	drive(left_servo, right_servo, 0.25);
	
}

/******************************************************/
void avoid(){
	int left_ir_value;
	int right_ir_value;
	
	left_ir_value = analog_et(LEFT_IR);
	right_ir_value = analog_et(RIGHT_IR);
	
	if(left_ir_value > avoid_threshold){
		drive(0.5, -0.5, 0.1);
	}
	
	else if(right_ir_value > avoid_threshold){
		drive(-0.5, 0.5, 0.1);
	}
}

/******************************************************/
void approach(){
	int left_ir_value;
	int right_ir_value;
	int approach_threshold = 300;
	
	left_ir_value = analog_et(LEFT_IR);
	right_ir_value = analog_et(RIGHT_IR);
	
	//printf("left_ir = %d, right_ir = %d\n", left_ir_value, right_ir_value);
	
	if(left_ir_value > approach_threshold){
		drive(0.1, 0.9, 0.5);
	}
	
	else if(right_ir_value > approach_threshold){
		drive(0.9, 0.1, 0.5);
	}
}
/******************************************************/
void straight_cruise(){
	//drive(1.0, 1.0, 0.5);
	drive(0.40, 0.40, 0.5);
}
/******************************************************/
void arc_cruise(){
	drive(0.1, 0.75, 0.5);
}
/******************************************************/
void stop(){
	drive(0.0, 0.0, 0.25);
}
/******************************************************/
void drive(float left, float right, float delay_seconds){
	float left_speed;
	float right_speed;
	
	delay_seconds *= 1000;
	//printf("left = %f, right = %f, TOP_SPEED = %f\n", left, right, TOP_SPEED);
	
	right_speed = TOP_SPEED - (right * TOP_SPEED);
	left_speed  = TOP_SPEED + (left * TOP_SPEED);
	
	//printf("left_speed = %f, right_speed = %f\n", left_speed, right_speed);	
	set_servo_position(LEFT_MOTOR, left_speed);
	set_servo_position(RIGHT_MOTOR, right_speed);
	msleep(delay_seconds);
	//disable_servos();
}
/******************************************************/
int min(int first, int second)
{
	int minValue = 0;
	if(first < second){
		minValue = first;
	}else{
		minValue = second;
	}
	return minValue;
}
/******************************************************/
int max(int first, int second)
{
	int maxValue = 0;
	
	if(first > second){
		maxValue = first;
	}else{
		maxValue = second;
	}
	
	return maxValue;
}
