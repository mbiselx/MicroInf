/*
 * move.c
 *
 *  Created on: 02.04.2020
 *      Author: samuel
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <sensors\proximity.h>

#include <motors.h>
#include <pi_regulator.h>

#define PI							3.1415f

#define WHEEL_PERIMETER_CM 			13 																//cm
#define WHEEL_PERIMITER_STEPS 		1000															//steps
#define GOAL_DIST_CM 				3 																//cm
#define GOAL_SENSOR 				230
#define WHEEL_DISTANCE_CM      		5.28f    														//cm
#define WHEEL_DISTANCE_STEPS		(WHEEL_DISTANCE_CM*WHEEL_PERIMITER_STEPS/WHEEL_PERIMETER_CM) 	//steps

//max speed=1100steps/s
#define BASIC_SPEED					500																//steps/s
#define SPEED_FACTOR				0.1

enum IR_SENSORS{IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};
enum WALL_SIDE{WALL_ON_LEFT, WALL_ON_RIGHT};


//maximal area that robot can map: circle of diameter ~8.5m (with only straight walls)
static int32_t current_pos_x=0; //steps
static int32_t current_pos_y=0; //steps
static float current_angle=0; //rad



void move_robot_motors_speed(int16_t speed_left, int16_t speed_right){

	/*static int32_t steps_left=0;
	static int32_t steps_right=0;
	steps_left = left_motor_get_pos() - steps_left;
	steps_right = right_motor_get_pos() - steps_right;
	*/

	right_motor_set_speed(speed_right);
	left_motor_set_speed(speed_left);
}

void move_robot_along_wall(int8_t wall_side){

	/*
	Wall is on the left side of the robot -> left IR sensor is detecting
	if robot has to come closer to wall -> right wheel has to turn faster (than left wheel)
	if robot has to get away from wall --> left wheel has to turn faster (than right wheel)
	       ^
	       |
Wall	 Front
  |      o   o
  |   o         o
  |  o   ROBOT   o
  |  o           o
  |   o         o
  |      o   o

	 */

	/*
	Wall is on the right side of the robot -> right IR sensor is detecting
	if robot has to come closer to wall -> right wheel has to turn slower (than left wheel)
	if robot has to get away from wall --> left wheel has to turn slower (than right wheel)
	        ^
		    |
	      Front    Wall
          o   o      |
	   o         o   |
      o   ROBOT   o  |
	  o           o  |
	   o         o   |
	      o   o      |

   */

	int speed_to_wall=0;

	//if wall on the right side of the robot -> IR_2 sensor is detecting
	if(wall_side==WALL_ON_RIGHT)
		speed_to_wall=-pi_regulator_regulator(get_prox(IR_2), GOAL_SENSOR);

	//if wall on the left side of the robot -> IR_7 sensor is detecting
	if(wall_side==WALL_ON_LEFT)
		speed_to_wall=pi_regulator_regulator(get_prox(IR_7), GOAL_SENSOR);


	//control if speed_to_wall is in higher or lower than MOTOR_SPEED_LIMIT
	if(speed_to_wall>MOTOR_SPEED_LIMIT)
		speed_to_wall=MOTOR_SPEED_LIMIT;
	if(speed_to_wall<-MOTOR_SPEED_LIMIT)
		speed_to_wall=-MOTOR_SPEED_LIMIT;


	move_robot_motors_speed(BASIC_SPEED, (int)(BASIC_SPEED+speed_to_wall*SPEED_FACTOR));

	if(speed_to_wall == 0)
		move_robot_motors_speed(BASIC_SPEED, BASIC_SPEED);
}

//main for testing PI regulator and move
void move_main(void){

	while(1){
		move_robot_along_wall(WALL_ON_RIGHT);
	}
}
