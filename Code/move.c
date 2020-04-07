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

#include "move.h"
#include "pi_regulator.h"

#include "map.h"

#define PI								3.1415f

#define WHEEL_PERIMETER_CM 				13 																//cm
#define WHEEL_PERIMITER_STEPS 			1000															//steps
#define GOAL_SENSOR 					270
#define WHEEL_DISTANCE_CM      			5.28f    														//cm
#define WHEEL_DISTANCE_STEPS			(WHEEL_DISTANCE_CM*WHEEL_PERIMITER_STEPS/WHEEL_PERIMETER_CM) 	//steps

//max speed=1100steps/s
#define BASIC_SPEED						400																//steps/s
#define SPEED_FACTOR					0.1
#define SPEED_ACUTE_ANGLE_INNER_WHEEL	(0.2*BASIC_SPEED) 												//value found experimentally
#define SPEED_ACUTE_ANGLE_OUTER_WHEEL	(1.7*BASIC_SPEED)												//value found experimentally
#define SPEED_REFLEX_ANGLE_INNER_WHEEL	(0.9*BASIC_SPEED)												//value found experimentally
#define SPEED_REFLEX_ANGLE_OUTER_WHEEL	(1.6*BASIC_SPEED)												//value found experimentally

#define SENSOR_WALL_CLOSE				270																//if sensor closer to wall value sensor higher
#define SENSOR_WALL_TO_CLOSE			700


enum IR_SENSORS{IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};
enum WALL_SIDE{WALL_ON_LEFT, WALL_ON_RIGHT};


int move_is_wall_close(int sensor){
	if(get_prox(sensor)>=SENSOR_WALL_CLOSE)
		return true;
	else
		return false;
}

int move_is_wall_to_close(void){//looks if Sensor IR1, IR2, IR7 or IR8 is to close to a wall
	for(int sensor=IR_1; sensor<=IR_8; sensor++){
		if(sensor == IR_3)
			sensor=IR_7;
		if(get_prox(sensor)>=SENSOR_WALL_TO_CLOSE)
			return true;
	}
	return false;
}

int move_is_sensor_max(int sensor){//returns true if sensor is further away to wall than in previous function call
	static int sensor_before=0;
	if(get_prox(sensor)<=sensor_before){
		sensor_before=0;
		return true;
	}
	sensor_before=get_prox(sensor);
	return false;
}

void move_robot_motors_speed_slow_increment(int16_t speed_left, int16_t speed_right){
	static int speed_left_before=0;
	static int speed_right_before=0;
	if(speed_left_before==speed_left && speed_right_before==speed_right)
		return;
	for(int count=8; count>=1; count--){//slow increment of speed
		right_motor_set_speed(speed_right/count);
		left_motor_set_speed(speed_left/count);
		chThdSleepMilliseconds(10);
	}
	speed_left_before=speed_left;
	speed_right_before=speed_right;
	/*right_motor_set_speed(speed_right);
	left_motor_set_speed(speed_left);
	chThdSleepMilliseconds(10);//time to take care of map stuff*/
}

void move_robot_motors_speed(int16_t speed_left, int16_t speed_right){
	right_motor_set_speed(speed_right);
	left_motor_set_speed(speed_left);
	chThdSleepMilliseconds(10);//time to take care of map stuff
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

void move_handler(void){
	/*
	 IR sensors on robot
	 	   FRONT
        , - ~ ~ - ,
    , ' \       /   ' ,
  ,     IR8    IR1      ,
 , \                  /  ,
,  IR7              IR2   ,
,                         ,
,-IR6                 IR3-,
,                         ,
 ,                       ,
  ,  IR5         IR4    ,
    , /           \  , '
      ' - , _ _ ,  '

     */


	while(true)	{
		move_robot_motors_speed(BASIC_SPEED, BASIC_SPEED);//drive forward until wall is reached
		if(move_is_wall_close(IR_6) || move_is_wall_close(IR_7) || move_is_wall_close(IR_8)){
			while(true){//left wall mode

				if(move_is_wall_to_close()){//robot in front of acute or obtuse angle corner (0-180°)
					move_robot_motors_speed(0, 0); //stops motors
					chThdSleepMilliseconds(100);
					while(true){
						move_robot_motors_speed_slow_increment(BASIC_SPEED,-BASIC_SPEED);
						if(!move_is_wall_to_close() && move_is_sensor_max(IR_6)){
							move_robot_motors_speed_slow_increment(0, 0);
							break;
						}
					}
				}

				while(!move_is_wall_close(IR_7)){//wall is far from IR_7 -> robot in front of reflex angle corner (180-360°)
					move_robot_motors_speed(SPEED_REFLEX_ANGLE_INNER_WHEEL, SPEED_REFLEX_ANGLE_OUTER_WHEEL);
				}
				move_robot_along_wall(WALL_ON_LEFT);
			}
		}

		if(move_is_wall_close(IR_3) || move_is_wall_close(IR_2) || move_is_wall_close(IR_1)){
			while(true){//right wall mode
				while(move_is_wall_close(IR_3) && move_is_wall_close(IR_2) && move_is_wall_close(IR_1)){//robot in front of acute or obtuse angle corner (0-180°)
					move_robot_motors_speed(SPEED_ACUTE_ANGLE_INNER_WHEEL, SPEED_ACUTE_ANGLE_OUTER_WHEEL);
				}
				while(!move_is_wall_close(IR_2)){//wall is far from IR_7 -> robot in front of reflex angle corner (180-360°)
					move_robot_motors_speed(SPEED_REFLEX_ANGLE_OUTER_WHEEL, SPEED_REFLEX_ANGLE_INNER_WHEEL);
				}
				move_robot_along_wall(WALL_ON_RIGHT);
			}
		}
	}

}


