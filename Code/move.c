//----global chibiOS includes----
#include "ch.h"
#include "hal.h"
#include <chprintf.h>

//----specific epuck2 includes----
#include <sensors\proximity.h>
#include <motors.h>
#include <leds.h>

//----specific personal includes----
#include "move.h"
#include "pi_regulator.h"
#include "map.h"



//Speed defines
//max speed=1100steps/s
#define BASIC_SPEED								450																//steps/s
#define SPEED_FACTOR							0.1																//value found experimentally
#define SPEED_ACUTE_ANGLE_INNER_WHEEL			(0.2*BASIC_SPEED)												//steps/s (value found experimentally)

//Sensor defines (if sensor closer to wall value sensor higher)
#define SENSOR_WALL_CLOSE						270																//~3cm
#define SENSOR_WALL_TO_CLOSE					600																//~2-2.5cm
#define GOAL_SENSOR 							270																//~3cm

//other defines
#define NUMBER_OF_INCREMENT						8

//static variables
static int8_t drive_mode=false; //if drive_mode==true -> robot drives; if drive_mode==false -> robot does not drive
								//drive_mode is initially false until robot has been placed

//enums
enum WALL_SIDE{WALL_ON_LEFT, WALL_ON_RIGHT};
enum IR_SENSORS{IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};



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



/**
* move_update_drive_mode
*
* @brief   					updates the state of driv_mode (changes state from true to false or from false to true)
*
* @param r					-
*
* @return					-
*/
void move_update_drive_mode(void){

	drive_mode = !drive_mode;
	chprintf((BaseSequentialStream*)&SD3, "drive_mode = %d\n\r", drive_mode);
	palTogglePad(GPIOD, GPIOD_LED5);
	chThdSleepSeconds(5);//wait for 5 sec
}



/**
* move_get_drive_mode
*
* @brief   					returns state of drieve_mode
*
* @param r					-
*
* @return					state of drive_mode (true or false)
*/
uint8_t move_get_drive_mode(void){
	return drive_mode;
}



/**
* move_is_wall_close
*
* @brief   					Returns if a wall is close or not to given sensor
*
* @param 					sensor_number: 0-7 (IR_1, IR_2, ... , IR_8)
*
* @return					true: if given sensor close to wall (value of sensor is higher or equal tan SENSOR_WALL_CLOSE)
* 							false: if given sensor far from wall (value of sensor is smaller tan SENSOR_WALL_CLOSE)
*/
int move_is_wall_close(int sensor){
	if(get_prox(sensor)>=SENSOR_WALL_CLOSE)
		return true;
	else
		return false;
}



/**
* move_is_wall_to_close
*
* @brief   					Returns if one of sensor: IR1, IR2, IR7 or IR8 is to close to a wall or not
*
* @param sensor_number		-
*
* @return					true: if one ore more of the sensors to close to wall (value of sensor is higher or equal tan SENSOR_WALL_TO_CLOSE)
* 							false: if non of the sensors to close to wall (value of sensor is smaller tan SENSOR_WALL_TO_CLOSE)
*/
int move_is_wall_to_close(void){//looks if Sensor IR1, IR2, IR7 or IR8 is to close to a wall
	for(int sensor=IR_1; sensor<=IR_8; sensor++){
		if(sensor == IR_3)
			sensor=IR_7;
		if(get_prox(sensor)>=SENSOR_WALL_TO_CLOSE)
			return true;
	}
	return false;
}




/**
* move_robot_motors_speed
*
* @brief   					sets speed of two motors
*
* @param sensor_number		speed_left: speed of left motor
* 							speed_right: speed of right motor
*
* @return					-
*/
void move_robot_motors_speed(int16_t speed_left, int16_t speed_right){
	right_motor_set_speed(speed_right);
	left_motor_set_speed(speed_left);
	chThdSleepMilliseconds(10);//time for other threads
}




/**
* move_robot_motors_speed_increment
*
* @brief   					increments speed of two motors in NUMBER_OF_INCREMENT steps
*
* @param sensor_number		speed_left: final speed of left motor
* 							speed_right: final speed of right motor
*
* @return					-
*/
void move_robot_motors_speed_increment(int16_t speed_left, int16_t speed_right){
	static int speed_left_before=0;
	static int speed_right_before=0;
	if(speed_left_before==speed_left && speed_right_before==speed_right)
		return;
	for(int count=NUMBER_OF_INCREMENT; count>=1; count--){//slow increment of speed
		move_robot_motors_speed(speed_left/count, speed_right/count);
	}
	speed_left_before=speed_left;
	speed_right_before=speed_right;
}



/**
* move_robot_along_wall
*
* @brief   					robot follows wall (right or left) with help of PI controller
*
* @param sensor_number		wall_side: 	WALL_ON_LEFT: wall is on left side of robot
* 										WALL_ON_RIGHT: wall is on right side of robot
*
* @return					-
*/
void move_robot_along_wall(int8_t wall_side){

	/*
	Wall is on the left side of the robot -> IR_7 is detecting
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
	Wall is on the right side of the robot -> IR_2 is detecting
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


	//control if speed_to_wall is in higher or lower than +/-MOTOR_SPEED_LIMIT
	if(speed_to_wall>MOTOR_SPEED_LIMIT)
		speed_to_wall=MOTOR_SPEED_LIMIT;
	if(speed_to_wall<-MOTOR_SPEED_LIMIT)
		speed_to_wall=-MOTOR_SPEED_LIMIT;


	move_robot_motors_speed(BASIC_SPEED, (int)(BASIC_SPEED+speed_to_wall*SPEED_FACTOR));//right wheel turns faster or slower to correct robots distance to wall

	if(speed_to_wall == 0)// if robot has correct distance to wall drive straight
		move_robot_motors_speed(BASIC_SPEED, BASIC_SPEED);
}



/**
* move_handler
*
* @brief   					function handles movement of robot
*
* @param sensor_number		-
*
* @return					-
*/
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

	while(true){
		while(!drive_mode){//do not drive until drive_mode is true
			move_robot_motors_speed(0, 0);//stop motors
			chThdSleepSeconds(5);//wait for 5s
		}
		while(!move_is_wall_to_close()){
			move_robot_motors_speed(BASIC_SPEED, BASIC_SPEED);//drive forward until wall is reached
		}
		while(move_is_wall_close(IR_7)){
			move_robot_motors_speed_increment(BASIC_SPEED,-BASIC_SPEED);//turn on place until well aligned with wall
		}



		//"left wall mode"
		if(move_is_wall_close(IR_6) || move_is_wall_close(IR_7) || move_is_wall_close(IR_8)){

			while(true){//robot stays in left wall mode until reset
				if(move_is_wall_to_close()){//robot in front of acute or obtuse angle corner (0-180°)
					move_robot_motors_speed(0, 0);//stops motors -> not too rapid change of speed of wheels
					chThdSleepMilliseconds(100);//time for other threads
					while(true){
						move_robot_motors_speed_increment(BASIC_SPEED,-BASIC_SPEED);//turn on place
						if(!move_is_wall_to_close() && !move_is_wall_close(IR_7)){//Robot aligned with wall after corner
							move_robot_motors_speed_increment(0, 0);//stop motors -> not too rapid change of speed of wheels
							break;
						}
					}
				}

				if(!move_is_wall_close(IR_6) && !move_is_wall_close(IR_7)){//robot in front of reflex angle corner (180-360°)
					move_robot_motors_speed(0, 0);//stops motors -> not too rapid change of speed of wheels
					chThdSleepMilliseconds(100);//wait for other threads
					while(true){
						move_robot_motors_speed_increment(SPEED_ACUTE_ANGLE_INNER_WHEEL, BASIC_SPEED);//turn curve
						if(move_is_wall_close(IR_7)){//Robot (more or less) aligned with wall after corner
							move_robot_motors_speed_increment(0, 0);//stop motors -> not too rapid change of speed of wheels
							break;
						}
					}
				}

				move_robot_along_wall(WALL_ON_LEFT);//if no corner in front of robot -> robot follows wall
			}
		}





		//"right wall mode"
		if(move_is_wall_close(IR_3) || move_is_wall_close(IR_2) || move_is_wall_close(IR_1)){

			while(true){//robot stays in right wall mode until reset
				if(move_is_wall_to_close()){//robot in front of acute or obtuse angle corner (0-180°)
					move_robot_motors_speed(0, 0);//stops motors -> not too rapid change of speed of wheels
					chThdSleepMilliseconds(100);//wait for other threads
					while(true){
						move_robot_motors_speed_increment(-BASIC_SPEED,BASIC_SPEED);//turn on place
						if(!move_is_wall_to_close() && !move_is_wall_close(IR_2)){//Robot aligned with wall after corner
							move_robot_motors_speed_increment(0, 0);//stop motors -> not too rapid change of speed of wheels
							break;
						}
					}
				}

				if(!move_is_wall_close(IR_2) && !move_is_wall_close(IR_3)){//robot in front of reflex angle corner (180-360°)
					move_robot_motors_speed(0, 0);//stops motors -> not too rapid change of speed of wheels
					chThdSleepMilliseconds(100);//wait for other threads
					while(true){
						move_robot_motors_speed_increment(BASIC_SPEED, SPEED_ACUTE_ANGLE_INNER_WHEEL);//turn curve
						if(move_is_wall_close(IR_2)){//Robot (more or less) aligned with wall after corner
							move_robot_motors_speed_increment(0, 0);//stop motors -> not too rapid change of speed of wheels
							break;
						}
					}
				}

				move_robot_along_wall(WALL_ON_RIGHT);//if no corner in front of robot -> robot follows wall
			}
		}
	}

}


