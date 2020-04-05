/*
 * map.h
 *
 *  Created on: 3 Apr 2020
 *      Author: Michael Biselx
 *
 * \brief		This module keeps track of walls encountered by the robot.
 *
 * 				While the robot is moving around, new points must be periodically logged.
 * 				Any encounter of a wall or corner triggers a reduction of the series of
 * 				points into a line by linear regression, in order to save memory space.
 * 				The resulting map may be sent to a computer at any time.
 *
 */

#ifndef MAP_H_
#define MAP_H_

#include <hal.h>

/*	\brief		function to log a point -> must be done relatively often to
 * 				keep track of the robot's position
 * 	steps_r		number of steps done by right motor since last call
 * 	steps_l		number of steps done by left motor since last call
 */
void map_log_new_point(int16_t steps_r, int16_t steps_l);

/*	\brief		function to log the encounter of a wall -> must be called at first encounter of a wall
 */
void map_log_wall_encounter(void);

/*	\brief		function to log the loss of a wall -> must be called at first loss of a wall
 */
void map_log_wall_loss(void);

/*	\brief		function to log the encounter of a corner (=end of a wall segment) -> must be called at every corner
 */
void map_log_corner(void);


/*	\brief		sends all active points to computer as ASCII text -> for debugging, your terminal will get flooded
 */
void map_display_all_points(void);

/*	\brief		sends all active points to computer in coded format to use in MATLAB
 */
void map_send_all_points_to_computer(void);

/*	\brief		sends all walls to computer in coded format to use in MATLAB
 */
void map_send_all_walls_to_computer(void);

/*	\brief		sends all active points and walls to computer in coded format to use in MATLAB
 */
void map_send_all_data_to_computer(void);


#endif /* MAP_H_ */
