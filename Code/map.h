/*
 * map.h
 *
 *  Created on: 3 Apr 2020
 *      Author: Michael Biselx
 *
 * \brief	This module keeps track of the path travelled by the robot.
 *
 * 			This thread - once started - automatically logs the robot's
 * 				movement and creates a map of the path it followed.
 * 				Furthermore, if so instructed, the thread will periodically
 * 				send a copy of the entire map created to the computer via Bluetooth.
 *
 */

#ifndef MAP_H_
#define MAP_H_

#include <hal.h>

/*	\brief	inits the map thread - must still be instructed to start
 * 				mapping / transmitting
 */
void map_init(void);

/*	\brief	starts the mapping process
 * 	\param	transmit	does the resulting map get transmitted
 */
void map_start_mapping(bool transmit);

/*				pauses the mapping process
 */
void map_pause_mapping(void);

/*	\brief	unpauses the mapping process
 * 	\param	transmit	does the resulting map get transmitted
 */
void map_unpause_mapping(bool transmit);

/*				ends the mapping process, creates & transmits final map
 */
void map_stop_mapping(void);


/*	\brief		sends all active points to computer as ASCII text
 * 				!! for debugging, your terminal will get flooded !!
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
