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

/*  \brief		initializes the thread for mapping the robot's movements
 * 				this thread - once started - automatically logs the robot's
 * 				movement and creates a map of the path it followed.
 * 				Furthermore, if so instructed, the thread will periodically
 * 				send a copy of the entire map created to the computer via Bluetooth.
 * 				If the tread detects a closed loop in the map, the map is finalized
 * 				and sent to the computer a final time before the thread is shut down
 */
void map_init(void);

/*  \brief		starts the mapping process
 * 	\param		transmit	sets the transmission state for the thread
 */
void map_start_mapping(bool transmit);

/*	\brief		pauses the mapping process
 */
void map_pause_mapping(void);

/*	\brief		unpauses the mapping process
 * 	\param		transmit	sets the transmission state for the thread
 */
void map_unpause_mapping(bool transmit);


#endif /* MAP_H_ */
