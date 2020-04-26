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

void map_init(void);

void map_start_mapping(bool transmit);

void map_pause_mapping(void);

void map_unpause_mapping(bool transmit);


#endif /* MAP_H_ */
