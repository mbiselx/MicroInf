/*
 * map.h
 *
 *  Created on: 3 Apr 2020
 *      Author: Michael Biselx
 */

#ifndef MAP_H_
#define MAP_H_

#include <hal.h>

void destroy_all_points(void);

void map_log_new_point(int16_t steps_r, int16_t steps_l);

void map_display_all_points(void);

void map_send_all_data_to_computer(void);


#endif /* MAP_H_ */
