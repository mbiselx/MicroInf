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

void map_log_wall_encounter(void);

void map_log_wall_loss(void);

void map_log_corner(void);




void map_display_all_points(void);

void map_send_all_points_to_computer(void);

void map_send_all_walls_to_computer(void);

void map_send_all_data_to_computer(void);


#endif /* MAP_H_ */
