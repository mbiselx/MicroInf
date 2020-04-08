/*
 * pi_regulator.h
 *
 *  Created on: 02.04.2020
 *      Author: samue
 */

#ifndef PI_REGULATOR_H_
#define PI_REGULATOR_H_



/**
* pi_regulator_regulator
*
* @brief   					PI controller
*
* @param sensor_number		sensor value: value of sensor
* 							goal: sensor value goal
*
* @return					-speed_to_wall: sign negative: because value of sensor gets bigger if distance of the sensor to the wall gets smaller
*/
int pi_regulator_regulator(int dist, float goal);


#endif /* PI_REGULATOR_H_ */
