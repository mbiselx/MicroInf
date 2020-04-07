/*
 * pi_regulator.c
 *
 *  Created on: 02.04.2020
 *      Author: samuel
 */


#include "ch.h"
#include "hal.h"
#include <math.h>



#define KP 150
#define KI 20
#define ERROR_TRESHOLD 30
#define MAX_SUM_ERROR 300
#define MIN_SUM_ERROR -300


//simple PI regulator
int pi_regulator_regulator(int dist, float goal){
	float error_i=0;
	float speed_to_wall=0;

	static float sum_error=0;

	error_i= dist -goal;

	if(fabs(error_i) < ERROR_TRESHOLD)
		return 0;

	sum_error+=error_i;

	if(sum_error>MAX_SUM_ERROR)
		sum_error=MAX_SUM_ERROR;
	if(sum_error<MIN_SUM_ERROR)
		sum_error=MIN_SUM_ERROR;

	speed_to_wall = KP*error_i + KI*sum_error;


	return (int) -speed_to_wall;
}
