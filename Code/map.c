/*
 * map.c
 *
 *  Created on: 3 Apr 2020
 *      Author: Michael Biselx
 */

//----includes----
#include <stdlib.h>
#include <hal.h>
#include <math.h>

#include "map.h"
#include "transmission.h"

//----debug includes----
#include <chprintf.h>


//----definitions----
#define WHEEL_PERIMETER_CM 			13 																//cm
#define WHEEL_PERIMITER_STEPS 		1000															//steps
#define WHEEL_DISTANCE_CM      		5.28f    														//cm
#define WHEEL_DISTANCE_STEPS		(WHEEL_DISTANCE_CM*WHEEL_PERIMITER_STEPS/WHEEL_PERIMETER_CM) 	//steps

typedef	struct POINT Point_t;
struct POINT
{
	int16_t x;
	int16_t y;
	float alpha;
	Point_t * last;
	Point_t * next;
};


//----declarations----
Point_t* start = NULL;
Point_t* end   = NULL;
uint16_t nb_points = 0;


//----internal functions----

Point_t* new_point(void);
/*void destroy_point(Point_t* old);
void destroy_all_points(void);
void map_log_new_point(int16_t steps_r, int16_t steps_l);*/


Point_t* new_point(void)
{
	Point_t* new = (Point_t*) malloc(sizeof(Point_t));
	if (!new)
	{
		return NULL;	//i'm going to hate myself for this
	}

	if (!start)
	{
		start = new;
		new->last = NULL;
	}
	else
	{
		end->next = new;
		new->last = end;
	}
	end = new;
	new->next = NULL;

	nb_points++;

	return new;
}

void destroy_point(Point_t* old)
{
	if (old == start)
	{
		if (old->next)
		{
			start = old->next;
			old->next->last = NULL;
		}
		else
			start = NULL;
	}
	else
	{
		if(old->last)
			old->last->next = old->next;
		if(old->next)
			old->next->last = old->last;
	}
	free(old);
	old = NULL;
	nb_points--;
}

void destroy_all_points(void)
{
	while (start)
	{
		destroy_point(start);
	}
}


//----public functions----
void map_log_new_point(int16_t steps_r, int16_t steps_l)
{
	Point_t* new = new_point();
	float r = 0;
	if (!(new->last))
	{
		new->alpha = 0;
		new->x = 0;
		new->y = 0;
	}
	else
	{
		new->alpha = new->last->alpha + (float)(steps_l-steps_r)/WHEEL_DISTANCE_STEPS;
		if (new->alpha != new->last->alpha)
		{
			r = (steps_l+steps_r)/2/(new->alpha - new->last->alpha);
			new->x = new->last->x + r*(cos(new->alpha) - cos(new->last->alpha));
			new->y = new->last->y + r*(sin(new->alpha) - sin(new->last->alpha));
		}
		else
		{
			new->x = new->last->x - steps_r*sin(new->alpha);
			new->y = new->last->y + steps_r*cos(new->alpha);
		}
	}


}


//----debug functions----
void map_display_all_points()
{
	Point_t* ptr = start;
	if (!start)
	{
		chprintf((BaseSequentialStream*)&SD3, "\r\n--- NO POINTS --- \r\n");
		return;
	}
	chprintf((BaseSequentialStream*)&SD3, "\r\n--- START --- \r\n");
	while (ptr->next)
	{
		chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t alpha: %f \r\n", ptr->x,ptr->y,ptr->alpha);
		ptr=ptr->next;
	}
	chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t alpha: %f \r\n", ptr->x,ptr->y,ptr->alpha);
	chprintf((BaseSequentialStream*)&SD3, "--- END --- \r\n");
}

void map_send_all_data_to_computer(void)
{
	Point_t* ptr = start;
	char s = 'S';
	if (!start)
	{
		return;
	}

	send_char_to_computer(&s);
	send_int16_to_computer(&nb_points);
	while (ptr->next)
	{
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}
	send_int16_to_computer(&(ptr->x));
	send_int16_to_computer(&(ptr->y));

}

