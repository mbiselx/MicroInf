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

typedef struct POINT Point_t;
typedef struct WALL  Wall_t;

struct POINT		//defined by x, y coords
{
	int16_t x;
	int16_t y;
	float alpha;
	Point_t * last;
	Point_t * next;
};

struct WALL			//defined by y = a+bx, and P_start, P_end
{
	float a;
	float b;
	int16_t x1;
	int16_t y1;
	int16_t x2;
	int16_t y2;
	Wall_t * last;
	Wall_t * next;
};


//----declarations----
Point_t* start_p = NULL;
Point_t* end_p   = NULL;
uint16_t nb_points = 0;

Wall_t * start_w = NULL;
Wall_t * end_w   = NULL;
uint16_t nb_walls = 0;


//----internal functions----
Point_t* new_point(void)
{
	Point_t* new = (Point_t*) malloc(sizeof(Point_t));
	if (!new)
	{
		return NULL;	//i'm going to hate myself for this
	}

	if (!start_p)
	{
		start_p = new;
		new->last = NULL;
	}
	else
	{
		end_p->next = new;
		new->last = end_p;
	}
	end_p = new;
	new->next = NULL;

	nb_points++;

	return new;
}

void destroy_point(Point_t* old)
{
	if (old == start_p)
	{
		if (old->next)
		{
			start_p = old->next;
			old->next->last = NULL;
		}
		else
			start_p = NULL;
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
	while (start_p)
	{
		destroy_point(start_p);
	}
}

Wall_t* new_wall(void)
{
	Wall_t* new = (Wall_t*) malloc(sizeof(Wall_t));
	if (!new)
	{
		return NULL;	//i'm going to hate myself for this
	}

	if (!start_w)
	{
		start_w = new;
		new->last = NULL;
	}
	else
	{
		end_w->next = new;
		new->last = end_w;
	}
	end_w = new;
	new->next = NULL;

	nb_walls++;

	return new;
}

void destroy_wall(Wall_t* old)
{
	if (old == start_w)
	{
		if (old->next)
		{
			start_w = old->next;
			old->next->last = NULL;
		}
		else
			start_w = NULL;
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
	nb_walls--;
}

void destroy_all_walls(void)
{
	while (start_w)
	{
		destroy_wall(start_w);
	}
}


//----public functions----
void map_log_new_point(int16_t steps_r, int16_t steps_l)
{
	Point_t* new = new_point();

	float r = 0;

	if (!(new->last))
	{
		new->alpha = (float)(steps_l-steps_r)/WHEEL_DISTANCE_STEPS;
		if (new->alpha != 0)
		{
			r = (steps_l+steps_r)/2/(new->alpha);
			new->x = r*(cos(new->alpha) - 1);
			new->y = r*sin(new->alpha);
		}
		else
		{
			new->x = 0;
			new->y = steps_r;
		}
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

void map_log_wall_encounter(void)
{
	int16_t x   = end_p->x;				//save values of last point
	int16_t y   = end_p->y;
	float alpha = end_p->alpha;
	destroy_all_points();				//clean list of useless points
	Point_t * point = new_point();		//create beginning point of next wall
	point->x = x;
	point->y = y;
	point->alpha = alpha;
}

void map_log_wall_loss(void)
{
	;
}

void map_log_corner(void)
{
	float bp = 0;
	float bq = 0;
	float mean_x = 0;
	float mean_y = 0;
	float alpha = end_p->alpha;
	Point_t * point;
	Wall_t * wall = new_wall();

	wall->x1 = start_p->x;		//fill in wall start- and end-points
	wall->y1 = start_p->y;
	wall->x2 = end_p->x;
	wall->y2 = end_p->y;

	//calculate least-squares linear regression line
	for (point = start_p; point; point = point->next)
	{
		mean_x += point->x;
		mean_y += point->y;
	}
	mean_x = mean_x / nb_points;
	mean_y = mean_y / nb_points;

	for (point = start_p; point; point = point->next)
	{
		bp += point->y * (point->x - mean_x);
		bq += (point->x - mean_x)*(point->x - mean_x);				//could be optimized
	}
	wall->b = bp/bq;
	wall->a = mean_y - wall->b * mean_x;

	destroy_all_points();		//clean list of now-useless points
	point = new_point();		//create new beginning point of next wall
	point->x = wall->x2;
	point->y = wall->y2;
	point->alpha = alpha;
}



//----debug functions----
void map_display_all_points()
{
	Point_t* ptr = start_p;
	if (!start_p)
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

void map_send_all_points_to_computer(void)
{
	Point_t* ptr = start_p;
	static const char s = 'S';
	static const char p = 'P';
	static const int16_t zero = 0;
	uint16_t data_size = 5*nb_points;

	if (!ptr)									//no points to transmit
	{
		return;
	}

	send_char_to_computer((char*)&s);					//transmission start character
	send_int16_to_computer((int16_t*)&data_size);			//transmission size in bytes
	send_int16_to_computer((int16_t*)&nb_points);			//number of points to be transmitted
	send_int16_to_computer((int16_t*)&zero);				//number of walls to be transmitted
	while (ptr->next)
	{
		send_char_to_computer((char*)&p);				//point start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}
	send_char_to_computer((char*)&p);
	send_int16_to_computer(&(ptr->x));
	send_int16_to_computer(&(ptr->y));

}

void map_send_all_walls_to_computer(void)
{
	Wall_t* ptr = start_w;
	static const char s = 'S';
	static const char w = 'W';
	static const int16_t zero = 0;
	uint16_t data_size = 13*nb_walls;

	if (!ptr)									//no walls to transmit
	{
		return;
	}

	send_char_to_computer((char*)&s);					//transmission start character
	send_int16_to_computer((int16_t*)&data_size);			//transmission size in bytes
	send_int16_to_computer((int16_t*)&zero);				//number of points to be transmitted
	send_int16_to_computer((int16_t*)&nb_walls);			//number of walls to be transmitted
	while (ptr->next)
	{
		send_char_to_computer((char*)&w);				//wall start character
		send_float_to_computer(&(ptr->a));
		send_float_to_computer(&(ptr->b));
		send_int16_to_computer(&(ptr->x2));
		send_int16_to_computer(&(ptr->y2));
		ptr = ptr->next;
	}
	send_char_to_computer((char*)&w);
	send_float_to_computer(&(ptr->a));
	send_float_to_computer(&(ptr->b));
	send_int16_to_computer(&(ptr->x2));
	send_int16_to_computer(&(ptr->y2));

}


void map_send_all_data_to_computer(void)
{
	Wall_t* ptr_w = start_w;
	Point_t* ptr_p = start_p;
	static const char s = 'S';
	static const char p = 'P';
	static const char w = 'W';
	uint16_t data_size = 5*nb_points + 13*nb_walls;


	if (!(ptr_p || ptr_w) )									//no data to transmit
	{
		return;
	}

	send_char_to_computer((char*)&s);					//transmission start character
	send_int16_to_computer((int16_t*)&data_size);			//transmission size in bytes
	send_int16_to_computer((int16_t*)&nb_points);			//number of points to be transmitted
	send_int16_to_computer((int16_t*)&nb_walls);			//number of walls to be transmitted

	while (ptr_p->next)							//send all points first
	{
		send_char_to_computer((char*)&p);				//point start character
		send_int16_to_computer(&(ptr_p->x));
		send_int16_to_computer(&(ptr_p->y));
		ptr_p = ptr_p->next;
	}
	send_char_to_computer((char*)&p);
	send_int16_to_computer(&(ptr_p->x));
	send_int16_to_computer(&(ptr_p->y));

	while (ptr_w->next)							//send all walls
	{
		send_char_to_computer((char*)&w);				//wall start character
		send_float_to_computer(&(ptr_w->a));
		send_float_to_computer(&(ptr_w->b));
		send_int16_to_computer(&(ptr_w->x2));
		send_int16_to_computer(&(ptr_w->y2));
		ptr_w = ptr_w->next;
	}
	send_char_to_computer((char*)&w);
	send_float_to_computer(&(ptr_w->a));
	send_float_to_computer(&(ptr_w->b));
	send_int16_to_computer(&(ptr_w->x2));
	send_int16_to_computer(&(ptr_w->y2));

}
