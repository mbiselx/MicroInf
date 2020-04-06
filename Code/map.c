/*
 * map.c
 *
 *  Created on: 3 Apr 2020
 *      Author: Michael Biselx
 */

//----stdlib includes----
#include <stdlib.h>
#include <math.h>

//----global chibiOS includes----
#include <hal.h>

//----specific epuck2 includes----
#include <motors.h>
#include <sensors\imu.h>

//----specific personal includes----
#include "constants.h"
#include "map.h"
#include "transmission.h"

//----debug includes----
#include <chprintf.h>


//----definitions----
#define MAX_CURVATURE			0.1f
#define MIN_POINTS_TO_SEND		50
#define TIME_BETWEEN_SAMPLES	200

typedef struct POINT Point_t;
struct POINT		//defined by x, y coords
{
	int16_t x;
	int16_t y;
	float alpha;
	Point_t * last;
	Point_t * next;
};

typedef struct WALL  Wall_t;
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
Point_t* start_p = NULL;		//point list
Point_t* end_p   = NULL;
uint16_t nb_points = 0;

Wall_t * start_w = NULL;		//wall list
Wall_t * end_w   = NULL;
uint16_t nb_walls = 0;

bool mapping_enabled 		= false;
bool transmission_enabled 	= false;

static const char* s = "START";	//transmission headers
static const char  p = 'P';
static const char  w = 'W';


/***************** internal functions ************************/
//----struct organizing functions----

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

//----map utility functions----

/*	\brief		function to log a point -> must be done relatively often to
 * 				keep track of the robot's position
 * 	steps_r		number of steps done by right motor since last call
 * 	steps_l		number of steps done by left motor since last call
 */
void map_log_new_point(int16_t steps_r, int16_t steps_l)
{
	Point_t* new = new_point();

	float r = 0;

	if (!(new->last))
	{
		new->alpha = (float)(steps_r-steps_l)/WHEEL_DISTANCE_STEPS;
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
		new->alpha = new->last->alpha + (float)(steps_r-steps_l)/WHEEL_DISTANCE_STEPS;
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

/*	\brief		function to log the encounter of a wall -> must be called at first encounter of a wall
 * 	TODO : make this useful
 */
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

/*	\brief		function to log the loss of a wall -> must be called at first loss of a wall
 */
void map_log_wall_loss(void)
{
	// TODO this;
}

/*	\brief		function to log the encounter of a corner (=end of a wall segment)
 *				! must be called at every corner
 */
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


/************************ THREAD ***************************/
static THD_WORKING_AREA(waThdMap, 128);
static THD_FUNCTION(ThdMap, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time 	= chVTGetSystemTime();
    float h 		= 0;
    float alpha_imu = 0;
    float curvature = 0;
    uint8_t limiter = 0;

    serial_start();

    while(true){

    	h = ((float)(chVTGetSystemTime()-time)) * 0.001;
    	time = chVTGetSystemTime();
    	alpha_imu += h*get_gyro_rate(2);

    	if (mapping_enabled)
    	{
			map_log_new_point(right_motor_get_pos(),left_motor_get_pos());
			right_motor_set_pos(0);
			left_motor_set_pos(0);

			limiter++;

			curvature = (end_p->alpha - end_p->last->alpha);
			if ((curvature < -MAX_CURVATURE) || (MAX_CURVATURE < curvature))
			{
				map_log_corner();
			}

			if (end_p->alpha < -2*PI || 2*PI < end_p->alpha)
			{
				palClearPad(GPIOD, GPIOD_LED1);	//indicate a full turn has been made
			}
			if (alpha_imu < -2*PI || 2*PI < alpha_imu)
			{
				palClearPad(GPIOD, GPIOD_LED3);	//indicate a full turn has been made
			}

    	}

    	if (transmission_enabled && (limiter > MIN_POINTS_TO_SEND))
    	{
    		map_send_all_data_to_computer();
    		limiter = 0;
    	}

        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(TIME_BETWEEN_SAMPLES));
    }
}


/***************** public functions ************************/

/*
 * \brief		initializes the thread for mapping the robot's movements
 * 				this thread - once started - automatically logs the robot's
 * 				movement and creates a map of the path it followed.
 * 				Furthermore, if so instructed, the thread will periodically
 * 				send a copy of the entire map created to the computer via Bluetooth.
 */
void map_init(void)
{
	chThdCreateStatic(waThdMap, sizeof(waThdMap), NORMALPRIO-1, ThdMap, NULL);
}

void map_start_mapping(bool transmit)
{
	map_log_new_point(0,0);
	right_motor_set_pos(0);
	left_motor_set_pos(0);
	mapping_enabled = true;
	transmission_enabled = transmit;
}

void map_pause_mapping(void)
{
	mapping_enabled = false;
	transmission_enabled = true;
}

void map_unpause_mapping(bool transmit)
{
	mapping_enabled = true;
	transmission_enabled = transmit;
}

void map_stop_mapping(void)
{
	mapping_enabled = false;
	map_log_corner();

	// TODO : create final map  & send to computer
}

/*
 * \brief		allows the user to manually add an "event" marker to the map
 */
void map_log_event(void)
{
	// TODO : do this
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
	chprintf((BaseSequentialStream*)&SD3, "nb_points : %d \r\n", nb_points);
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
	static const int16_t zero = 0;
	uint16_t data_size = 5*nb_points;

	if (!ptr)									//no points to transmit
	{
		return;
	}

	send_str_to_computer((char*)s, 5);					//transmission start character
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
	static const int16_t zero = 0;
	uint16_t data_size = 13*nb_walls;

	if (!ptr)									//no walls to transmit
	{
		return;
	}

	send_str_to_computer((char*)s, 5);					//transmission start character
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
	uint16_t data_size = 5*nb_points + 13*nb_walls;


	if (!(ptr_p || ptr_w) )									//no data to transmit
	{
		return;
	}

	send_str_to_computer((char*)s, 5);					//transmission start characters
	send_int16_to_computer((int16_t*)&data_size);			//transmission size in bytes
	send_int16_to_computer((int16_t*)&nb_points);			//number of points to be transmitted
	send_int16_to_computer((int16_t*)&nb_walls);			//number of walls to be transmitted

	if (ptr_p)
	{
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
	}

	if (ptr_w)
	{
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

}
