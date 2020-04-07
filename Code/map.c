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
#include <pal.h>
#include <chprintf.h>


//----definitions----
#define L_MIN					50		//minimum length of line segment, squared
#define	E_MAX					20		//maximum deviation from a line permissible
#define MIN_POINTS_TO_SEND		40
#define MS_BETWEEN_POINTS		100

typedef enum POINT_T {DOMINANT, TEMPORARY} Point_t;

typedef struct POINT Point;
struct POINT		//defined by type & (x, y) coords
{
	Point_t type;
	int16_t x;
	int16_t y;
	float alpha;
	Point * last;
	Point * next;
};


//----declarations----
Point* first_dom 	= NULL;		//dominant point list
Point* final_dom	= NULL;
uint16_t nb_dom 	= 0;

Point* first_temp 	= NULL;		//temporary point list
Point* final_temp	= NULL;
uint16_t nb_temp 	= 0;

bool mapping_enabled 		= false;
bool transmission_enabled 	= false;

static const char* s = "START";	//transmission headers
static const char  d = 'D';
static const char  t = 'T';
static const char  eot = 'f';	//end of transmission (debug)


/***************** internal functions ************************/
//----struct organizing functions----

Point* new_point(Point_t type)
{
	Point* new = (Point*) malloc(sizeof(Point));
	if (!new)
	{
		return NULL;	//i'm going to hate myself for this
	}

	if (type == TEMPORARY)
	{
		if(!first_temp)
		{
			first_temp = new;
			new->last = NULL;
		}
		else
		{
			final_temp->next = new;
			new->last = final_temp;
		}
		final_temp = new;
		nb_temp++;
	}
	else
	{
		if(!first_dom)
		{
			first_dom = new;
			new->last = NULL;
		}
		else
		{
			final_dom->next = new;
			new->last = final_dom;
		}
		final_dom = new;
		nb_dom++;
	}

	new->next = NULL;
	new->type = type;

	return new;
}

void destroy_point(Point* old)
{
	if (old == first_temp)
	{
		if (old->next)
		{
			first_temp = old->next;
			old->next->last = NULL;
		}
		else
			first_temp = NULL;
	}
	else
	{
		if (old == first_dom)
		{
			if (old->next)
			{
				first_dom = old->next;
				old->next->last = NULL;
			}
			else
				first_dom = NULL;
		}

		else
		{
			if(old->last)
				old->last->next = old->next;
			if(old->next)
				old->next->last = old->last;
		}
	}

	if (old->type == TEMPORARY)
		nb_temp--;
	else
		nb_dom--;

	free(old);
	old = NULL;
}

void destroy_all_type_points(Point_t type)
{
	if (type == TEMPORARY)
	{
		while (first_temp)
			destroy_point(first_temp);
		if (nb_temp)
			palClearPad(GPIOD, GPIOD_LED5);		//debug
	}
	else
	{
		while (first_dom)
			destroy_point(first_dom);
	}
}


//----map utility functions----

int16_t max_error_line_fit(int16_t x, int16_t y)
{
	Point* pt = first_temp;
	int16_t e_max 	= 0;
	int16_t	e		= 0;
	int16_t delta_x = x - final_dom->x;
	int16_t delta_y = y - final_dom->y;

	if (!delta_x && !delta_y)			//superposition
		return 0;

	int16_t a 		= (delta_y * x) - (delta_x * y);
	float delta = sqrt(delta_x*delta_x + delta_y*delta_y);		//optimize sqrt()

	while (pt)
	{
		e = abs(delta_x*pt->y - delta_y*pt->x + a);				//optimize abs()
		if (e > e_max)
			e_max = e;
		pt = pt->next;
	}

	return (int16_t) (2*e_max/delta);

}

void migrate_temp_point_to_dom(Point * pt)
{
	if(pt == final_temp)				//cut
		final_temp = pt->last;
	if(pt == first_temp)
		first_temp = pt->next;
	if(pt->last)
		pt->last->next = pt->next;
	if(pt->next)
		pt->next->last = pt->last;

	final_dom->next = pt;				//paste to end of dom list
	pt->last = final_dom;
	pt->next = NULL;
	final_dom = pt;

	nb_temp--;
	nb_dom++;
	pt->type = DOMINANT;

}

/*	\brief		function to log a point -> must be done relatively often to
 * 				keep track of the robot's position
 * 	steps_r		number of steps done by right motor since last call
 * 	steps_l		number of steps done by left motor since last call
 */
void log_new_temp_point(int16_t steps_r, int16_t steps_l)
{
	Point* 	last 	= ((final_temp) ? final_temp : final_dom);	//last point to calculate from
	float 	alpha 	= 0;
	float 	r 		= 0;
	int16_t x 		= 0;
	int16_t y	 	= 0;
	int		d		= 0;

	// calculate coords of point
	alpha = last->alpha + (float)(steps_r-steps_l)/WHEEL_DISTANCE_STEPS;
	if (alpha == last->alpha)							//straight line
	{
		x = last->x - steps_r*sin(alpha);
		y = last->y + steps_r*cos(alpha);
	}
	else												//curved line
	{
		r = (steps_l+steps_r)/2/(alpha - last->alpha);
		x = last->x + r*(cos(alpha) - cos(last->alpha));	//could use truly obscure trig identities
		y = last->y + r*(sin(alpha) - sin(last->alpha));	//to optimize one calculation here -> TODO: is it worth it?
	}

	//decide type of point
	d = sqrt((final_dom->x - x)*(final_dom->x - x) + (final_dom->y - y)*(final_dom->y - y));	//distance between this point and last dominant point
	if (d > L_MIN)											//the point is far enough from other dominant points
	{
		if (max_error_line_fit(x, y) > E_MAX)				//the maximum error is too large to fit all points in a single line
		{
			migrate_temp_point_to_dom(final_temp);			//the last temp point becomes a dominant point
			destroy_all_type_points(TEMPORARY);				//all other temp points are discarded
		}
	}

	Point* new 	= new_point(TEMPORARY);					//the newly logged point is added as a temp point
	new->alpha 	= alpha;
	new->x 		= x;
	new->y 		= y;
}

void log_origin(void)
{
	Point* origin 	= new_point(DOMINANT);
	origin->alpha 	= 0;
	origin->x		= 0;
	origin->y		= 0;
}


/************************ THREAD ***************************/
static THD_WORKING_AREA(waThdMap, 1024);
static THD_FUNCTION(ThdMap, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time 	= chVTGetSystemTime();
    float h 		= 0;
    float alpha_imu = 0;
    uint8_t limiter = 0;

    serial_start();

    while(true){

    	h = ((float)(chVTGetSystemTime()-time)) * 0.001;
    	time = chVTGetSystemTime();
    	alpha_imu += h*get_gyro_rate(2); //integrate rate to get value

		palTogglePad(GPIOD, GPIOD_LED1);

    	if (mapping_enabled)
    	{
			log_new_temp_point(right_motor_get_pos(),left_motor_get_pos());
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			limiter++;

			/*if (alpha_imu < -2*PI || 2*PI < alpha_imu)
			{
				palClearPad(GPIOD, GPIOD_LED3);	//indicate a full turn has been made
			}*/

    	}

    	if (transmission_enabled && (limiter > MIN_POINTS_TO_SEND))
    	{
    		palClearPad(GPIOD, GPIOD_LED3);		//debug
    		map_send_all_points_to_computer();
    		limiter = 0;
    		palSetPad(GPIOD, GPIOD_LED3);		//debug
    	}


        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(MS_BETWEEN_POINTS));
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
	log_origin();
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
void map_display_points(Point_t type)
{
	Point* ptr = (type == TEMPORARY)? first_temp : first_dom;
	if (!ptr)
	{
		chprintf((BaseSequentialStream*)&SD3, "\r\n--- NO POINTS --- \r\n");
		return;
	}
	chprintf((BaseSequentialStream*)&SD3, "\r\n--- START --- \r\n");
	chprintf((BaseSequentialStream*)&SD3, "nb_points : %d \r\n", (type == TEMPORARY)? nb_temp : nb_dom);
	while (ptr->next)
	{
		chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t alpha: %f \r\n", ptr->x,ptr->y,ptr->alpha);
		ptr=ptr->next;
	}
	chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t alpha: %f \r\n", ptr->x,ptr->y,ptr->alpha);
	chprintf((BaseSequentialStream*)&SD3, "--- END --- \r\n");
}

void map_send_points_to_computer(Point_t type)
{
	Point* ptr = (type == TEMPORARY)? first_temp : first_dom;
	static const uint16_t zero = 0;
	uint16_t data_size = 5*((type == TEMPORARY)? nb_temp : nb_dom);

	if (!ptr)									//no points to transmit
	{
		return;
	}

	send_str_to_computer((char*)s, 5);								//transmission start character
	send_int16_to_computer((int16_t*)&data_size);									//transmission size in bytes
	send_int16_to_computer((int16_t*) ((type == TEMPORARY)? &zero : &nb_dom));		//number of dom_points to be transmitted
	send_int16_to_computer((int16_t*) ((type == TEMPORARY)? &nb_temp : &zero));		//number of temp_points to be transmitted
	while (ptr)
	{
		send_char_to_computer((char*)((type == TEMPORARY)? &t : &d));	//point start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}

	send_char_to_computer((char*)&eot);

}

void map_display_all_points(void)
{
	//TODO
}

void map_send_all_points_to_computer(void)
{
	Point* ptr = first_dom;
	uint16_t data_size = 5*(nb_temp + nb_dom);

	send_str_to_computer((char*)s, 5);					//transmission start character
	send_int16_to_computer((int16_t*)&data_size);		//transmission size in bytes
	send_int16_to_computer((int16_t*) &nb_dom);			//number of points to be transmitted
	send_int16_to_computer((int16_t*) &nb_temp);		//number of walls to be transmitted

	while (ptr)
	{
		send_char_to_computer((char*)&d);			//dom point start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}

	ptr = first_temp;
	while (ptr)
	{
		send_char_to_computer((char*)&t);			//temp point start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}


	send_char_to_computer((char*)&eot);
}

