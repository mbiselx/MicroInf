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

//----specific personal includes----
#include "constants.h"
#include "map.h"
#include "transmission.h"

//----debug includes----
#include <pal.h>
#include <chprintf.h>


//----definitions----

/*
 * coordinate system used :
 *
 * 				^ y
 * 				|  /
 * 				| /
 * 				|/\	phi
 * 	   <----------------->	x
 * 				|
 * 				|
 * 				|
 * 				v
 */

#define	E_MAX					10		//maximum deviation from a line permissible
#define L_MIN					50		//minimum length of dominant line segment
#define L_TOLERANCE				500		//distance error tolerance for a loop
#define PHI_TOLERANCE			0.5		//angular alignment error for a loop
#define MIN_POSES_TO_SEND		40		//minimum number of poses to gather before sending data to the computer (avoid swamping the BT)
#define MS_BETWEEN_POSES		100		//wait time in ms before gathering a new pose


typedef enum POSE_T {DOMINANT, TEMPORARY} Pose_t;

typedef struct POSE Pose;
struct POSE		//defined by type & (x, y) coords & distance & angle
{
	Pose_t type;
	int16_t x;
	int16_t y;
	uint32_t l;
	float phi;
	Pose * last;
	Pose * next;
};


//----declarations----
Pose* first_dom 	= NULL;		//dominant Pose list
Pose* final_dom		= NULL;
uint16_t nb_dom 	= 0;

Pose* first_temp 	= NULL;		//temporary Pose list
Pose* final_temp	= NULL;
uint16_t nb_temp 	= 0;

bool mapping_enabled 		= false;
bool transmission_enabled 	= false;

static const char* s = "START";	//transmission headers
static const char  d = 'D';
static const char  t = 'T';
static const char  eot = 4;	//end of transmission (debug)


/***************** internal functions ************************/
//----struct organizing functions----

Pose* new_pose(Pose_t type)
{
	Pose* new = (Pose*) malloc(sizeof(Pose));
	if (!new)
	{
		//TODO: something better
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

void destroy_pose(Pose* old)
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

void destroy_all_type_poses(Pose_t type)
{
	if (type == TEMPORARY)
	{
		while (first_temp)
			destroy_pose(first_temp);
	}
	else
	{
		while (first_dom)
			destroy_pose(first_dom);
	}
}

void migrate_temp_pose_to_dom(Pose * pt)
{
	if(pt == final_temp)				//cut
		final_temp = pt->last;
	if(pt == first_temp)
		first_temp = pt->next;
	if(pt->last)
		pt->last->next = pt->next;
	if(pt->next)
		pt->next->last = pt->last;

	if (!first_dom)
		first_dom = pt;
	else
		final_dom->next = pt;				//paste to end of dom list
	pt->last = final_dom;
	pt->next = NULL;
	final_dom = pt;

	nb_temp--;
	nb_dom++;
	pt->type = DOMINANT;
}

void migrate_dom_pose_to_temp(Pose * pt)
{
	if(pt == final_dom)				//cut
		final_dom = pt->last;
	if(pt == first_dom)
		first_dom = pt->next;
	if(pt->last)
		pt->last->next = pt->next;
	if(pt->next)
		pt->next->last = pt->last;

	if (!first_temp)
		first_temp = pt;
	else
		final_temp->next = pt;				//paste to end of temp list
	pt->last = final_temp;
	pt->next = NULL;
	final_temp = pt;

	nb_temp++;
	nb_dom--;
	pt->type = TEMPORARY;
}

//----map utility functions----

void log_origin(void)
{
	Pose* origin 	= new_pose(DOMINANT);
	origin->phi 	= 0;
	origin->x		= 0;
	origin->y		= 0;
	origin->l		= 0;
}

/*
 * \brief	find the maximum distance of a the temp points to a line defined by final_dom and (x,y)
 *
 * 							X (pt)
 * 							|
 * 							|
 * 			X---------------+-----------------X
 * 		(final_dom)							(x,y)
 *
 * \param	x
 * \param	y
 * \return	e_max
 */
int16_t max_error_line_fit(int16_t x, int16_t y)
{
	int e_max 	= 0;
	int	e		= 0;
	int16_t delta_x = x - final_dom->x;
	int16_t delta_y = y - final_dom->y;

	if (!delta_x && !delta_y)								//superposition of final_dom and (x,y)
		return 0;

	int a 		= (delta_x * final_dom->y) - (delta_y * final_dom->x);	//a constant
	float delta = sqrt(delta_x*delta_x + delta_y*delta_y);		//optimize sqrt()

	for (Pose* pt = first_temp; pt; pt=pt->next)
	{
		e = abs(delta_y*pt->x - delta_x*pt->y + a);				//optimize abs()
		if (e > e_max)
			e_max = e;
	}

	return (int16_t) (2*e_max/delta);

}

/*	\brief		function to log a Pose -> must be done relatively often to
 * 				keep track of the robot's position
 * 	steps_r		number of steps done by right motor since last call
 * 	steps_l		number of steps done by left motor since last call
 */
void log_pose(int16_t steps_r, int16_t steps_l)
{
	Pose* 	last 	= ((final_temp) ? final_temp : final_dom);	//last Pose to calculate from
	float 	d_phi	= (float)(steps_r-steps_l)/WHEEL_DISTANCE_STEPS;
	int32_t d_l		= (steps_l+steps_r)>>1;
	int32_t x 		= 0;
	int32_t y	 	= 0;

	// calculate coords of Pose
	if (!d_phi)											//straight line
	{
		x = last->x + d_l*cos(last->phi);
		y = last->y + d_l*sin(last->phi);
	}
	else												//curved line
	{
		x = last->x + d_l/d_phi*(sin(last->phi + d_phi) - sin(last->phi));	//could use truly obscure trig identities
		y = last->y + d_l/d_phi*(cos(last->phi) - cos(last->phi + d_phi));	//to optimize one calculation here -> TODO: is it worth it?
	}

	//decide type of pose
	if((last->l + d_l - final_dom->l) > L_MIN)			//the Pose is far enough from other dominant Poses
	{
		if (max_error_line_fit(x, y) > E_MAX)			//the maximum error is too large to fit all Poses in a single line
		{
			migrate_temp_pose_to_dom(final_temp);		//the last temp Pose becomes a dominant Pose
			destroy_all_type_poses(TEMPORARY);			//all other temp Poses are discarded

		}
	}

	Pose* new 	= new_pose(TEMPORARY);					//the newly logged Pose is added as a temp pose
	new->phi 	= last->phi + d_phi;
	new->l		= last->l + d_l;
	new->x 		= x;
	new->y 		= y;
}


bool simple_loop_detection(void)
{
	//check all previous poses to see if one exists with a similar location and a similar orientation
	Pose * ptr = first_dom->next;	// don't take the origin, it's usually not at a wall

	float d_l = 0;
	float d_phi = 0;

	while (ptr->next && (final_dom->l - ptr->l > 2*L_TOLERANCE))
	{
		d_l 	= sqrt( (ptr->x - final_dom->x)*(ptr->x - final_dom->x) + (ptr->y - final_dom->y)*(ptr->y - final_dom->y));
		d_phi 	= fmod(final_dom->phi - ptr->phi, 2*PI);

		if ((d_l < L_TOLERANCE) && (fabs(d_phi) < PHI_TOLERANCE))
		{
			destroy_all_type_poses(TEMPORARY);
			if (first_temp)
				palClearPad(GPIOD, GPIOD_LED3);		//debug

			while (first_dom != ptr)
				destroy_pose(first_dom);		// get rid of the useless part of the map

			//*** EXPERIMENTAL ZONE ***//

			int16_t e_x = (final_dom->x-first_dom->x)/nb_dom;	//error in x
			int16_t e_y = (final_dom->y-first_dom->y)/nb_dom;	//error in y

			ptr = first_dom->next;								// first point doesn's get corrected
			for (int i = 1; ptr; i++, ptr = ptr->next)
			{
				ptr->x = ptr->x - e_x*i;
				ptr->y = ptr->y - e_y*i;
			}

			//*** END EXPERIMENTAL ZONE ***//

			return true;
		}

		ptr = ptr->next;
	}
	return false;
}


//----transmission functions----

void map_send_poses_to_computer(Pose_t type)
{
	Pose* ptr = (type == TEMPORARY)? first_temp : first_dom;
	static const uint16_t zero = 0;
	uint16_t data_size = 5*((type == TEMPORARY)? nb_temp : nb_dom);

	if (!ptr)									//no Poses to transmit
	{
		return;
	}

	send_str_to_computer((char*)s, 5);								//transmission start character
	send_int16_to_computer((int16_t*)&data_size);									//transmission size in bytes
	send_int16_to_computer((int16_t*) ((type == TEMPORARY)? &zero : &nb_dom));		//number of dom_poses to be transmitted
	send_int16_to_computer((int16_t*) ((type == TEMPORARY)? &nb_temp : &zero));		//number of temp_poses to be transmitted
	while (ptr)
	{
		send_char_to_computer((char*)((type == TEMPORARY)? &t : &d));	//Pose start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}

	send_char_to_computer((char*)&eot);

}

void map_send_all_poses_to_computer(void)
{
	Pose* ptr = first_dom;
	uint16_t data_size = 5*(nb_temp + nb_dom);

	send_str_to_computer((char*)s, 5);					//transmission start character
	send_int16_to_computer((int16_t*)&data_size);		//transmission size in bytes
	send_int16_to_computer((int16_t*) &nb_dom);			//number of Poses to be transmitted
	send_int16_to_computer((int16_t*) &nb_temp);		//number of walls to be transmitted

	while (ptr)
	{
		send_char_to_computer((char*)&d);			//dom Pose start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}

	ptr = first_temp;
	while (ptr)
	{
		send_char_to_computer((char*)&t);			//temp Pose start character
		send_int16_to_computer(&(ptr->x));
		send_int16_to_computer(&(ptr->y));
		ptr = ptr->next;
	}


	send_char_to_computer((char*)&eot);
}



/************************ THREAD ***************************/
static THD_WORKING_AREA(waThdMap, 1024);
static THD_FUNCTION(ThdMap, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time 	= chVTGetSystemTime();
    uint8_t limiter = 0;

    serial_start();

    while(true){

    	if (mapping_enabled)
    	{
			log_pose(right_motor_get_pos(), left_motor_get_pos());
			right_motor_set_pos(0);
			left_motor_set_pos(0);
			limiter++;

			palTogglePad(GPIOD, GPIOD_LED1);
    	}

    	if (transmission_enabled && (limiter > MIN_POSES_TO_SEND))
    	{
    		//palClearPad(GPIOD, GPIOD_LED3);		//debug
    		map_send_all_poses_to_computer();
    		limiter = 0;
    		//palSetPad(GPIOD, GPIOD_LED3);		//debug

	    	if (first_dom->next && simple_loop_detection())
	    	{

	    		// TODO -> apply angle and distance correction

				palClearPad(GPIOD, GPIOD_LED5);		//debug
				chThdSleepMilliseconds(1000);
				map_send_poses_to_computer(DOMINANT);
				chThdSleepMilliseconds(1000);
				map_send_poses_to_computer(DOMINANT);	//send twice because sometimes windows doesn't receive ??
				mapping_enabled = false;
				transmission_enabled = false;
	    	}

    	}

        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(MS_BETWEEN_POSES));
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


//---- debug ----
void map_display_poses(Pose_t type)
{
	Pose* ptr = (type == TEMPORARY)? first_temp : first_dom;
	if (!ptr)
	{
		chprintf((BaseSequentialStream*)&SD3, "\r\n--- NO POSES --- \r\n");
		return;
	}
	chprintf((BaseSequentialStream*)&SD3, "\r\n--- START --- \r\n");
	chprintf((BaseSequentialStream*)&SD3, "nb_poses : %d \r\n", (type == TEMPORARY)? nb_temp : nb_dom);
	while (ptr->next)
	{
		chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t phi: %f \r\n", ptr->x,ptr->y,ptr->phi);
		ptr=ptr->next;
	}
	chprintf((BaseSequentialStream*)&SD3, "x: %d   \t y: %d \t phi: %f \r\n", ptr->x,ptr->y,ptr->phi);
	chprintf((BaseSequentialStream*)&SD3, "--- END --- \r\n");
}

void map_display_all_poses(void)
{
	//TODO
}
