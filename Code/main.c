//----stdlib includes----
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//----global chibiOS includes----
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

//----specific epuck2 includes----
#include <sensors\proximity.h>
#include <sensors\imu.h>
#include <motors.h>

//----specific personal inlcudes----
#include "move.h"
#include "transmission.h"
#include "map.h"

//----debug includes----
#include <chprintf.h>


//----defines----
#define THRESH_GYRO		0.01f
#define THRESH_J 		1500

//----global declarations----
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//----internal functions----

void calibrate(void)
{
    chThdSleepMilliseconds(500);
    calibrate_ir();
    calibrate_acc();
    calibrate_gyro();
    chThdSleepMilliseconds(500);
}

//----MAIN----
int main(void)
{
	//global inits
    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);	//If you don't do this nothing works right

    //specific inits (sensors & motors)
    proximity_start();
    imu_start();
    motors_init();

    //debug inits
    serial_start();


    //variable declarations
	float omega = 0,
		  alpha = 0,
		  h = 0;

    systime_t time = chVTGetSystemTime();

    //wait for robot to be stable, then calibrate
    calibrate();

    move_handler();


    /* Infinite loop. */
    while (1)
    {
    	;
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
