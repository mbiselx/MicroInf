//----stdlib includes----
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//----global chibiOS includes----
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

//----specific epuck2 includes----
#include <sensors\proximity.h>
#include <sensors\imu.h>
#include <motors.h>

//----specific personal includes----
#include "main.h"
#include "move.h"
#include "map.h"
#include "imu_handler.h"
#include "transmission.h"

//----debug includes----
#include <chprintf.h>


//----global declarations----
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


//----MAIN----
int main(void)
{
	//global inits
    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);	//If you don't do this nothing works right

    //specific inits (sensors & motors)
    serial_start();
    proximity_start();
    imu_start();
    imu_handler_init();
    motors_init();
    map_init();

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
