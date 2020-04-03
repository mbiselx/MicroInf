#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <sensors\proximity.h>
#include "memory_protection.h"
#include <main.h>
#include <move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	//inits
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    proximity_start();
    calibrate_ir();

    motors_init();

    move_main();

    /* Infinite loop. */
    while (1)
    {

    	//waits 1 second
        chThdSleepMilliseconds(1000);
        //test2_samuel
    }}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
