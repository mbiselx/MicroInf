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

//----debug includes----
#include <chprintf.h>


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

    //wait for robot to be stable, then calibrate
    calibrate();

    map_init();
    map_start_mapping(true);

    move_handler();

    /* Infinite loop. */
    while (1)
    {
    	/*
    	right_motor_set_speed(700);
    	left_motor_set_speed(700);
    	chThdSleepMilliseconds(2000);

    	right_motor_set_speed(0);
    	left_motor_set_speed(0);
    	chThdSleepMilliseconds(10);

    	right_motor_set_speed(500);
    	left_motor_set_speed(-500);
    	chThdSleepMilliseconds(510);

    	right_motor_set_speed(0);
    	left_motor_set_speed(0);
    	chThdSleepMilliseconds(10);*/
    	;
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
