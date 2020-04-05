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
	int acc_z = 0,
		jerk_z = 0;

    systime_t time = chVTGetSystemTime();

    //wait for robot to be stable, then calibrate
    calibrate();

    move_main();			//contains infinite loop


    /* Infinite loop. */
    while (1)
    {
		acc_z = get_acc(2);
    	//waits .01 seconds
        chThdSleepMilliseconds(10);

        omega = get_gyro_rate(2);				//current angle calculation
		h  = 0.001 * (chVTGetSystemTime() - time);
		time = chVTGetSystemTime();
		alpha += h*((-THRESH_GYRO < omega && omega < THRESH_GYRO) ? 0 : omega);

		/*bool disturbed = false;					//disturbance detection
		jerk_z = acc_z - get_acc(2);
		while (jerk_z < -THRESH_J || jerk_z > THRESH_J)
		{
			//disturbance event - wait 1 second
			acc_z = get_acc(2);
			disturbed = true;
			chprintf((BaseSequentialStream*)&SD3, ".");
			chThdSleepMilliseconds(1000);
			jerk_z = acc_z - get_acc(2);
		}
		if (disturbed)							//recalibrate
		{
			chprintf((BaseSequentialStream*)&SD3, "disturbance detected\r\n");
			calibrate();
		}*/

        //debugging data output
        //chprintf((BaseSequentialStream*)&SD3, "alpha = %f rad \t h = %f s \r\n", alpha, h);
        //chprintf((BaseSequentialStream*)&SD3, "x: %d \t y: %d \t z: %d \t \r\n", get_acc(0)-get_acc_offset(0),get_acc(1) - get_acc_offset(1), acc_z);
        //chprintf((BaseSequentialStream*)&SD3, "0: %d   \t 1: %d \t 2: %d \t 3: %d \r\n", get_prox(0),get_prox(1),get_prox(2),get_prox(3));
        //chprintf((BaseSequentialStream*)&SD3, "4: %d   \t 5: %d \t 6: %d \t 7: %d \r\n", get_prox(4),get_prox(5),get_prox(6),get_prox(7));
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
