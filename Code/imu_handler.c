/*
 * imu_handler.c
 *
 *  Created on: 09.04.2020
 *      Author: samuel
 */

#include <ch.h>
#include <hal.h>
#include <sensors\imu.h>
#include <leds.h>
#include <math.h>
#include <move.h>
#include <msgbus\messagebus.h>
#include <i2c_bus.h>

//----specific personal includes----
#include "main.h"

//----debug includes----
#include <chprintf.h>

#define TIME_BETWEEN_SAMPLES			200//ms
#define ACC_MAX							3000 //not in m/s^2
#define TIME_WITHOUT_ACC_OVER_ACC_MAX 	4000//ms

/*
 * This Thread controlls if
 *
 */
static THD_WORKING_AREA(waThdImuHandler, 1024);
static THD_FUNCTION(ThdImuHandler, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time 	= chVTGetSystemTime();


    chThdSleepSeconds(2);//wait for 2s otherwise robot can calibrate too fast after reset
    calibrate_acc();
    clear_leds();
    set_led(LED7, 1);//LED7 is on if calibrating is finished

    static systime_t time_before = false;	//system time when last time ACC_x, ACC_Y or ACC_Z was bigger than ACC_MAX
        									//time_before == 0 if ACC_x, ACC_Y or ACC_Z was not jet bigger than ACC_MAX and thus robot was not jet moved
    										//(functions the same time as a flag if robot was moved or not -> )

    while(true){
    	if(move_get_drive_mode()){//if move in drive_mode this thread gets terminated
    		imu_stop();
    		chThdExit(0);//Terminates current thread
    	}
    	if(abs(get_acc(X_AXIS)-get_acc_offset(X_AXIS))>=ACC_MAX || abs(get_acc(Y_AXIS)-get_acc_offset(Y_AXIS))>=ACC_MAX || abs(get_acc(Z_AXIS)-get_acc_offset(Z_AXIS))>=ACC_MAX){
    		time_before = chVTGetSystemTime();//time_before is the last time ACC_x, ACC_Y or ACC_Z was bigger than ACC_MAX
    	}

    	if(((chVTGetSystemTime()-time_before)>TIME_WITHOUT_ACC_OVER_ACC_MAX) && time_before){
    		move_update_drive_mode();//if robot was moved and then was not moved during the last TIME_WITHOUT_ACC_OVER_ACC_MAX ms -> drive_mode is updated to true
    	}
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(TIME_BETWEEN_SAMPLES));
    }
}

void imu_handler_init(void){
	chThdCreateStatic(waThdImuHandler, sizeof(waThdImuHandler), NORMALPRIO-2, ThdImuHandler, NULL);
}
