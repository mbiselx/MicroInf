/*
 * transmission.c
 *
 *  Created on: 2 Apr 2020
 *      Author: Michael Biselx
 */

#include "ch.h"
#include "hal.h"
#include <main.h>

#include "transmission.h"

void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void send_char_to_computer(char* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, 1);
}

void send_int8_to_computer(uint8_t* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, 1);
}

void send_int16_to_computer(uint16_t* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, 2);
}
