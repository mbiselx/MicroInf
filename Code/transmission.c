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
	sdStart(&SD3, &ser_cfg); // UART3 -> COM9(USB) and COM13(Bluetooth)  (for me)
}

void send_char_to_computer(char* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(char));
}

void send_int8_to_computer(uint8_t* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(int8_t));
}

void send_int16_to_computer(int16_t* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(int16_t));
}

void send_float_to_computer(float* data)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(float));
}


//experimental
void send_int16_to_computer_w_parity(int16_t* data)
{
	uint8_t parity = 0;
	uint16_t sum = 0;
	for (uint8_t i = 15; i > 0; i--)
	{
		sum += ((*data)>>i) & 0x0001;
	}
	sum += ((*data) & 0x0001);
	parity = (!(sum & 0x01)) | ((!(sum % 3))<<1);		//even 2 & 3 parity

	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(int16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, &parity, sizeof(uint8_t));
}

void send_float_to_computer_w_parity(float* data)
{
	uint8_t parity = 0;
	uint32_t sum = 0;
	for (uint8_t i = 15; i > 0; i--)
	{
		sum += (((uint32_t)(*data))>>i) & 0x00000001;
	}
	sum += (((uint32_t)(*data)) & 0x00000001);
	parity = (!(sum & 0x01)) | ((!(sum % 3))<<1);		//even 2 & 3 parity

	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, sizeof(float));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, &parity, sizeof(uint8_t));
}
