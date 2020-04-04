/*
 * transmission.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Michael Biselx
 */

#ifndef TRANSMISSION_H_
#define TRANSMISSION_H_

void serial_start(void);

void send_char_to_computer(char* data);

void send_int8_to_computer(uint8_t* data);

void send_int16_to_computer(uint16_t* data);


#endif /* TRANSMISSION_H_ */
