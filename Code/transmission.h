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

void send_str_to_computer(char* data, uint16_t strleng);

void send_int8_to_computer(uint8_t* data);

void send_int16_to_computer(int16_t* data);

void send_float_to_computer(float* data);

/*
 * experimental functions -> detect errors when transmitting via bluetooth
 */
void send_int16_to_computer_w_parity(int16_t* data);

void send_float_to_computer_w_parity(float* data);


#endif /* TRANSMISSION_H_ */
