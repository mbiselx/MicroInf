/*
 * transmission.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Michael Biselx
 */

#ifndef TRANSMISSION_H_
#define TRANSMISSION_H_

/* \brief		starts a serial connection for SD3
 */
void serial_start(void);

/* \brief		send a string of chars to computer (like chprintf)
 */
void send_str_to_computer(char* data, uint16_t strleng);

/* \brief		send an int16 to computer in binary
 */
void send_int16_to_computer(int16_t* data);

/* \brief		send an float to computer in binary
 */
void send_float_to_computer(float* data);


//---- experimental ----

/*
 *  experimental functions -> detect errors when transmitting via bluetooth
 *  not used in final version : too slow and cumbersome
 * 	needs more work before it can be useful
 */
void send_int16_to_computer_w_parity(int16_t* data);

void send_float_to_computer_w_parity(float* data);


#endif /* TRANSMISSION_H_ */
