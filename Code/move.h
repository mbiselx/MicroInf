#ifndef MOVE_H_
#define MOVE_H_



/**
* move_handler
*
* @brief   					function handles movement of robot
*
* @param sensor_number		-
*
* @return					-
*/
void move_handler(void);



/**
* move_update_drive_mode
*
* @brief   					updates the state of driv_mode (changes state from true to false or from false to true)
*
* @param r					-
*
* @return					-
*/
void move_update_drive_mode(void);



/**
* move_get_drive_mode
*
* @brief   					returns state of drieve_mode
*
* @param r					-
*
* @return					state of drive_mode (true or false)
*/
uint8_t move_get_drive_mode(void);

#endif /* MOVE_H_ */
