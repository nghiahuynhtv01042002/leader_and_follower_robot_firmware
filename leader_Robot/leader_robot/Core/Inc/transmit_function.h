/*
 * transmit_function.h
 *
 *  Created on: Feb 3, 2024
 *      Author: Nghia
 */

#ifndef INC_TRANSMIT_FUNCTION_H_
#define INC_TRANSMIT_FUNCTION_H_
#include "main.h"
//extern uint8_t Tx_buff[4];
void Convert_int32_to_String4(uint8_t* Tx_buff,int32_t data);
/* */
void Merge_frame_data_transmit(uint8_t* tx_buffer,uint8_t buffer1[],uint8_t buffer2[]);
/* */
#endif /* INC_TRANSMIT_FUNCTION_H_ */
