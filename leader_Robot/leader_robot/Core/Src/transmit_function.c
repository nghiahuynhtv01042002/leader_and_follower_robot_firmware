/*
 * transmit_function.c
 *
 *  Created on: Feb 3, 2024
 *      Author: Nghia
 */
#include "transmit_function.h"
#include "main.h"


void Convert_int32_to_String4(uint8_t* Tx_buff,int32_t data){
	Tx_buff[0] = (data >> 24) & 0xFF;
	Tx_buff[1] = (data >> 16) & 0xFF;
	Tx_buff[2] = (data >> 8) & 0xFF;
	Tx_buff[3] = data & 0xFF;
}
void Merge_frame_data_transmit(uint8_t* tx_buffer,uint8_t buffer1[],uint8_t buffer2[]){
	tx_buffer[0] = '#';
	tx_buffer[9] = '\n';
	for (int i = 1; i < 9; i++) {
	    if (i <= 4) {
	      tx_buffer[i] = buffer1[i - 1];
	    } else {
	      tx_buffer[i] = buffer2[i - 5];
	    }
	  }
}

