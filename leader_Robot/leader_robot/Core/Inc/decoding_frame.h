/*
 * decoding_frame.h
 *
 *  Created on: Mar 31, 2024
 *      Author: Nghia
 */

#ifndef INC_DECODING_FRAME_H_
#define INC_DECODING_FRAME_H_

#include "stdio.h"
#include "stdlib.h"
typedef struct{
	char cmd_d[4];
	float x_d;
	float y_d;
	float phi_d;
}desired_point;
void desired_point_init(desired_point* my_desired_point);
void split_frame(char *frame,desired_point* my_desired_point);


#endif /* INC_DECODING_FRAME_H_ */
