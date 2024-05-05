/*
 * decoding_frame.c
 *
 *  Created on: Mar 31, 2024
 *      Author: Nghia
 */
#include "decoding_frame.h"
#include "string.h"
void desired_point_init(desired_point* my_desired_point){
	my_desired_point->x_d = 0;
	my_desired_point->y_d = 0;
	my_desired_point->phi_d = 0;
	strcpy(my_desired_point->cmd_d, "stp");
}
void split_frame(char *frame, desired_point* my_desired_point) {
    // Sử dụng sscanf để trích xuất giá trị từ chuỗi
    sscanf(frame, "!cmd:%[^#]#x:%f#y:%f#phi:%f#\n", my_desired_point->cmd_d, &my_desired_point->x_d, &my_desired_point->y_d, &my_desired_point->phi_d);
}

