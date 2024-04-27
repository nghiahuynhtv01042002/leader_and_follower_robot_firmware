/*
 * Robot_odom.c
 *
 *  Created on: Mar 31, 2024
 *      Author: Nghia
 */
#include "Robot_odom.h"



/// initial robot parameter

void init_Robot(Robot* robot){
    robot->x = 0;
    robot->y = 0;
    robot->theta = 0;
    robot->omega = 0;
    robot->v = 0.05;//60rpm = 0.2041 50 = 0.17
//    robot->v = 0.0;//60rpm = 0.2041 50 = 0.17

    robot->v_r = 0;
    robot->v_l = 0;
    robot->v_r_rpm = 0;
    robot->v_l_rpm = 0;
    robot->v_r_PWM = 0;
    robot->v_l_PWM = 0;
    robot->S_distance = 0;
    strcpy(robot->cmd,"STP");

}
/// @brief calculat new posstion of Robot from encoder value
/// @param robot
/// @param encoder_Right_Value
/// @param encoder_Left_Value
void update_Position(Robot* robot, int encoder_difference_left, int encoder_difference_right,float delta_t){
		float D_r  = (float) pi*d*(encoder_difference_right)/N;
		float D_l = (float) pi*d*(encoder_difference_left)/N;
		float D_c = (D_r + D_l)/2;

      robot->x += D_c*cos(robot->theta);
      robot->y += D_c*sin(robot->theta);
      robot->theta += (D_r -D_l)/(L);
//      robot->v = 0.1*robot->omega;
//      robot->theta = robot->omega* delta_t;
      robot->theta = atan2(sin(robot->theta),cos(robot->theta)) ;
//      if (robot->phi > 2 * pi) robot->phi -= 2 * pi;
//      else if (robot->phi < 0) robot->phi += 2 * pi;
}
void update_Position_base_velocity(Robot* robot, int encoder_difference_left, int encoder_difference_right,float delta_t){
	float D_r  = (float) pi*d*(encoder_difference_right)/N;
	float D_l = (float) pi*d*(encoder_difference_left)/N;
	robot->v_l = D_l/delta_t;
	robot->v_r = D_r/delta_t;
	robot->v = (robot->v_r + robot->v_l )/2;
	robot->S_distance = (D_r + D_l)/2;
//	robot->x += D_c*cos(robot->phi);
//	robot->y += D_c*sin(robot->phi);
	robot->x += robot->v * cos(robot->theta)*delta_t;
	robot->y +=	robot->v *sin(robot->theta)*delta_t;
//
//	robot->theta += (D_r -D_l)/(L);
//	robot->theta += robot->omega*delta_t;
	robot->theta += (((robot->v_r/(d/2)) - (robot->v_l/(d/2)))*delta_t) / L ;// rad/s *delta_t => rad.
	robot->theta= atan2(sin(robot->theta),cos(robot->theta)) ;
}




/// @brief getter
/// @param robot
/// @return
float get_X(Robot* robot){
    return robot->x;
}
float get_Y(Robot* robot){
    return robot->y;
}
float get_Phi(Robot* robot){
    return robot->theta;
}
float get_Vr(Robot* robot){
    return robot->v_r;
}
float get_Vl(Robot* robot){
    return robot->v_l;
}
float get_V(Robot* robot){
    return robot->v;
}
float get_Omega(Robot* robot){
    return robot->omega;
}

/// @brief setter
/// @param robot


