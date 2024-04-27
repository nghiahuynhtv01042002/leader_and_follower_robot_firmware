/*
 * pid_module.c
 *
 *  Created on: Apr 1, 2024
 *      Author: Nghia
 */
#include "pid_module.h"
//convert unit
float rad_to_degree(float rad){
	float degree = rad *(180/  3.14);
	return degree;
}
float degree_to_rad(float degree){
	float rad = degree *(3.14 /180);
	return rad;
}

float mps_to_rpm(float mps){
	return (float)mps*60/pi*d;
}

float rpm_to_mps(float rpm){
	return (float)rpm*pi*d/60;
}
float rpm_to_pwm(float rpm){
	return (float)rpm*1000/80.167;
}
float mps_to_PWM(float mps){
	return (float) mps*(60/(3.14*d))*(1000/80.167);
}

// initial pid
void pid_set_Kp_Ki_Kd(PID_handleTypedef* hpid,float Kp,float Ki,float Kd, float delta_t){
    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;
    hpid->delta_t = delta_t;
}

float calculate_pid_output(PID_handleTypedef* hpid){
    hpid->error = (hpid->setpoint -hpid->current);//rad
    hpid->integral += hpid->error;
    hpid->derivative = hpid->error - hpid->pre_error;
    float output_pid = (hpid->Kp*hpid->error) + (hpid->Ki*hpid->integral)+ (hpid->Kd*hpid->derivative);
    hpid->pre_error = hpid->error;
//    return degree_to_rad(ouput_pid);//rad/s
    return output_pid;//0.8
}

float cal_speed_left_motor(float pid_output,float v_speed){
	return (float)(2*v_speed - pid_output*L)/(2) ;
}
float cal_speed_right_motor(float pid_output,float v_speed){
	return (float)(2*v_speed + pid_output*L)/(2) ;
}


