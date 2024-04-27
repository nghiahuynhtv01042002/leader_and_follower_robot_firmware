/*
 * pid_module.h
 *
 *  Created on: Apr 1, 2024
 *      Author: Nghia
 */

#ifndef INC_PID_MODULE_H_
#define INC_PID_MODULE_H_

#include "stdlib.h"
#include"stdio.h"
#include"math.h"
#include "Robot_odom.h"
typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float delta_t;

    float setpoint;
    float current;
    float error;
    float pre_error;
    float integral;
    float derivative;

}PID_handleTypedef;

void pid_set_Kp_Ki_Kd(PID_handleTypedef* hpid,float Kp,float Ki,float Kd,float delta_t);
float calculate_pid_output(PID_handleTypedef* hpid);

float mps_to_rpm(float mps);
float rpm_to_mps(float rpm);
float rad_to_degree(float rad);
float degree_to_rad(float degree);
float rpm_to_pwm(float rpm);

float mps_to_PWM(float mps);

float cal_speed_left_motor(float pid_output,float v_speed);
float cal_speed_right_motor(float pid_output,float v_speed);


#endif /* INC_PID_MODULE_H_ */
