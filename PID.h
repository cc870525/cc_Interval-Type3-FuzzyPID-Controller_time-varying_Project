#ifndef PID_CONTROLLER
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "fuzzy.h" 
#define PID_CONTROLLER
#define pi 3.14
#define D 0.1016
#define alpha_cut_1 0.84f
#define alpha_cut_2 0.89f
#define alpha_cut_3 0.94f
#define alpha_cut_4 0.99f
#define type3_delta 2.0f 
#define c_factor 0.5f
extern int pwm_1;
extern int pwm_2;
extern int pwm_3;
extern int pwm_4;
extern float scale_factor_1;
extern float scale_factor_2;
extern float scale_factor_3;
extern float scale_factor_4;
extern volatile unsigned long *pwm_width_addr_1;
extern volatile unsigned long *pwm_width_addr_2;
extern volatile unsigned long *pwm_width_addr_3;
extern volatile unsigned long *pwm_width_addr_4;
extern volatile unsigned long *cw_conter_addr_1;
extern volatile unsigned long *ccw_conter_addr_1;
extern volatile unsigned long *cw_conter_addr_2;
extern volatile unsigned long *ccw_conter_addr_2;
extern volatile unsigned long *cw_conter_addr_3;
extern volatile unsigned long *ccw_conter_addr_3;
extern volatile unsigned long *cw_conter_addr_4;
extern volatile unsigned long *ccw_conter_addr_4;
extern float execute_time;
typedef struct PIDController {	
	float Kp_1;
	float Kp_2;
	float Kp_3;
	float Ki_1;
	float Ki_2;
	float Ki_3;
	float Kd_1;
	float Kd_2;
	float Kd_3;
	float T ;	
	float integrator_x_1;
	float integrator_y_1;
	float integrator_theta_1;
	float integrator_x_2;
	float integrator_y_2;
	float integrator_theta_2;
	float integrator_x_3;
	float integrator_y_3;
	float integrator_theta_3;
	float integrator_x_4;
	float integrator_y_4;
	float integrator_theta_4;
	float differentiator_x;
	float differentiator_y;
	float differentiator_theta;	
	float prevError_x;
	float prevError_y;
	float prevError_theta;
	float prevMeasurement_x;
	float prevMeasurement_y;
	float prevMeasurement_theta;
    float limMin_cw;
    float limMax_cw;
    float limMin_ccw;
    float limMax_ccw;
} PIDController;

typedef struct time_pose {
	float setpoint_x;
	float setpoint_y;
	float setpoint_theta;
}time_pose;

typedef struct P_matrix{
	float row_element_1[3];
	float row_element_2[3];
	float row_element_3[3]; 
	float row_element_4[3]; 
	float *row[4];
	float pretheta;
}P_matrix;

typedef struct P_matrix_inverse{
	float row_element_1[4];
	float row_element_2[4];
	float row_element_3[4]; 	 
	float *row[3];
}P_matrix_inverse;

typedef struct velocity_matrix{
	float velocity_1;
	float velocity_2;
	float velocity_3;
	float velocity_4;
}velocity_matrix;	

typedef struct measurement_matrix_dt{
	float measurement_x;
	float measurement_y;
	float measurement_theta;
	float measurement_time_x;
	float measurement_time_y;
	float measurement_time_theta;
	float error_x_dt;
	float error_y_dt;
	float error_theta_dt;	
}measurement_matrix_dt;

typedef struct out_matrix{
	float out_1;
	float out_2;
	float out_3;
	float out_4;
}out_matrix;

typedef struct setpoint_queue_data{
	float setpoint_x;
	float setpoint_y;
	float setpoint_theta;
}setpoint_queue_data;//Goal

typedef struct setpoint_queue_node{
	setpoint_queue_data data;
	struct setpoint_queue_node* next;
}setpoint_queue_node;//Node

typedef struct setpoint_queue_pointer{
	setpoint_queue_node* front;
	setpoint_queue_node* rear;
	int size;
} setpoint_queue_pointer;//Goal Queue
//Function Pointer Declarations
typedef  int (*read_word_t)(volatile unsigned long *);
typedef setpoint_queue_data (*peek_queue_t)(setpoint_queue_pointer);
typedef void (*P_matrix_update_theata_t)(P_matrix*,float);
//Function Declarations
int read_word(volatile unsigned long*);
setpoint_queue_data peek_queue(setpoint_queue_pointer);
void P_matrix_update_theata(P_matrix*,float);
PIDController PIDController_Init(void);
out_matrix out_matrix_init(void);
velocity_matrix velocity_matrix_init(void);
measurement_matrix_dt measurement_matrix_dt_init(void);
time_pose time_pose_init(void);
void PID_TYPE3_FUZZY_Controller_Update(
	PIDController*, 
    measurement_matrix_dt*,
	P_matrix*,
	P_matrix_update_theata_t,
    out_matrix*);
P_matrix P_matrix_init(void);
P_matrix_inverse P_matrix_inverse_init(void);
P_matrix_inverse P_matrix_inverse_update_theata(
	P_matrix_inverse,
	P_matrix);
velocity_matrix ReadEncoder_velocity(
	read_word_t,
	velocity_matrix,
	PIDController*,
	out_matrix);
measurement_matrix_dt P_inverse_by_velocity(
	P_matrix_inverse,
	velocity_matrix,
	measurement_matrix_dt,
	time_pose*,
	float);
setpoint_queue_pointer queue_init(void);
int queue_empty(setpoint_queue_pointer*);
int push_queue(
    setpoint_queue_pointer*,
    setpoint_queue_data);
int pop_queue(setpoint_queue_pointer*);
void out_to_pwm(out_matrix);
int check_goal_reached(
	peek_queue_t,
	setpoint_queue_pointer,
	PIDController*);
int check_time_reached(
    float,
    PIDController *); 	
static inline float warp(float theta){
    const float TWO_PI = 6.2831853071795864769f;
return remainderf(theta,TWO_PI);
}
static inline time_pose time_pose_call(float exetime){
	time_pose tp = {
		.setpoint_x = 0.0f,
		.setpoint_y = 0.0f,
		.setpoint_theta = 0.0f
	};
	
	//ellipse trajectory
	
	float const radian_vel = 0.05f;// (rad/sec)(架空測:0.05)
	float const r1 = 0.5f; // m
	float const r2 = 0.5f; // m
	tp.setpoint_x = (float)(r1*cosf(radian_vel*exetime));
	tp.setpoint_y = (float)(r2*sinf(radian_vel*exetime));
	tp.setpoint_theta = -2.0071f;//(架空測:-2.0071)	
	
	//flower trajectory
	/*	
	float const radian_vel = 0.025f;// (rad/sec)
	float const R = 10.0f;
	float const n = 4.0f; 
	float phi = radian_vel * exetime;
	float r = R * cosf(n * phi);
	tp.setpoint_x = (float)(r*cosf(radian_vel*exetime));
	tp.setpoint_y = (float)(r*sinf(radian_vel*exetime));
	tp.setpoint_theta = -2.0071f;//(架空測:-2.0071)
	*/
return tp;
}	


#endif