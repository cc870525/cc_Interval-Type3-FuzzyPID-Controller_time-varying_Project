#ifndef FUZZY_CONTROLLER
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define FUZZY_CONTROLLER
typedef struct fuzzy_data{
    float error_fuzzy[5];
    float error_fuzzy_dt[5];
}fuzzy_data;
typedef struct kp_output_matrix{
    int kp_output_matrix[5][5];
}kp_output_matrix;
typedef struct kp_output_label{
    float kp_output_label[5];
}kp_output_label;
typedef struct kp_sugeno_data{
    float numerator;
    float denominator;
}kp_sugeno_data;
extern const kp_output_matrix kp_matrix;
extern const kp_output_label kp_label;

typedef struct ki_output_matrix{
    int ki_output_matrix[5][5];
}ki_output_matrix;
typedef struct ki_output_label{
    float ki_output_label[5];
}ki_output_label;
typedef struct ki_sugeno_data{
    float numerator;
    float denominator;
}ki_sugeno_data;
extern const ki_output_matrix ki_matrix;
extern const ki_output_label ki_label;

typedef struct kd_output_matrix{
    int kd_output_matrix[5];
}kd_output_matrix;
typedef struct kd_output_label{
    float kd_output_label[5];
}kd_output_label;
typedef struct kd_sugeno_data{
    float numerator;
    float denominator;
}kd_sugeno_data;
extern const kd_output_matrix kd_matrix;
extern const kd_output_label kd_label;

//theta label
typedef struct kp_output_label_theta{
    float kp_output_label_theta[5];
}kp_output_label_theta;
typedef struct ki_output_label_theta{
    float ki_output_label_theta[5];
}ki_output_label_theta;
typedef struct kd_output_label_theta{
    float kd_output_label_theta[5];
}kd_output_label_theta;
extern const kp_output_label_theta kp_label_theta;
extern const ki_output_label_theta ki_label_theta;
extern const kd_output_label_theta kd_label_theta;

typedef struct type3_data{
    int type3_output_label[4];
    float upper_value_error[4];
    float lower_value_error[4];
    float upper_value_error_dt[4];
    float lower_value_error_dt[4];
}type3_data;
typedef struct type3_zdata{
    float z_lower_alpha_lower_error[4];
    float z_lower_alpha_upper_error[4];
    float z_upper_alpha_upper_error[4];
    float z_upper_alpha_lower_error[4];
    float z_lower_alpha_lower_error_dt[4];
    float z_lower_alpha_upper_error_dt[4];
    float z_upper_alpha_upper_error_dt[4];
    float z_upper_alpha_lower_error_dt[4];
}type3_zdata;
typedef struct type3_fdata{
    float fout_upper[4];
    float fout_lower[4];
}type3_fdata;

//Function Declarations
float trapmf_lower(float x,float a,float b,float c,float d);
float trimf_lower(float x,float a,float b,float c);
float trapmf_upper(float x,float a,float b,float c,float d);
float trimf_upper(float x,float a,float b,float c);


#endif