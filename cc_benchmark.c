/*
 * Original Version Author: [HSU, MING-HSIANG] (CC)
 * Initial Version: 2026
 *
 * Developed for academic research at:
 * National Chung Cheng University
 * Intelligent Robotics and Embedded Systems Laboratory, Factory 203
 *
 * Research use by the above laboratory is permitted.
 * Original-version author attribution must be retained.
 * See README.md for detailed terms of use.
 */
#include "hps_0.h"
#include "PID.h"
/*
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
*/
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
volatile unsigned long *cw_conter_addr_1=NULL;    //read,low_32bit_vaild
volatile unsigned long *ccw_conter_addr_1=NULL;   //read,low_32bit_vaild
volatile unsigned long *pwm_width_addr_1=NULL;    //pwm 20HZ setting duty 1ms = 5%(ccw full) 2ms = 10%(cw full)
volatile unsigned long *cw_conter_addr_2=NULL;    
volatile unsigned long *ccw_conter_addr_2=NULL;
volatile unsigned long *pwm_width_addr_2=NULL;
volatile unsigned long *cw_conter_addr_3=NULL;
volatile unsigned long *ccw_conter_addr_3=NULL;
volatile unsigned long *pwm_width_addr_3=NULL;
volatile unsigned long *cw_conter_addr_4=NULL;
volatile unsigned long *ccw_conter_addr_4=NULL;
volatile unsigned long *pwm_width_addr_4=NULL;   
volatile unsigned long *reset_addr=NULL;   
float scale_factor_1=0.0f;
float scale_factor_2=0.0f;
float scale_factor_3=0.0f;
float scale_factor_4=0.0f;
int pwm_1=0;
int pwm_2=0;
int pwm_3=0;
int pwm_4=0;
char enter = '\n';
float execute_time=0.0f;
static  int64_t time_calculate(struct timespec *time){
    return (int64_t)time->tv_sec * 1000000000LL + (int64_t)time->tv_nsec;
}
/*
static float deegree2radian(float degree){
    return degree * (pi/180.0f);
}
*/
int main() {
    void *virtual;
    int fd;
    if((fd = open("/dev/mem",(O_RDWR | O_SYNC)))==-1){
        printf("open fail");
        return 1;
    }
    virtual = mmap(NULL,HW_REGS_SPAN,(PROT_READ | PROT_WRITE),MAP_SHARED,fd,HW_REGS_BASE);
    if(virtual==MAP_FAILED){
        printf("mapping error");
        close(fd);
        return 1;
    }
    cw_conter_addr_1 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + DIPSW_PIO_CW_BASE) & (unsigned long)(HW_REGS_MASK));   
    ccw_conter_addr_1 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + BUTTON_PIO_CCW_BASE) & (unsigned long)(HW_REGS_MASK));
    pwm_width_addr_1 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + LED_PIO_PWM_BASE) & (unsigned long)(HW_REGS_MASK));
    cw_conter_addr_2 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CW_2_BASE) & (unsigned long)(HW_REGS_MASK));   
    ccw_conter_addr_2 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CCW_2_BASE) & (unsigned long)(HW_REGS_MASK));
    pwm_width_addr_2 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + PWM_2_BASE) & (unsigned long)(HW_REGS_MASK));
    cw_conter_addr_3 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CW_3_BASE) & (unsigned long)(HW_REGS_MASK));   
    ccw_conter_addr_3 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CCW_3_BASE) & (unsigned long)(HW_REGS_MASK));
    pwm_width_addr_3 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + PWM_3_BASE) & (unsigned long)(HW_REGS_MASK));
    cw_conter_addr_4 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CW_4_BASE) & (unsigned long)(HW_REGS_MASK));   
    ccw_conter_addr_4 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + CCW_4_BASE) & (unsigned long)(HW_REGS_MASK));
    pwm_width_addr_4 = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + PWM_4_BASE) & (unsigned long)(HW_REGS_MASK)); 
    reset_addr = virtual + ((unsigned long)(ALT_LWFPGASLVS_OFST + RESET_PIO_BASE) & (unsigned long)(HW_REGS_MASK));                
    struct timespec time1,time2;
    PIDController PID = PIDController_Init();
    PIDController *PID_Pointer = &PID;
    out_matrix OUT = out_matrix_init();
    out_matrix *OUT_Pointer = &OUT;
    P_matrix P = P_matrix_init();
    P_matrix_inverse PI = P_matrix_inverse_init(); 
    velocity_matrix vm = velocity_matrix_init();
    measurement_matrix_dt mmd = measurement_matrix_dt_init();
    measurement_matrix_dt *mmd_pointer = &mmd;
    time_pose tp = time_pose_init();
    time_pose *tp_pointer = &tp;
    read_word_t read_t = read_word;
    P_matrix_update_theata_t pmut = P_matrix_update_theata;
    alt_write_word(reset_addr,0);
    alt_write_word(pwm_width_addr_1,75000);
    alt_write_word(pwm_width_addr_2,75000);
    alt_write_word(pwm_width_addr_3,75000);
    alt_write_word(pwm_width_addr_4,75000);
    alt_write_word(reset_addr,1);
    clock_gettime(CLOCK_MONOTONIC,&time1);
    while(1)
    {   
        printf("================================================\n");
        clock_gettime(CLOCK_MONOTONIC,&time2);
        PID.T = (float)(time_calculate(&time2) - time_calculate(&time1))*1e-9f;
        vm = ReadEncoder_velocity(read_t,vm,PID_Pointer,OUT);
        printf("vm struct value:v1->%.6f v2->%.6f v3->%.6f v4->%.6f\n",vm.velocity_1,vm.velocity_2,vm.velocity_3,vm.velocity_4);      
        PI = P_matrix_inverse_update_theata(PI,P);    
        mmd = measurement_matrix_dt_init();
        mmd = P_inverse_by_velocity(PI,vm,mmd,tp_pointer,PID.T);//call timepose
        //printf("mmd struct value:x->%.6f y->%.6f theta->%.6f\n",mmd.measurement_x,mmd.measurement_y,mmd.measurement_theta);
        OUT = out_matrix_init();     
        PID_TYPE3_FUZZY_Controller_Update(PID_Pointer,mmd_pointer,&P,pmut,OUT_Pointer);//call timepose   
        //printf("position_xadd->%6f position_yadd->%6f position_thetaadd->%6f\n",PID.prevMeasurement_xbuffer,PID.prevMeasurement_ybuffer,PID.prevMeasurement_thetabuffer);
        printf("time_position_x->%.6f time_position_y->%.6f time_position_theta->%.6f\n",tp.setpoint_x,tp.setpoint_y,tp.setpoint_theta);
        printf("position_x->%.6f position_y->%.6f position_theta->%.6f\n",PID.prevMeasurement_x,PID.prevMeasurement_y,PID.prevMeasurement_theta);
        printf("error: error_x->%.6f error_y->%.6f error_theta->%.6f\n",PID.prevError_x,PID.prevError_y,PID.prevError_theta);
        printf("error_dt: error_xdt->%.6f error_ydt->%.6f error_thetadt->%.6f\n",mmd.error_x_dt,mmd.error_y_dt,mmd.error_theta_dt);
        if(check_time_reached(execute_time,&PID)==1){
            alt_write_word(reset_addr,0);
            printf("按下任意鍵繼續...\n");
            scanf(" %c",&enter);            
            free(P.row[0]);
            free(P.row[1]);
            free(P.row[2]);
            free(P.row[3]);
            free(PI.row[0]);
            free(PI.row[1]); 
            free(PI.row[2]);
            printf("complete all work!!\n");   
            break;            
        }          
        execute_time+=PID.T;
        printf("CPU execute time->%3f\n",execute_time);
        alt_write_word(reset_addr,0);
        usleep(100);
        alt_write_word(reset_addr,1);
        clock_gettime(CLOCK_MONOTONIC,&time1);
        out_to_pwm(OUT);
        usleep(10000);
    }
  
    close(fd);
    if(munmap(virtual,HW_REGS_SPAN)!=0){
        printf("munmap error");
        return 1;
    }
    
    

return 0;
}



