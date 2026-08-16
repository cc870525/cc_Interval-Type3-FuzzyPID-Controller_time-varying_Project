#include "PID.h"
static bool controller_initloop = false;
int read_word(
    volatile unsigned long * addr)
{
    int buffer = alt_read_word(addr);
    return buffer;
}
setpoint_queue_data peek_queue(setpoint_queue_pointer sqp)
{   
    setpoint_queue_data buffer=sqp.front->data;
    return buffer;   
}

void P_matrix_update_theata(
    P_matrix *P,
    float new_theta)
{
    float delta=0.7853981634f;
    P->pretheta=new_theta;
	for(int i=0;i<2;i++){
		if(i==0){
			*((float*)P->row[0]+i) = -sinf(delta+new_theta);
			*((float*)P->row[1]+i) = -cosf(delta+new_theta);
			*((float*)P->row[2]+i) =  sinf(delta+new_theta);
			*((float*)P->row[3]+i) =  cosf(delta+new_theta);
		}
		else if(i==1){
			*((float*)P->row[0]+i) =  cosf(delta+new_theta);
			*((float*)P->row[1]+i) = -sinf(delta+new_theta);
			*((float*)P->row[2]+i) = -cosf(delta+new_theta);
			*((float*)P->row[3]+i) =  sinf(delta+new_theta);		
		}
		else{
			break;
		}	
	}
return;
}


PIDController PIDController_Init(void)
    
{
    PIDController pid  = {
        /*8葉花瓣取平均
        .Kp_1 = 0.50026555f,
        .Kp_2 = 0.50026555f,
        .Kp_3 = 0.50026555f,
        .Ki_1 = 0.01042145f,
        .Ki_2 = 0.01042145f,
        .Ki_3 = 0.01042145f,
        .Kd_1 = 0.04007557f,
        .Kd_2 = 0.04007557f,
        .Kd_3 = 0.04007557f,
        */
        //橢圓取平均
        
        .Kp_1 = 0.50768476f,
        .Kp_2 = 0.50768476f,
        .Kp_3 = 0.50768476f,
        .Ki_1 = 0.01134231f,
        .Ki_2 = 0.01134231f,
        .Ki_3 = 0.01134231f,
        .Kd_1 = 0.02517510f,
        .Kd_2 = 0.02517510f,
        .Kd_3 = 0.02517510f,
        
        .T = 0.0f,//Ts
        //初始
        /*       
        .Kp_1 = 0.25f,
        .Kp_2 = 0.25f,
        .Kp_3 = 0.25f,       
        .Ki_1 = 0.01f,
        .Ki_2 = 0.01f,
        .Ki_3 = 0.01f,
        .Kd_1 = 0.006f,
        .Kd_2 = 0.006f,
        .Kd_3 = 0.006f, 
        */  
        .integrator_x_1 = 0.0f,
        .integrator_y_1 = 0.0f,
        .integrator_theta_1 = 0.0f,
        .integrator_x_2 = 0.0f,
        .integrator_y_2 = 0.0f,
        .integrator_theta_2 = 0.0f,
        .integrator_x_3 = 0.0f,
        .integrator_y_3 = 0.0f,
        .integrator_theta_3 = 0.0f,
        .integrator_x_4 = 0.0f,
        .integrator_y_4 = 0.0f,
        .integrator_theta_4 = 0.0f,
        .prevError_x = 0.0f,
        .prevError_y = 0.0f,
        .prevError_theta = 0.0f,
        .prevMeasurement_x = 0.0f,//ellipse設0.0,flower設10.0
        .prevMeasurement_y = 0.0f,
        .prevMeasurement_theta = 0.0f,
        .differentiator_x = 0.0f,  
        .differentiator_y = 0.0f,  
        .differentiator_theta = 0.0f,  
        .limMax_cw = 0.77f,
        .limMin_cw = 0.04f,
        .limMax_ccw = -0.77f,
        .limMin_ccw = -0.04f,
    };  
return pid;
}

out_matrix out_matrix_init(void)
{
    out_matrix out = {
        .out_1 = 0.0f,
        .out_2 = 0.0f,
        .out_3 = 0.0f,
        .out_4 = 0.0f
    };
return out;
}

velocity_matrix velocity_matrix_init(void)
{
    velocity_matrix vm = {
        .velocity_1 = 0.0f,
        .velocity_2 = 0.0f, 
        .velocity_3 = 0.0f,
        .velocity_4 = 0.0f
    };

return vm;
}

measurement_matrix_dt measurement_matrix_dt_init(void)
{
    measurement_matrix_dt mmd = {
        .measurement_x = 0.0f,
        .measurement_y = 0.0f,
        .measurement_theta = 0.0f,
        .measurement_time_x = 0.0f,
        .measurement_time_y = 0.0f,
        .measurement_time_theta = 0.0f,
        .error_x_dt = 0.0f,
        .error_y_dt = 0.0f,
        .error_theta_dt = 0.0f
    };

return mmd;
}

time_pose time_pose_init(void)
{
    time_pose tp = {
        .setpoint_x = 0.0f,
        .setpoint_y = 0.0f,
        .setpoint_theta = 0.0f
    };
return tp;    
}

void PID_TYPE3_FUZZY_Controller_Update(
    PIDController *pid, 
    measurement_matrix_dt *premeasurement,
    P_matrix *P,
    P_matrix_update_theata_t pmut,
    out_matrix *out)   
{   
    if(controller_initloop==false){
        controller_initloop=true;
        return;        
    }
    time_pose tp = time_pose_call(execute_time);
    pid->prevMeasurement_x +=premeasurement->measurement_x*pid->T;
    pid->prevMeasurement_y +=premeasurement->measurement_y*pid->T;
    pid->prevMeasurement_theta +=premeasurement->measurement_theta*pid->T;
    pid->prevMeasurement_theta = warp(pid->prevMeasurement_theta);
	pmut(P,pid->prevMeasurement_theta);
    //time_pose xd,yd,thetad
    float desired_x_dt = premeasurement->measurement_time_x;
    float desired_y_dt = premeasurement->measurement_time_y;
    float desired_theta_dt = premeasurement->measurement_time_theta;
    //Error_dt signal	   
    float error_x_dt = premeasurement->error_x_dt;
    float error_y_dt = premeasurement->error_y_dt;
    float error_theta_dt = premeasurement->error_theta_dt;
    //Error signal
    float error_x = tp.setpoint_x  - pid->prevMeasurement_x;
    float error_y = tp.setpoint_y  - pid->prevMeasurement_y;
    float error_theta = warp(tp.setpoint_theta - pid->prevMeasurement_theta);   
    //============================fuzzy type3 logic=====================================
    //e & edot scale [-1,1]
    //error根據x y setpoint調整,theta固定pi
    float scale_error_x = error_x/10.0f;//Flower:1.83 Ellipse:10
    float scale_error_y = error_y/1.94f;//Flower:2.14 Ellipse:1.94
    float scale_error_theta = error_theta/3.14f; //都相同3.14
    //error_dt根據PID實驗調整
    float scale_error_xdt = error_x_dt/1.2f;//Flower:0.41 Ellipse:1.2
    float scale_error_ydt = error_y_dt/0.75f;//Flower:0.43 Ellipse:0.75
    float scale_error_thetadt = error_theta_dt/1.3f;//Flower:1.3 Ellipse:1.3
    float error_dt_fuzzy_upper = 0.0f;
    float error_fuzzy_upper = 0.0f;
    float error_dt_fuzzy_lower = 0.0f;
    float error_fuzzy_lower = 0.0f;
    float numerator_upper= 0.0f;
    float denominator_upper= 0.0f;
    float numerator_lower= 0.0f;
    float denominator_lower= 0.0f;
    float output_numberator = 0.0f;
    float output_denominator = 0.0f;
    int alpha_cut_position = 1;
    float kp_x_increase = 0.0f;
    float ki_x_increase = 0.0f;
    float kd_x_increase = 0.0f;
    float kp_y_increase = 0.0f;
    float ki_y_increase = 0.0f;
    float kd_y_increase = 0.0f;
    float kp_theta_increase = 0.0f;
    float ki_theta_increase = 0.0f;
    float kd_theta_increase = 0.0f;  
    if(scale_error_x>1.0f){scale_error_x=1.0f;}  
    if(scale_error_x<-1.0f){scale_error_x=-1.0f;}
    if(scale_error_y>1.0f){scale_error_y=1.0f;}  
    if(scale_error_y<-1.0f){scale_error_y=-1.0f;}
    if(scale_error_theta>1.0f){scale_error_theta=1.0f;}  
    if(scale_error_theta<-1.0f){scale_error_theta=-1.0f;}
    if(scale_error_xdt>1.0f){scale_error_xdt=1.0f;}
    if(scale_error_xdt<-1.0f){scale_error_xdt=-1.0f;}
    if(scale_error_ydt>1.0f){scale_error_ydt=1.0f;}
    if(scale_error_ydt<-1.0f){scale_error_ydt=-1.0f;}
    if(scale_error_thetadt>1.0f){scale_error_thetadt=1.0f;}
    if(scale_error_thetadt<-1.0f){scale_error_thetadt=-1.0f;}
//X axis
/*花瓣
    fuzzy_data x_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_x, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_upper(scale_error_x, -0.08538110f, -0.03000003f, 0.0f),
            trimf_upper(scale_error_x, -0.03000003f, 0.0f, 0.03000003f),
            trimf_upper(scale_error_x, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_upper(scale_error_x, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_xdt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_upper(scale_error_xdt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_upper(scale_error_xdt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_upper(scale_error_xdt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_upper(scale_error_xdt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };

    fuzzy_data x_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_x, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_lower(scale_error_x, -0.08538110f, -0.03000003f, 0.0f),
            trimf_lower(scale_error_x, -0.03000003f, 0.0f, 0.03000003f),
            trimf_lower(scale_error_x, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_lower(scale_error_x, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_xdt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_lower(scale_error_xdt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_lower(scale_error_xdt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_lower(scale_error_xdt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_lower(scale_error_xdt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };
*/
    //橢圓  
    fuzzy_data x_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_x, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_upper(scale_error_x, -0.08816659f, -0.03026020f, 0.0f),
            trimf_upper(scale_error_x, -0.03026020f, 0.0f, 0.03026020f),
            trimf_upper(scale_error_x, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_upper(scale_error_x, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_xdt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_upper(scale_error_xdt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_upper(scale_error_xdt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_upper(scale_error_xdt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_upper(scale_error_xdt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };

    fuzzy_data x_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_x, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_lower(scale_error_x, -0.08816659f, -0.03026020f, 0.0f),
            trimf_lower(scale_error_x, -0.03026020f, 0.0f, 0.03026020f),
            trimf_lower(scale_error_x, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_lower(scale_error_x, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_xdt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_lower(scale_error_xdt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_lower(scale_error_xdt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_lower(scale_error_xdt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_lower(scale_error_xdt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };

    /*初始    

    fuzzy_data x_fuzzy_upper = {
        .error_fuzzy={
            trapmf_upper(scale_error_x,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_x,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_x,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_x,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_x,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_upper(scale_error_xdt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_xdt,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_xdt,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_xdt,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_xdt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
    fuzzy_data x_fuzzy_lower = {
        .error_fuzzy={
            trapmf_lower(scale_error_x,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_x,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_x,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_x,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_x,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_lower(scale_error_xdt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_xdt,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_xdt,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_xdt,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_xdt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
*/
    //x_kp  
    type3_data kp_x_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kp_x_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kp_x_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int x_kp_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = x_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = x_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = x_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = x_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                x_kp_value_controlpointer++;
                kp_x_data.upper_value_error[x_kp_value_controlpointer] = error_fuzzy_upper;
                kp_x_data.lower_value_error[x_kp_value_controlpointer] = error_fuzzy_lower;
                kp_x_data.upper_value_error_dt[x_kp_value_controlpointer] = error_dt_fuzzy_upper;
                kp_x_data.lower_value_error_dt[x_kp_value_controlpointer] = error_dt_fuzzy_lower;                         
                kp_x_data.type3_output_label[x_kp_value_controlpointer] = kp_matrix.kp_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=x_kp_value_controlpointer;i++){
            kp_x_zdata.z_lower_alpha_lower_error[i] = kp_x_data.lower_value_error[i] + (c_factor * (kp_x_data.upper_value_error[i] - kp_x_data.lower_value_error[i])) * alpha_cut_lower;
            kp_x_zdata.z_lower_alpha_upper_error[i] = kp_x_data.lower_value_error[i] + (c_factor * (kp_x_data.upper_value_error[i] - kp_x_data.lower_value_error[i])) * alpha_cut_upper;
            kp_x_zdata.z_upper_alpha_upper_error[i] = kp_x_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_x_data.upper_value_error[i] - kp_x_data.lower_value_error[i])) * alpha_cut_upper;
            kp_x_zdata.z_upper_alpha_lower_error[i] = kp_x_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_x_data.upper_value_error[i] - kp_x_data.lower_value_error[i])) * alpha_cut_lower;
            kp_x_zdata.z_lower_alpha_lower_error_dt[i] = kp_x_data.lower_value_error_dt[i] + (c_factor * (kp_x_data.upper_value_error_dt[i] - kp_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kp_x_zdata.z_lower_alpha_upper_error_dt[i] = kp_x_data.lower_value_error_dt[i] + (c_factor * (kp_x_data.upper_value_error_dt[i] - kp_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_x_zdata.z_upper_alpha_upper_error_dt[i] = kp_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_x_data.upper_value_error_dt[i] - kp_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_x_zdata.z_upper_alpha_lower_error_dt[i] = kp_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_x_data.upper_value_error_dt[i] - kp_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=x_kp_value_controlpointer;i++){
            numerator_upper += (kp_x_zdata.z_lower_alpha_upper_error[i] * kp_x_zdata.z_lower_alpha_upper_error_dt[i] * kp_label.kp_output_label[kp_x_data.type3_output_label[i]])
            + (kp_x_zdata.z_upper_alpha_upper_error[i] * kp_x_zdata.z_upper_alpha_upper_error_dt[i] * kp_label.kp_output_label[kp_x_data.type3_output_label[i]]);
            denominator_upper += (kp_x_zdata.z_lower_alpha_upper_error[i] * kp_x_zdata.z_lower_alpha_upper_error_dt[i])
            + (kp_x_zdata.z_upper_alpha_upper_error[i] * kp_x_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kp_x_zdata.z_lower_alpha_lower_error[i] * kp_x_zdata.z_lower_alpha_lower_error_dt[i] * kp_label.kp_output_label[kp_x_data.type3_output_label[i]])
            + (kp_x_zdata.z_upper_alpha_lower_error[i] * kp_x_zdata.z_upper_alpha_lower_error_dt[i] * kp_label.kp_output_label[kp_x_data.type3_output_label[i]]);
            denominator_lower += (kp_x_zdata.z_lower_alpha_lower_error[i] * kp_x_zdata.z_lower_alpha_lower_error_dt[i])
            + (kp_x_zdata.z_upper_alpha_lower_error[i] * kp_x_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kp_x_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kp_x_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=x_kp_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kp_x_fdata.fout_upper[i] * alpha_cut_upper )
        + (kp_x_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kp_x_increase = output_numberator/output_denominator;
    printf("kp_x_increase->%.6f\n",kp_x_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //x_ki
    type3_data ki_x_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata ki_x_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata ki_x_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int x_ki_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = x_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = x_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = x_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = x_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                x_ki_value_controlpointer++;
                ki_x_data.upper_value_error[x_ki_value_controlpointer] = error_fuzzy_upper;
                ki_x_data.lower_value_error[x_ki_value_controlpointer] = error_fuzzy_lower;
                ki_x_data.upper_value_error_dt[x_ki_value_controlpointer] = error_dt_fuzzy_upper;
                ki_x_data.lower_value_error_dt[x_ki_value_controlpointer] = error_dt_fuzzy_lower;                         
                ki_x_data.type3_output_label[x_ki_value_controlpointer] = ki_matrix.ki_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=x_ki_value_controlpointer;i++){
            ki_x_zdata.z_lower_alpha_lower_error[i] = ki_x_data.lower_value_error[i] + (c_factor * (ki_x_data.upper_value_error[i] - ki_x_data.lower_value_error[i])) * alpha_cut_lower;
            ki_x_zdata.z_lower_alpha_upper_error[i] = ki_x_data.lower_value_error[i] + (c_factor * (ki_x_data.upper_value_error[i] - ki_x_data.lower_value_error[i])) * alpha_cut_upper;
            ki_x_zdata.z_upper_alpha_upper_error[i] = ki_x_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_x_data.upper_value_error[i] - ki_x_data.lower_value_error[i])) * alpha_cut_upper;
            ki_x_zdata.z_upper_alpha_lower_error[i] = ki_x_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_x_data.upper_value_error[i] - ki_x_data.lower_value_error[i])) * alpha_cut_lower;
            ki_x_zdata.z_lower_alpha_lower_error_dt[i] = ki_x_data.lower_value_error_dt[i] + (c_factor * (ki_x_data.upper_value_error_dt[i] - ki_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
            ki_x_zdata.z_lower_alpha_upper_error_dt[i] = ki_x_data.lower_value_error_dt[i] + (c_factor * (ki_x_data.upper_value_error_dt[i] - ki_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_x_zdata.z_upper_alpha_upper_error_dt[i] = ki_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_x_data.upper_value_error_dt[i] - ki_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_x_zdata.z_upper_alpha_lower_error_dt[i] = ki_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_x_data.upper_value_error_dt[i] - ki_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=x_ki_value_controlpointer;i++){
            numerator_upper += (ki_x_zdata.z_lower_alpha_upper_error[i] * ki_x_zdata.z_lower_alpha_upper_error_dt[i] * ki_label.ki_output_label[ki_x_data.type3_output_label[i]])
            + (ki_x_zdata.z_upper_alpha_upper_error[i] * ki_x_zdata.z_upper_alpha_upper_error_dt[i] * ki_label.ki_output_label[ki_x_data.type3_output_label[i]]);
            denominator_upper += (ki_x_zdata.z_lower_alpha_upper_error[i] * ki_x_zdata.z_lower_alpha_upper_error_dt[i])
            + (ki_x_zdata.z_upper_alpha_upper_error[i] * ki_x_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (ki_x_zdata.z_lower_alpha_lower_error[i] * ki_x_zdata.z_lower_alpha_lower_error_dt[i] * ki_label.ki_output_label[ki_x_data.type3_output_label[i]])
            + (ki_x_zdata.z_upper_alpha_lower_error[i] * ki_x_zdata.z_upper_alpha_lower_error_dt[i] * ki_label.ki_output_label[ki_x_data.type3_output_label[i]]);
            denominator_lower += (ki_x_zdata.z_lower_alpha_lower_error[i] * ki_x_zdata.z_lower_alpha_lower_error_dt[i])
            + (ki_x_zdata.z_upper_alpha_lower_error[i] * ki_x_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        ki_x_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        ki_x_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=x_ki_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (ki_x_fdata.fout_upper[i] * alpha_cut_upper )
        + (ki_x_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    ki_x_increase = output_numberator/output_denominator;
    printf("ki_x_increase->%.6f\n",ki_x_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //x_kd
    type3_data kd_x_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kd_x_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kd_x_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int x_kd_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = x_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = x_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = x_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = x_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                x_kd_value_controlpointer++;
                kd_x_data.upper_value_error[x_kd_value_controlpointer] = error_fuzzy_upper;
                kd_x_data.lower_value_error[x_kd_value_controlpointer] = error_fuzzy_lower;
                kd_x_data.upper_value_error_dt[x_kd_value_controlpointer] = error_dt_fuzzy_upper;
                kd_x_data.lower_value_error_dt[x_kd_value_controlpointer] = error_dt_fuzzy_lower;                         
                kd_x_data.type3_output_label[x_kd_value_controlpointer] = kd_matrix.kd_output_matrix[j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=x_kd_value_controlpointer;i++){
            kd_x_zdata.z_lower_alpha_lower_error[i] = kd_x_data.lower_value_error[i] + (c_factor * (kd_x_data.upper_value_error[i] - kd_x_data.lower_value_error[i])) * alpha_cut_lower;
            kd_x_zdata.z_lower_alpha_upper_error[i] = kd_x_data.lower_value_error[i] + (c_factor * (kd_x_data.upper_value_error[i] - kd_x_data.lower_value_error[i])) * alpha_cut_upper;
            kd_x_zdata.z_upper_alpha_upper_error[i] = kd_x_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_x_data.upper_value_error[i] - kd_x_data.lower_value_error[i])) * alpha_cut_upper;
            kd_x_zdata.z_upper_alpha_lower_error[i] = kd_x_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_x_data.upper_value_error[i] - kd_x_data.lower_value_error[i])) * alpha_cut_lower;
            kd_x_zdata.z_lower_alpha_lower_error_dt[i] = kd_x_data.lower_value_error_dt[i] + (c_factor * (kd_x_data.upper_value_error_dt[i] - kd_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kd_x_zdata.z_lower_alpha_upper_error_dt[i] = kd_x_data.lower_value_error_dt[i] + (c_factor * (kd_x_data.upper_value_error_dt[i] - kd_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_x_zdata.z_upper_alpha_upper_error_dt[i] = kd_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_x_data.upper_value_error_dt[i] - kd_x_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_x_zdata.z_upper_alpha_lower_error_dt[i] = kd_x_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_x_data.upper_value_error_dt[i] - kd_x_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=x_kd_value_controlpointer;i++){
            numerator_upper += (kd_x_zdata.z_lower_alpha_upper_error[i] * kd_x_zdata.z_lower_alpha_upper_error_dt[i] * kd_label.kd_output_label[kd_x_data.type3_output_label[i]])
            + (kd_x_zdata.z_upper_alpha_upper_error[i] * kd_x_zdata.z_upper_alpha_upper_error_dt[i] * kd_label.kd_output_label[kd_x_data.type3_output_label[i]]);
            denominator_upper += (kd_x_zdata.z_lower_alpha_upper_error[i] * kd_x_zdata.z_lower_alpha_upper_error_dt[i])
            + (kd_x_zdata.z_upper_alpha_upper_error[i] * kd_x_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kd_x_zdata.z_lower_alpha_lower_error[i] * kd_x_zdata.z_lower_alpha_lower_error_dt[i] * kd_label.kd_output_label[kd_x_data.type3_output_label[i]])
            + (kd_x_zdata.z_upper_alpha_lower_error[i] * kd_x_zdata.z_upper_alpha_lower_error_dt[i] * kd_label.kd_output_label[kd_x_data.type3_output_label[i]]);
            denominator_lower += (kd_x_zdata.z_lower_alpha_lower_error[i] * kd_x_zdata.z_lower_alpha_lower_error_dt[i])
            + (kd_x_zdata.z_upper_alpha_lower_error[i] * kd_x_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kd_x_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kd_x_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=x_kd_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kd_x_fdata.fout_upper[i] * alpha_cut_upper )
        + (kd_x_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kd_x_increase = output_numberator/output_denominator;
    printf("kd_x_increase->%.6f\n",kd_x_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
//Y axis
    /*花瓣
    fuzzy_data y_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_y, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_upper(scale_error_y, -0.08538110f, -0.03000003f, 0.0f),
            trimf_upper(scale_error_y, -0.03000003f, 0.0f, 0.03000003f),
            trimf_upper(scale_error_y, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_upper(scale_error_y, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_ydt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_upper(scale_error_ydt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_upper(scale_error_ydt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_upper(scale_error_ydt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_upper(scale_error_ydt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };

    fuzzy_data y_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_y, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_lower(scale_error_y, -0.08538110f, -0.03000003f, 0.0f),
            trimf_lower(scale_error_y, -0.03000003f, 0.0f, 0.03000003f),
            trimf_lower(scale_error_y, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_lower(scale_error_y, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_ydt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_lower(scale_error_ydt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_lower(scale_error_ydt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_lower(scale_error_ydt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_lower(scale_error_ydt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };
*/    
//橢圓
    fuzzy_data y_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_y, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_upper(scale_error_y, -0.08816659f, -0.03026020f, 0.0f),
            trimf_upper(scale_error_y, -0.03026020f, 0.0f, 0.03026020f),
            trimf_upper(scale_error_y, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_upper(scale_error_y, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_ydt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_upper(scale_error_ydt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_upper(scale_error_ydt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_upper(scale_error_ydt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_upper(scale_error_ydt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };

    fuzzy_data y_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_y, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_lower(scale_error_y, -0.08816659f, -0.03026020f, 0.0f),
            trimf_lower(scale_error_y, -0.03026020f, 0.0f, 0.03026020f),
            trimf_lower(scale_error_y, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_lower(scale_error_y, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_ydt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_lower(scale_error_ydt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_lower(scale_error_ydt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_lower(scale_error_ydt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_lower(scale_error_ydt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };

    //初始    
/*
    fuzzy_data y_fuzzy_upper = {
        .error_fuzzy={
            trapmf_upper(scale_error_y,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_y,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_y,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_y,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_y,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_upper(scale_error_ydt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_ydt,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_ydt,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_ydt,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_ydt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
    fuzzy_data y_fuzzy_lower = {
        .error_fuzzy={
            trapmf_lower(scale_error_y,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_y,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_y,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_y,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_y,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_lower(scale_error_ydt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_ydt,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_ydt,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_ydt,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_ydt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
*/
    //y_kp
    type3_data kp_y_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kp_y_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kp_y_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int y_kp_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = y_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = y_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = y_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = y_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                y_kp_value_controlpointer++;
                kp_y_data.upper_value_error[y_kp_value_controlpointer] = error_fuzzy_upper;
                kp_y_data.lower_value_error[y_kp_value_controlpointer] = error_fuzzy_lower;
                kp_y_data.upper_value_error_dt[y_kp_value_controlpointer] = error_dt_fuzzy_upper;
                kp_y_data.lower_value_error_dt[y_kp_value_controlpointer] = error_dt_fuzzy_lower;                         
                kp_y_data.type3_output_label[y_kp_value_controlpointer] = kp_matrix.kp_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=y_kp_value_controlpointer;i++){
            kp_y_zdata.z_lower_alpha_lower_error[i] = kp_y_data.lower_value_error[i] + (c_factor * (kp_y_data.upper_value_error[i] - kp_y_data.lower_value_error[i])) * alpha_cut_lower;
            kp_y_zdata.z_lower_alpha_upper_error[i] = kp_y_data.lower_value_error[i] + (c_factor * (kp_y_data.upper_value_error[i] - kp_y_data.lower_value_error[i])) * alpha_cut_upper;
            kp_y_zdata.z_upper_alpha_upper_error[i] = kp_y_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_y_data.upper_value_error[i] - kp_y_data.lower_value_error[i])) * alpha_cut_upper;
            kp_y_zdata.z_upper_alpha_lower_error[i] = kp_y_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_y_data.upper_value_error[i] - kp_y_data.lower_value_error[i])) * alpha_cut_lower;
            kp_y_zdata.z_lower_alpha_lower_error_dt[i] = kp_y_data.lower_value_error_dt[i] + (c_factor * (kp_y_data.upper_value_error_dt[i] - kp_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kp_y_zdata.z_lower_alpha_upper_error_dt[i] = kp_y_data.lower_value_error_dt[i] + (c_factor * (kp_y_data.upper_value_error_dt[i] - kp_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_y_zdata.z_upper_alpha_upper_error_dt[i] = kp_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_y_data.upper_value_error_dt[i] - kp_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_y_zdata.z_upper_alpha_lower_error_dt[i] = kp_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_y_data.upper_value_error_dt[i] - kp_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=y_kp_value_controlpointer;i++){
            numerator_upper += (kp_y_zdata.z_lower_alpha_upper_error[i] * kp_y_zdata.z_lower_alpha_upper_error_dt[i] * kp_label.kp_output_label[kp_y_data.type3_output_label[i]])
            + (kp_y_zdata.z_upper_alpha_upper_error[i] * kp_y_zdata.z_upper_alpha_upper_error_dt[i] * kp_label.kp_output_label[kp_y_data.type3_output_label[i]]);
            denominator_upper += (kp_y_zdata.z_lower_alpha_upper_error[i] * kp_y_zdata.z_lower_alpha_upper_error_dt[i])
            + (kp_y_zdata.z_upper_alpha_upper_error[i] * kp_y_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kp_y_zdata.z_lower_alpha_lower_error[i] * kp_y_zdata.z_lower_alpha_lower_error_dt[i] * kp_label.kp_output_label[kp_y_data.type3_output_label[i]])
            + (kp_y_zdata.z_upper_alpha_lower_error[i] * kp_y_zdata.z_upper_alpha_lower_error_dt[i] * kp_label.kp_output_label[kp_y_data.type3_output_label[i]]);
            denominator_lower += (kp_y_zdata.z_lower_alpha_lower_error[i] * kp_y_zdata.z_lower_alpha_lower_error_dt[i])
            + (kp_y_zdata.z_upper_alpha_lower_error[i] * kp_y_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kp_y_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kp_y_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=y_kp_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kp_y_fdata.fout_upper[i] * alpha_cut_upper )
        + (kp_y_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kp_y_increase = output_numberator/output_denominator;
    printf("kp_y_increase->%.6f\n",kp_y_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //y_ki
    type3_data ki_y_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata ki_y_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata ki_y_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int y_ki_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = y_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = y_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = y_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = y_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                y_ki_value_controlpointer++;
                ki_y_data.upper_value_error[y_ki_value_controlpointer] = error_fuzzy_upper;
                ki_y_data.lower_value_error[y_ki_value_controlpointer] = error_fuzzy_lower;
                ki_y_data.upper_value_error_dt[y_ki_value_controlpointer] = error_dt_fuzzy_upper;
                ki_y_data.lower_value_error_dt[y_ki_value_controlpointer] = error_dt_fuzzy_lower;                         
                ki_y_data.type3_output_label[y_ki_value_controlpointer] = kp_matrix.kp_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=y_ki_value_controlpointer;i++){
            ki_y_zdata.z_lower_alpha_lower_error[i] = ki_y_data.lower_value_error[i] + (c_factor * (ki_y_data.upper_value_error[i] - ki_y_data.lower_value_error[i])) * alpha_cut_lower;
            ki_y_zdata.z_lower_alpha_upper_error[i] = ki_y_data.lower_value_error[i] + (c_factor * (ki_y_data.upper_value_error[i] - ki_y_data.lower_value_error[i])) * alpha_cut_upper;
            ki_y_zdata.z_upper_alpha_upper_error[i] = ki_y_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_y_data.upper_value_error[i] - ki_y_data.lower_value_error[i])) * alpha_cut_upper;
            ki_y_zdata.z_upper_alpha_lower_error[i] = ki_y_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_y_data.upper_value_error[i] - ki_y_data.lower_value_error[i])) * alpha_cut_lower;
            ki_y_zdata.z_lower_alpha_lower_error_dt[i] = ki_y_data.lower_value_error_dt[i] + (c_factor * (ki_y_data.upper_value_error_dt[i] - ki_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
            ki_y_zdata.z_lower_alpha_upper_error_dt[i] = ki_y_data.lower_value_error_dt[i] + (c_factor * (ki_y_data.upper_value_error_dt[i] - ki_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_y_zdata.z_upper_alpha_upper_error_dt[i] = ki_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_y_data.upper_value_error_dt[i] - ki_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_y_zdata.z_upper_alpha_lower_error_dt[i] = ki_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_y_data.upper_value_error_dt[i] - ki_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=y_ki_value_controlpointer;i++){
            numerator_upper += (ki_y_zdata.z_lower_alpha_upper_error[i] * ki_y_zdata.z_lower_alpha_upper_error_dt[i] * ki_label.ki_output_label[ki_y_data.type3_output_label[i]])
            + (ki_y_zdata.z_upper_alpha_upper_error[i] * ki_y_zdata.z_upper_alpha_upper_error_dt[i] * ki_label.ki_output_label[ki_y_data.type3_output_label[i]]);
            denominator_upper += (ki_y_zdata.z_lower_alpha_upper_error[i] * ki_y_zdata.z_lower_alpha_upper_error_dt[i])
            + (ki_y_zdata.z_upper_alpha_upper_error[i] * ki_y_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (ki_y_zdata.z_lower_alpha_lower_error[i] * ki_y_zdata.z_lower_alpha_lower_error_dt[i] * ki_label.ki_output_label[ki_y_data.type3_output_label[i]])
            + (ki_y_zdata.z_upper_alpha_lower_error[i] * ki_y_zdata.z_upper_alpha_lower_error_dt[i] * ki_label.ki_output_label[ki_y_data.type3_output_label[i]]);
            denominator_lower += (ki_y_zdata.z_lower_alpha_lower_error[i] * ki_y_zdata.z_lower_alpha_lower_error_dt[i])
            + (ki_y_zdata.z_upper_alpha_lower_error[i] * ki_y_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        ki_y_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        ki_y_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }

    for(int i=0;i<=y_ki_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (ki_y_fdata.fout_upper[i] * alpha_cut_upper )
        + (ki_y_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    ki_y_increase = output_numberator/output_denominator;
    printf("ki_y_increase->%.6f\n",ki_y_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //y_kd
    type3_data kd_y_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kd_y_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kd_y_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int y_kd_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = y_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = y_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = y_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = y_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                y_kd_value_controlpointer++;
                kd_y_data.upper_value_error[y_kd_value_controlpointer] = error_fuzzy_upper;
                kd_y_data.lower_value_error[y_kd_value_controlpointer] = error_fuzzy_lower;
                kd_y_data.upper_value_error_dt[y_kd_value_controlpointer] = error_dt_fuzzy_upper;
                kd_y_data.lower_value_error_dt[y_kd_value_controlpointer] = error_dt_fuzzy_lower;                         
                kd_y_data.type3_output_label[y_kd_value_controlpointer] = kd_matrix.kd_output_matrix[j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=y_kd_value_controlpointer;i++){
            kd_y_zdata.z_lower_alpha_lower_error[i] = kd_y_data.lower_value_error[i] + (c_factor * (kd_y_data.upper_value_error[i] - kd_y_data.lower_value_error[i])) * alpha_cut_lower;
            kd_y_zdata.z_lower_alpha_upper_error[i] = kd_y_data.lower_value_error[i] + (c_factor * (kd_y_data.upper_value_error[i] - kd_y_data.lower_value_error[i])) * alpha_cut_upper;
            kd_y_zdata.z_upper_alpha_upper_error[i] = kd_y_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_y_data.upper_value_error[i] - kd_y_data.lower_value_error[i])) * alpha_cut_upper;
            kd_y_zdata.z_upper_alpha_lower_error[i] = kd_y_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_y_data.upper_value_error[i] - kd_y_data.lower_value_error[i])) * alpha_cut_lower;
            kd_y_zdata.z_lower_alpha_lower_error_dt[i] = kd_y_data.lower_value_error_dt[i] + (c_factor * (kd_y_data.upper_value_error_dt[i] - kd_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kd_y_zdata.z_lower_alpha_upper_error_dt[i] = kd_y_data.lower_value_error_dt[i] + (c_factor * (kd_y_data.upper_value_error_dt[i] - kd_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_y_zdata.z_upper_alpha_upper_error_dt[i] = kd_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_y_data.upper_value_error_dt[i] - kd_y_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_y_zdata.z_upper_alpha_lower_error_dt[i] = kd_y_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_y_data.upper_value_error_dt[i] - kd_y_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=y_kd_value_controlpointer;i++){
            numerator_upper += (kd_y_zdata.z_lower_alpha_upper_error[i] * kd_y_zdata.z_lower_alpha_upper_error_dt[i] * kd_label.kd_output_label[kd_y_data.type3_output_label[i]])
            + (kd_y_zdata.z_upper_alpha_upper_error[i] * kd_y_zdata.z_upper_alpha_upper_error_dt[i] * kd_label.kd_output_label[kd_y_data.type3_output_label[i]]);
            denominator_upper += (kd_y_zdata.z_lower_alpha_upper_error[i] * kd_y_zdata.z_lower_alpha_upper_error_dt[i])
            + (kd_y_zdata.z_upper_alpha_upper_error[i] * kd_y_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kd_y_zdata.z_lower_alpha_lower_error[i] * kd_y_zdata.z_lower_alpha_lower_error_dt[i] * kd_label.kd_output_label[kd_y_data.type3_output_label[i]])
            + (kd_y_zdata.z_upper_alpha_lower_error[i] * kd_y_zdata.z_upper_alpha_lower_error_dt[i] * kd_label.kd_output_label[kd_y_data.type3_output_label[i]]);
            denominator_lower += (kd_y_zdata.z_lower_alpha_lower_error[i] * kd_y_zdata.z_lower_alpha_lower_error_dt[i])
            + (kd_y_zdata.z_upper_alpha_lower_error[i] * kd_y_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kd_y_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kd_y_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=y_kd_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kd_y_fdata.fout_upper[i] * alpha_cut_upper )
        + (kd_y_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kd_y_increase = output_numberator/output_denominator;
    printf("kd_y_increase->%.6f\n",kd_y_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
//theta axis
    /*花瓣
    fuzzy_data theta_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_theta, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_upper(scale_error_theta, -0.08538110f, -0.03000003f, 0.0f),
            trimf_upper(scale_error_theta, -0.03000003f, 0.0f, 0.03000003f),
            trimf_upper(scale_error_theta, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_upper(scale_error_theta, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_thetadt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_upper(scale_error_thetadt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_upper(scale_error_thetadt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_upper(scale_error_thetadt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_upper(scale_error_thetadt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };

    fuzzy_data theta_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_theta, -1.0f, -1.0f, -0.08538110f, -0.03000003f),
            trimf_lower(scale_error_theta, -0.08538110f, -0.03000003f, 0.0f),
            trimf_lower(scale_error_theta, -0.03000003f, 0.0f, 0.03000003f),
            trimf_lower(scale_error_theta, 0.0f, 0.03000003f, 0.08538110f),
            trapmf_lower(scale_error_theta, 0.03000003f, 0.08538110f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_thetadt, -1.0f, -1.0f, -0.08778865f, -0.03003870f),
            trimf_lower(scale_error_thetadt, -0.08778865f, -0.03003870f, 0.0f),
            trimf_lower(scale_error_thetadt, -0.03003870f, 0.0f, 0.03003870f),
            trimf_lower(scale_error_thetadt, 0.0f, 0.03003870f, 0.08778865f),
            trapmf_lower(scale_error_thetadt, 0.03003870f, 0.08778865f, 1.0f, 1.0f)
        }
    };
*/    
//橢圓
    fuzzy_data theta_fuzzy_upper = {
        .error_fuzzy = {
            trapmf_upper(scale_error_theta, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_upper(scale_error_theta, -0.08816659f, -0.03026020f, 0.0f),
            trimf_upper(scale_error_theta, -0.03026020f, 0.0f, 0.03026020f),
            trimf_upper(scale_error_theta, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_upper(scale_error_theta, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_upper(scale_error_thetadt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_upper(scale_error_thetadt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_upper(scale_error_thetadt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_upper(scale_error_thetadt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_upper(scale_error_thetadt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };

    fuzzy_data theta_fuzzy_lower = {
        .error_fuzzy = {
            trapmf_lower(scale_error_theta, -1.0f, -1.0f, -0.08816659f, -0.03026020f),
            trimf_lower(scale_error_theta, -0.08816659f, -0.03026020f, 0.0f),
            trimf_lower(scale_error_theta, -0.03026020f, 0.0f, 0.03026020f),
            trimf_lower(scale_error_theta, 0.0f, 0.03026020f, 0.08816659f),
            trapmf_lower(scale_error_theta, 0.03026020f, 0.08816659f, 1.0f, 1.0f)
        },
        .error_fuzzy_dt = {
            trapmf_lower(scale_error_thetadt, -1.0f, -1.0f, -0.16205138f, -0.03030600f),
            trimf_lower(scale_error_thetadt, -0.16205138f, -0.03030600f, 0.0f),
            trimf_lower(scale_error_thetadt, -0.03030600f, 0.0f, 0.03030600f),
            trimf_lower(scale_error_thetadt, 0.0f, 0.03030600f, 0.16205138f),
            trapmf_lower(scale_error_thetadt, 0.03030600f, 0.16205138f, 1.0f, 1.0f)
        }
    };
 
    /*初始

    fuzzy_data theta_fuzzy_upper = {
        .error_fuzzy={
            trapmf_upper(scale_error_theta,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_theta,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_theta,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_theta,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_theta,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_upper(scale_error_thetadt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_upper(scale_error_thetadt,-0.5f,-0.2f,0.0f),
            trimf_upper(scale_error_thetadt,-0.2f,0.0f,0.2f),
            trimf_upper(scale_error_thetadt,0.0f,0.2f,0.5f),
            trapmf_upper(scale_error_thetadt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
    fuzzy_data theta_fuzzy_lower = {
        .error_fuzzy={
            trapmf_lower(scale_error_theta,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_theta,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_theta,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_theta,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_theta,0.2f,0.5f,1.0f,1.0f)
        },
        .error_fuzzy_dt={
            trapmf_lower(scale_error_thetadt,-1.0f,-1.0f,-0.5f,-0.2f),
            trimf_lower(scale_error_thetadt,-0.5f,-0.2f,0.0f),
            trimf_lower(scale_error_thetadt,-0.2f,0.0f,0.2f),
            trimf_lower(scale_error_thetadt,0.0f,0.2f,0.5f),
            trapmf_lower(scale_error_thetadt,0.2f,0.5f,1.0f,1.0f)
        }             
    };
*/
    //theta_kp
    type3_data kp_theta_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kp_theta_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kp_theta_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int theta_kp_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = theta_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = theta_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = theta_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = theta_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                theta_kp_value_controlpointer++;
                kp_theta_data.upper_value_error[theta_kp_value_controlpointer] = error_fuzzy_upper;
                kp_theta_data.lower_value_error[theta_kp_value_controlpointer] = error_fuzzy_lower;
                kp_theta_data.upper_value_error_dt[theta_kp_value_controlpointer] = error_dt_fuzzy_upper;
                kp_theta_data.lower_value_error_dt[theta_kp_value_controlpointer] = error_dt_fuzzy_lower;                         
                kp_theta_data.type3_output_label[theta_kp_value_controlpointer] = kp_matrix.kp_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=theta_kp_value_controlpointer;i++){
            kp_theta_zdata.z_lower_alpha_lower_error[i] = kp_theta_data.lower_value_error[i] + (c_factor * (kp_theta_data.upper_value_error[i] - kp_theta_data.lower_value_error[i])) * alpha_cut_lower;
            kp_theta_zdata.z_lower_alpha_upper_error[i] = kp_theta_data.lower_value_error[i] + (c_factor * (kp_theta_data.upper_value_error[i] - kp_theta_data.lower_value_error[i])) * alpha_cut_upper;
            kp_theta_zdata.z_upper_alpha_upper_error[i] = kp_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_theta_data.upper_value_error[i] - kp_theta_data.lower_value_error[i])) * alpha_cut_upper;
            kp_theta_zdata.z_upper_alpha_lower_error[i] = kp_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (kp_theta_data.upper_value_error[i] - kp_theta_data.lower_value_error[i])) * alpha_cut_lower;
            kp_theta_zdata.z_lower_alpha_lower_error_dt[i] = kp_theta_data.lower_value_error_dt[i] + (c_factor * (kp_theta_data.upper_value_error_dt[i] - kp_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kp_theta_zdata.z_lower_alpha_upper_error_dt[i] = kp_theta_data.lower_value_error_dt[i] + (c_factor * (kp_theta_data.upper_value_error_dt[i] - kp_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_theta_zdata.z_upper_alpha_upper_error_dt[i] = kp_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_theta_data.upper_value_error_dt[i] - kp_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kp_theta_zdata.z_upper_alpha_lower_error_dt[i] = kp_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kp_theta_data.upper_value_error_dt[i] - kp_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=theta_kp_value_controlpointer;i++){
            numerator_upper += (kp_theta_zdata.z_lower_alpha_upper_error[i] * kp_theta_zdata.z_lower_alpha_upper_error_dt[i] * kp_label_theta.kp_output_label_theta[kp_theta_data.type3_output_label[i]])
            + (kp_theta_zdata.z_upper_alpha_upper_error[i] * kp_theta_zdata.z_upper_alpha_upper_error_dt[i] * kp_label_theta.kp_output_label_theta[kp_theta_data.type3_output_label[i]]);
            denominator_upper += (kp_theta_zdata.z_lower_alpha_upper_error[i] * kp_theta_zdata.z_lower_alpha_upper_error_dt[i])
            + (kp_theta_zdata.z_upper_alpha_upper_error[i] * kp_theta_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kp_theta_zdata.z_lower_alpha_lower_error[i] * kp_theta_zdata.z_lower_alpha_lower_error_dt[i] * kp_label_theta.kp_output_label_theta[kp_theta_data.type3_output_label[i]])
            + (kp_theta_zdata.z_upper_alpha_lower_error[i] * kp_theta_zdata.z_upper_alpha_lower_error_dt[i] * kp_label_theta.kp_output_label_theta[kp_theta_data.type3_output_label[i]]);
            denominator_lower += (kp_theta_zdata.z_lower_alpha_lower_error[i] * kp_theta_zdata.z_lower_alpha_lower_error_dt[i])
            + (kp_theta_zdata.z_upper_alpha_lower_error[i] * kp_theta_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kp_theta_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kp_theta_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=theta_kp_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kp_theta_fdata.fout_upper[i] * alpha_cut_upper )
        + (kp_theta_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kp_theta_increase = output_numberator/output_denominator;
    printf("kp_theta_increase->%.6f\n",kp_theta_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //theta_ki
    type3_data ki_theta_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata ki_theta_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata ki_theta_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int theta_ki_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = theta_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = theta_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = theta_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = theta_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                theta_ki_value_controlpointer++;
                ki_theta_data.upper_value_error[theta_ki_value_controlpointer] = error_fuzzy_upper;
                ki_theta_data.lower_value_error[theta_ki_value_controlpointer] = error_fuzzy_lower;
                ki_theta_data.upper_value_error_dt[theta_ki_value_controlpointer] = error_dt_fuzzy_upper;
                ki_theta_data.lower_value_error_dt[theta_ki_value_controlpointer] = error_dt_fuzzy_lower;                         
                ki_theta_data.type3_output_label[theta_ki_value_controlpointer] = ki_matrix.ki_output_matrix[i][j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=theta_ki_value_controlpointer;i++){
            ki_theta_zdata.z_lower_alpha_lower_error[i] = ki_theta_data.lower_value_error[i] + (c_factor * (ki_theta_data.upper_value_error[i] - ki_theta_data.lower_value_error[i])) * alpha_cut_lower;
            ki_theta_zdata.z_lower_alpha_upper_error[i] = ki_theta_data.lower_value_error[i] + (c_factor * (ki_theta_data.upper_value_error[i] - ki_theta_data.lower_value_error[i])) * alpha_cut_upper;
            ki_theta_zdata.z_upper_alpha_upper_error[i] = ki_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_theta_data.upper_value_error[i] - ki_theta_data.lower_value_error[i])) * alpha_cut_upper;
            ki_theta_zdata.z_upper_alpha_lower_error[i] = ki_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (ki_theta_data.upper_value_error[i] - ki_theta_data.lower_value_error[i])) * alpha_cut_lower;
            ki_theta_zdata.z_lower_alpha_lower_error_dt[i] = ki_theta_data.lower_value_error_dt[i] + (c_factor * (ki_theta_data.upper_value_error_dt[i] - ki_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
            ki_theta_zdata.z_lower_alpha_upper_error_dt[i] = ki_theta_data.lower_value_error_dt[i] + (c_factor * (ki_theta_data.upper_value_error_dt[i] - ki_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_theta_zdata.z_upper_alpha_upper_error_dt[i] = ki_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_theta_data.upper_value_error_dt[i] - ki_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            ki_theta_zdata.z_upper_alpha_lower_error_dt[i] = ki_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (ki_theta_data.upper_value_error_dt[i] - ki_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=theta_ki_value_controlpointer;i++){
            numerator_upper += (ki_theta_zdata.z_lower_alpha_upper_error[i] * ki_theta_zdata.z_lower_alpha_upper_error_dt[i] * ki_label_theta.ki_output_label_theta[ki_theta_data.type3_output_label[i]])
            + (ki_theta_zdata.z_upper_alpha_upper_error[i] * ki_theta_zdata.z_upper_alpha_upper_error_dt[i] * ki_label_theta.ki_output_label_theta[ki_theta_data.type3_output_label[i]]);
            denominator_upper += (ki_theta_zdata.z_lower_alpha_upper_error[i] * ki_theta_zdata.z_lower_alpha_upper_error_dt[i])
            + (ki_theta_zdata.z_upper_alpha_upper_error[i] * ki_theta_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (ki_theta_zdata.z_lower_alpha_lower_error[i] * ki_theta_zdata.z_lower_alpha_lower_error_dt[i] * ki_label_theta.ki_output_label_theta[ki_theta_data.type3_output_label[i]])
            + (ki_theta_zdata.z_upper_alpha_lower_error[i] * ki_theta_zdata.z_upper_alpha_lower_error_dt[i] * ki_label_theta.ki_output_label_theta[ki_theta_data.type3_output_label[i]]);
            denominator_lower += (ki_theta_zdata.z_lower_alpha_lower_error[i] * ki_theta_zdata.z_lower_alpha_lower_error_dt[i])
            + (ki_theta_zdata.z_upper_alpha_lower_error[i] * ki_theta_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        ki_theta_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        ki_theta_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=theta_ki_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (ki_theta_fdata.fout_upper[i] * alpha_cut_upper )
        + (ki_theta_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    ki_theta_increase = output_numberator/output_denominator;
    printf("ki_theta_increase->%.6f\n",ki_theta_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    //theta_kd
    type3_data kd_theta_data = {
        .type3_output_label={0,0,0,0},
        .upper_value_error={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error={0.0f,0.0f,0.0f,0.0f},
        .upper_value_error_dt={0.0f,0.0f,0.0f,0.0f},
        .lower_value_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_zdata kd_theta_zdata = {
        .z_lower_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_lower_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_upper_error_dt={0.0f,0.0f,0.0f,0.0f},
        .z_upper_alpha_lower_error_dt={0.0f,0.0f,0.0f,0.0f}
    };
    type3_fdata kd_theta_fdata = {
        .fout_upper={0.0f,0.0f,0.0f,0.0f},
        .fout_lower={0.0f,0.0f,0.0f,0.0f}
     };
    int theta_kd_value_controlpointer = -1;
    for(int i=0;i<5;i++){
        error_fuzzy_upper = theta_fuzzy_upper.error_fuzzy[i];
        error_fuzzy_lower = theta_fuzzy_lower.error_fuzzy[i];
        if(error_fuzzy_upper==0.0f || error_fuzzy_lower==0.0f){continue;}
        for(int j=0;j<5;j++){
            error_dt_fuzzy_upper = theta_fuzzy_upper.error_fuzzy_dt[j];
            error_dt_fuzzy_lower = theta_fuzzy_lower.error_fuzzy_dt[j];
            if(error_dt_fuzzy_upper==0.0f || error_dt_fuzzy_lower==0.0f){continue;}
            else{
                theta_kd_value_controlpointer++;
                kd_theta_data.upper_value_error[theta_kd_value_controlpointer] = error_fuzzy_upper;
                kd_theta_data.lower_value_error[theta_kd_value_controlpointer] = error_fuzzy_lower;
                kd_theta_data.upper_value_error_dt[theta_kd_value_controlpointer] = error_dt_fuzzy_upper;
                kd_theta_data.lower_value_error_dt[theta_kd_value_controlpointer] = error_dt_fuzzy_lower;                         
                kd_theta_data.type3_output_label[theta_kd_value_controlpointer] = kd_matrix.kd_output_matrix[j];
                continue;
            }
        }
    }
    while(1){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        if(alpha_cut_position>4){
            alpha_cut_position = 1;
            numerator_upper = 0.0f;
            denominator_upper = 0.0f;
            numerator_lower = 0.0f;
            denominator_lower = 0.0f;
            break;
        }
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        for(int i=0;i<=theta_kd_value_controlpointer;i++){
            kd_theta_zdata.z_lower_alpha_lower_error[i] = kd_theta_data.lower_value_error[i] + (c_factor * (kd_theta_data.upper_value_error[i] - kd_theta_data.lower_value_error[i])) * alpha_cut_lower;
            kd_theta_zdata.z_lower_alpha_upper_error[i] = kd_theta_data.lower_value_error[i] + (c_factor * (kd_theta_data.upper_value_error[i] - kd_theta_data.lower_value_error[i])) * alpha_cut_upper;
            kd_theta_zdata.z_upper_alpha_upper_error[i] = kd_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_theta_data.upper_value_error[i] - kd_theta_data.lower_value_error[i])) * alpha_cut_upper;
            kd_theta_zdata.z_upper_alpha_lower_error[i] = kd_theta_data.upper_value_error[i] - ((1.0f - c_factor) * (kd_theta_data.upper_value_error[i] - kd_theta_data.lower_value_error[i])) * alpha_cut_lower;
            kd_theta_zdata.z_lower_alpha_lower_error_dt[i] = kd_theta_data.lower_value_error_dt[i] + (c_factor * (kd_theta_data.upper_value_error_dt[i] - kd_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
            kd_theta_zdata.z_lower_alpha_upper_error_dt[i] = kd_theta_data.lower_value_error_dt[i] + (c_factor * (kd_theta_data.upper_value_error_dt[i] - kd_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_theta_zdata.z_upper_alpha_upper_error_dt[i] = kd_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_theta_data.upper_value_error_dt[i] - kd_theta_data.lower_value_error_dt[i])) * alpha_cut_upper;
            kd_theta_zdata.z_upper_alpha_lower_error_dt[i] = kd_theta_data.upper_value_error_dt[i] - ((1.0f - c_factor) * (kd_theta_data.upper_value_error_dt[i] - kd_theta_data.lower_value_error_dt[i])) * alpha_cut_lower;
        }
        for(int i=0;i<=theta_kd_value_controlpointer;i++){
            numerator_upper += (kd_theta_zdata.z_lower_alpha_upper_error[i] * kd_theta_zdata.z_lower_alpha_upper_error_dt[i] * kd_label_theta.kd_output_label_theta[kd_theta_data.type3_output_label[i]])
            + (kd_theta_zdata.z_upper_alpha_upper_error[i] * kd_theta_zdata.z_upper_alpha_upper_error_dt[i] * kd_label_theta.kd_output_label_theta[kd_theta_data.type3_output_label[i]]);
            denominator_upper += (kd_theta_zdata.z_lower_alpha_upper_error[i] * kd_theta_zdata.z_lower_alpha_upper_error_dt[i])
            + (kd_theta_zdata.z_upper_alpha_upper_error[i] * kd_theta_zdata.z_upper_alpha_upper_error_dt[i]);
            numerator_lower += (kd_theta_zdata.z_lower_alpha_lower_error[i] * kd_theta_zdata.z_lower_alpha_lower_error_dt[i] * kd_label_theta.kd_output_label_theta[kd_theta_data.type3_output_label[i]])
            + (kd_theta_zdata.z_upper_alpha_lower_error[i] * kd_theta_zdata.z_upper_alpha_lower_error_dt[i] * kd_label_theta.kd_output_label_theta[kd_theta_data.type3_output_label[i]]);
            denominator_lower += (kd_theta_zdata.z_lower_alpha_lower_error[i] * kd_theta_zdata.z_lower_alpha_lower_error_dt[i])
            + (kd_theta_zdata.z_upper_alpha_lower_error[i] * kd_theta_zdata.z_upper_alpha_lower_error_dt[i]);
            
        }
        kd_theta_fdata.fout_upper[alpha_cut_position-1] = numerator_upper/denominator_upper;
        kd_theta_fdata.fout_lower[alpha_cut_position-1] = numerator_lower/denominator_lower;    
        alpha_cut_position++;
    }
    for(int i=0;i<=theta_kd_value_controlpointer;i++){
        float alpha_cut = 0.0f;
        float alpha_cut_upper = 0.0f;
        float alpha_cut_lower = 0.0f;
        if(alpha_cut_position==1){alpha_cut = alpha_cut_1;}
        if(alpha_cut_position==2){alpha_cut = alpha_cut_2;}
        if(alpha_cut_position==3){alpha_cut = alpha_cut_3;}
        if(alpha_cut_position==4){alpha_cut = alpha_cut_4;}
        alpha_cut_upper = powf(alpha_cut, type3_delta);
        alpha_cut_lower = powf(alpha_cut, 1.0f/type3_delta);
        output_numberator += (kd_theta_fdata.fout_upper[i] * alpha_cut_upper )
        + (kd_theta_fdata.fout_lower[i] * alpha_cut_lower);
        output_denominator += (alpha_cut_upper + alpha_cut_lower);
        alpha_cut_position++;
    } 
    kd_theta_increase = output_numberator/output_denominator;
    printf("kd_theta_increase->%.6f\n",kd_theta_increase);
    alpha_cut_position = 1;
    output_numberator = 0.0f;
    output_denominator = 0.0f;
    
    //=============================fuzzy type3 logic=====================================
    //Proportional	
    float proportional_x = (pid->Kp_1+kp_x_increase)* error_x;
    float proportional_y = (pid->Kp_2+kp_y_increase) * error_y;
    float proportional_theta = (pid->Kp_3+kp_theta_increase) * error_theta;	
	//Integral	
    float update_integrator_x = 0.5f * (pid->Ki_1+ki_x_increase) * pid->T * (error_x + pid->prevError_x);
    float update_integrator_y = 0.5f * (pid->Ki_2+ki_y_increase) * pid->T * (error_y + pid->prevError_y);
    float update_integrator_theta = 0.5f * (pid->Ki_3+ki_theta_increase) * pid->T * (error_theta + pid->prevError_theta);
    pid->integrator_x_1 = pid->integrator_x_1 + update_integrator_x;		
    pid->integrator_y_1 = pid->integrator_y_1 + update_integrator_y;		
    pid->integrator_theta_1 = pid->integrator_theta_1 + update_integrator_theta;		
	pid->integrator_x_2 = pid->integrator_x_2 + update_integrator_x;		
    pid->integrator_y_2 = pid->integrator_y_2 + update_integrator_y;		
    pid->integrator_theta_2 = pid->integrator_theta_2 + update_integrator_theta;
    pid->integrator_x_3 = pid->integrator_x_3 + update_integrator_x;		
    pid->integrator_y_3 = pid->integrator_y_3 + update_integrator_y;		
    pid->integrator_theta_3 = pid->integrator_theta_3 + update_integrator_theta;
    pid->integrator_x_4 = pid->integrator_x_4 + update_integrator_x;		
    pid->integrator_y_4 = pid->integrator_y_4 + update_integrator_y;		
    pid->integrator_theta_4 = pid->integrator_theta_4 + update_integrator_theta;   
    //Derivative 	
    pid->differentiator_x = error_x_dt*(pid->Kd_1+kd_x_increase) + desired_x_dt;	
    pid->differentiator_y = error_y_dt*(pid->Kd_2+kd_y_increase) + desired_y_dt;	
    pid->differentiator_theta = error_theta_dt*(pid->Kd_3+kd_theta_increase) + desired_theta_dt;	
	//input matrix variables
    float input_x_1 = proportional_x + pid->integrator_x_1 + pid->differentiator_x;
    float input_y_1 = proportional_y + pid->integrator_y_1 + pid->differentiator_y;
    float input_theta_1 = proportional_theta + pid->integrator_theta_1 + pid->differentiator_theta;
    float input_x_2 = proportional_x + pid->integrator_x_2 + pid->differentiator_x;
    float input_y_2 = proportional_y + pid->integrator_y_2 + pid->differentiator_y;
    float input_theta_2 = proportional_theta + pid->integrator_theta_2 + pid->differentiator_theta;
    float input_x_3 = proportional_x + pid->integrator_x_3 + pid->differentiator_x;
    float input_y_3 = proportional_y + pid->integrator_y_3 + pid->differentiator_y;
    float input_theta_3 = proportional_theta + pid->integrator_theta_3 + pid->differentiator_theta;
    float input_x_4 = proportional_x + pid->integrator_x_4 + pid->differentiator_x;
    float input_y_4 = proportional_y + pid->integrator_y_4 + pid->differentiator_y;
    float input_theta_4 = proportional_theta + pid->integrator_theta_4 + pid->differentiator_theta;
    //direction cw:1 ccw:0
    int motor_select=1;
    int motor_direction=1;
    for(int i=0;i<3;i++){
        float row_buffer_1 =*((float*)P->row[0]+i);
        float row_buffer_2 =*((float*)P->row[1]+i);
        float row_buffer_3 =*((float*)P->row[2]+i);
        float row_buffer_4 =*((float*)P->row[3]+i);
        if(i==0){//x
            out->out_1 += (row_buffer_1 * input_x_1);
            out->out_2 += (row_buffer_2 * input_x_2);
            out->out_3 += (row_buffer_3 * input_x_3);
            out->out_4 += (row_buffer_4 * input_x_4);
            continue;
        }
        else if(i==1){//y
            out->out_1 += (row_buffer_1 * input_y_1);
            out->out_2 += (row_buffer_2 * input_y_2);
            out->out_3 += (row_buffer_3 * input_y_3);
            out->out_4 += (row_buffer_4 * input_y_4);
            continue;
        }
        else if(i==2){//theta
            out->out_1 += (row_buffer_1 * input_theta_1);
            out->out_2 += (row_buffer_2 * input_theta_2);
            out->out_3 += (row_buffer_3 * input_theta_3);
            out->out_4 += (row_buffer_4 * input_theta_4);
            continue;
        }
        else{
            break;
        }
    }
    //check motor to do anti-windup & clamp
    while(1){
    
        if(motor_select==1 && motor_direction==1){ 
            if(out->out_1>0.0f && out->out_1>pid->limMax_cw){ 
                out->out_1 = pid->limMax_cw;
                pid->integrator_x_1 = pid->integrator_x_1 - update_integrator_x;
                pid->integrator_y_1 = pid->integrator_y_1 - update_integrator_y;
                pid->integrator_theta_1 = pid->integrator_theta_1 - update_integrator_theta;               
                motor_select=2;
                motor_direction=1;
                continue;                                  
            }
            else if(out->out_1>0.0f && out->out_1 < pid->limMin_cw){ 
                out->out_1 = pid->limMin_cw;//架空把死區加回來pid->limMin_cw;
                motor_select=2;
                motor_direction=1;
                continue;
            }
            else{
                motor_direction=0;
                continue;
            }
        }
        else if(motor_select==1 && motor_direction==0){
            if(out->out_1<0.0f && out->out_1 > pid->limMin_ccw){
                out->out_1 = pid->limMin_ccw;//架空把死區加回來pid->limMin_ccw;
                motor_select=2;
                motor_direction=1;
                continue;
            }
            else if(out->out_1<0.0f && out->out_1 < pid->limMax_ccw){
                out->out_1 = pid->limMax_ccw;
                pid->integrator_x_1 = pid->integrator_x_1 - update_integrator_x; 
                pid->integrator_y_1 = pid->integrator_y_1 - update_integrator_y;
                pid->integrator_theta_1 = pid->integrator_theta_1 - update_integrator_theta; 
                motor_select=2;
                motor_direction=1;
                continue;
            }                       
            else{
                motor_select=2;
                motor_direction=1;
                continue;
            }
        }
        else if(motor_select==2 && motor_direction==1){ 
            if(out->out_2>0.0f && out->out_2>pid->limMax_cw){ 
                out->out_2 = pid->limMax_cw;
                pid->integrator_x_2 = pid->integrator_x_2 - update_integrator_x; 
                pid->integrator_y_2 = pid->integrator_y_2 - update_integrator_y; 
                pid->integrator_theta_2 = pid->integrator_theta_2 - update_integrator_theta; 
                motor_select=3;
                motor_direction=1;
                continue;
            }
            else if(out->out_2>0.0f && out->out_2 < pid->limMin_cw){                
                out->out_2 = pid->limMin_cw;//同上;
                motor_select=3;
                motor_direction=1;
                continue;
            }
            else{
                motor_direction=0;
                continue;
            }
        }
        else if(motor_select==2 && motor_direction==0){
            if(out->out_2<0.0f && out->out_2 > pid->limMin_ccw){
                out->out_2 = pid->limMin_ccw;//同上;
                motor_select=3;
                motor_direction=1;
                continue;
            }
            else if(out->out_2<0.0f && out->out_2 < pid->limMax_ccw){
                out->out_2 = pid->limMax_ccw;
                pid->integrator_x_2 = pid->integrator_x_2 - update_integrator_x;
                pid->integrator_y_2 = pid->integrator_y_2 - update_integrator_y; //anti-windup
                pid->integrator_theta_2 = pid->integrator_theta_2 - update_integrator_theta; //anti-windup
                motor_select=3;
                motor_direction=1;
                continue;
            }
            else{
                motor_select=3;
                motor_direction=1;
                continue;
            }
        }
        else if(motor_select==3 && motor_direction==1){ 
            if(out->out_3>0.0f && out->out_3>pid->limMax_cw){ 
                out->out_3 = pid->limMax_cw;
                pid->integrator_x_3 = pid->integrator_x_3 - update_integrator_x; //anti-windup
                pid->integrator_y_3 = pid->integrator_y_3 - update_integrator_y; //anti-windup
                pid->integrator_theta_3 = pid->integrator_theta_3 - update_integrator_theta; //anti-windup
                motor_select=4;
                motor_direction=1;
                continue;
            }
            else if(out->out_3>0.0f && out->out_3 < pid->limMin_cw){ 
                out->out_3 = pid->limMin_cw;//同上;
                motor_select=4;
                motor_direction=1;
                continue;
            }
            else{
                motor_direction=0;
                continue;
            }
        }
        else if(motor_select==3 && motor_direction==0){
            if(out->out_3<0.0f && out->out_3 > pid->limMin_ccw){
                out->out_3 = pid->limMin_ccw;//同上;
                motor_select=4;
                motor_direction=1;
                continue;
            }
            else if(out->out_3<0.0f && out->out_3 < pid->limMax_ccw){
                out->out_3 = pid->limMax_ccw;
                pid->integrator_x_3 = pid->integrator_x_3 - update_integrator_x; //anti-windup
                pid->integrator_y_3 = pid->integrator_y_3 - update_integrator_y; //anti-windup
                pid->integrator_theta_3 = pid->integrator_theta_3 - update_integrator_theta; //anti-windup
                motor_select=4;
                motor_direction=1;
                continue;
            }
            else{
                motor_select=4;
                motor_direction=1;
                continue;
            }
        }
        else if(motor_select==4 && motor_direction==1){ 
            if(out->out_4>0.0f && out->out_4>pid->limMax_cw){ 
                out->out_4 = pid->limMax_cw;
                pid->integrator_x_4 = pid->integrator_x_4 - update_integrator_x; 
                pid->integrator_y_4 = pid->integrator_y_4 - update_integrator_y; 
                pid->integrator_theta_4 = pid->integrator_theta_4 - update_integrator_theta; 
                break;
            }
            else if(out->out_4>0.0f && out->out_4 < pid->limMin_cw){ 
                out->out_4 = pid->limMin_cw;//同上;
                break;
            }
            else{
                motor_direction=0;
                continue;
            }
        }
        else if(motor_select==4 && motor_direction==0){
            if(out->out_4<0.0f && out->out_4 > pid->limMin_ccw){
                out->out_4 = pid->limMin_ccw;//同上;
                break;
            }
            else if(out->out_4<0.0f && out->out_4 < pid->limMax_ccw){
                out->out_4 = pid->limMax_ccw;
                pid->integrator_x_4 = pid->integrator_x_4 - update_integrator_x; 
                pid->integrator_y_4 = pid->integrator_y_4 - update_integrator_y; 
                pid->integrator_theta_4 = pid->integrator_theta_4 - update_integrator_theta; 
                break;
            }
            else{
                break;
            }
        }

    } 
    pid->prevError_x       = error_x;
    pid->prevError_y       = error_y;
    pid->prevError_theta   = error_theta;

return;	
}

P_matrix P_matrix_init(void)
{
	float L = 0.25f;//平台中心到輪子的長度(架空測:0.25)(地面測:0.20) 
	float delta=0.7853981634f;//輪子與輪子之間固定角
	float theta=0.0f;//平台旋轉姿態角度
	P_matrix p = {
		.row_element_1 = {-sinf(delta+theta) ,  cosf(delta+theta) , L},
		.row_element_2 = {-cosf(delta+theta) , -sinf(delta+theta) , L},
		.row_element_3 = { sinf(delta+theta) , -cosf(delta+theta) , L},
		.row_element_4 = { cosf(delta+theta) ,  sinf(delta+theta) , L},
        .pretheta=theta
	};
	p.row[0] = (float*)malloc(sizeof(float)*3);
	p.row[1] = (float*)malloc(sizeof(float)*3);
	p.row[2] = (float*)malloc(sizeof(float)*3);
	p.row[3] = (float*)malloc(sizeof(float)*3);
	for(int i=0;i<3;i++){
		*((float*)p.row[0]+i)=p.row_element_1[i];
		*((float*)p.row[1]+i)=p.row_element_2[i];
		*((float*)p.row[2]+i)=p.row_element_3[i];
		*((float*)p.row[3]+i)=p.row_element_4[i];
	}
return p;
}

P_matrix_inverse P_matrix_inverse_init(void)
{
	float L = 0.25f;
	float delta=0.7853981634f;
	float theta=0.0f;
	P_matrix_inverse p = {
		.row_element_1 = {(-sinf(delta+theta))/2,(-cosf(delta+theta))/2,(sinf(delta+theta))/2,(cosf(delta+theta))/2},
		.row_element_2 = {( cosf(delta+theta))/2,(-sinf(delta+theta))/2,(-cosf(delta+theta))/2,(sinf(delta+theta))/2},
		.row_element_3 = { (1/(4*L)),(1/(4*L)),(1/(4*L)),(1/(4*L))}		
	};
	p.row[0] = (float*)malloc(sizeof(float)*4);
	p.row[1] = (float*)malloc(sizeof(float)*4);
	p.row[2] = (float*)malloc(sizeof(float)*4);
	
	for(int i=0;i<4;i++){
		*((float*)p.row[0]+i)=p.row_element_1[i];
		*((float*)p.row[1]+i)=p.row_element_2[i];
		*((float*)p.row[2]+i)=p.row_element_3[i];		
	}
return p;
}

P_matrix_inverse P_matrix_inverse_update_theata(
    P_matrix_inverse PI,
    P_matrix P)
{
    float delta=0.7853981634f;
    float pretheat = P.pretheta;
	for(int i=0;i<4;i++){
		if(i==0){
		    *((float*)PI.row[0]+i) = (-sinf(delta+pretheat))/2;
			*((float*)PI.row[1]+i) = ( cosf(delta+pretheat))/2;
			
		}
		else if(i==1){
			*((float*)PI.row[0]+i) = (-cosf(delta+pretheat))/2;
			*((float*)PI.row[1]+i) = (-sinf(delta+pretheat))/2;
				
		}
		else if(i==2){
            *((float*)PI.row[0]+i) = ( sinf(delta+pretheat))/2;
			*((float*)PI.row[1]+i) = (-cosf(delta+pretheat))/2;
            	
        }
        else if(i==3){
            *((float*)PI.row[0]+i) = ( cosf(delta+pretheat))/2;
		    *((float*)PI.row[1]+i) = ( sinf(delta+pretheat))/2;
        			
		}
        else{
            break;
        }	
	}
return PI;
}

velocity_matrix ReadEncoder_velocity(
    read_word_t read,
    velocity_matrix vm,
    PIDController *ptr,
    out_matrix OUT)
{
    if(controller_initloop==false){
        return vm;   
    }
    if(OUT.out_1==0.0f && OUT.out_2==0.0f && OUT.out_3==0.0f && OUT.out_4==0.0f){
        return vm;
    }
    int diffcw1 = read(cw_conter_addr_1);
    int diffccw1 = read(ccw_conter_addr_1);
    int diffcw2 =  read(cw_conter_addr_2);
    int diffccw2 = read(ccw_conter_addr_2);
    int diffcw3 = read(cw_conter_addr_3);
    int diffccw3 = read(ccw_conter_addr_3);
    int diffcw4 =  read(cw_conter_addr_4);
    int diffccw4 = read(ccw_conter_addr_4);
    
    int motor1_state=0;//true:cw false:ccw
    int motor2_state=0;
    int motor3_state=0;
    int motor4_state=0;
    int select_motor=1;
    
    while(1){        
        if(select_motor==1){
            if(diffcw1>diffccw1){
                motor1_state=1; //改變馬達方向,初始狀態為0(反轉)
                select_motor++;
                continue;
            }
            else
            {
                select_motor++; 
                continue;}
        }   
        else if(select_motor==2){
            if(diffcw2>diffccw2){
                motor2_state=1;
                select_motor++;
                continue;
            }
            else
            {
                select_motor++; 
                continue;}
        }   
        else if(select_motor==3){
            if(diffcw3>diffccw3){
                motor3_state=1;
                select_motor++;
                continue;
            }
            else
            {
                select_motor++; 
                continue;}
        }   
        else if(select_motor==4){
            if(diffcw4>diffccw4){
                motor4_state=1;
                select_motor=1;
                break;
            }
            else
            {   select_motor=1;               
                break;}
        }      
    };
    while(1){
        if(select_motor==1){
            if(motor1_state==1){
                vm.velocity_1 = ((float)diffcw1/1000.0f)*(60/ptr->T)*(pi*D/60);//轉換成m/s
            }else{
                vm.velocity_1 = -((float)diffccw1/1000.0f)*(60/ptr->T)*(pi*D/60);
            }
            select_motor++;
            continue;
        }   
        else if(select_motor==2){
            if(motor2_state==1){
                vm.velocity_2 = ((float)diffcw2/1000.0f)*(60/ptr->T)*(pi*D/60);//轉換成m/s
            }else{
                vm.velocity_2 = -((float)diffccw2/1000.0f)*(60/ptr->T)*(pi*D/60);
            }
            select_motor++;
            continue;
        }   
        else if(select_motor==3){
            if(motor3_state==1){
                vm.velocity_3 = ((float)diffcw3/1000.0f)*(60/ptr->T)*(pi*D/60);//轉換成m/s
            }else{
                vm.velocity_3 = -((float)diffccw3/1000.0f)*(60/ptr->T)*(pi*D/60);
            }
            select_motor++;
            continue;
        }   
        else if(select_motor==4){
            if(motor4_state==1){
                vm.velocity_4 = ((float)diffcw4/1000.0f)*(60/ptr->T)*(pi*D/60);//轉換成m/s
            }else{
                vm.velocity_4 = -((float)diffccw4/1000.0f)*(60/ptr->T)*(pi*D/60);
            }
            select_motor=1;
            break;
        }
    }
return vm;  
}
measurement_matrix_dt P_inverse_by_velocity(
    P_matrix_inverse PI,
    velocity_matrix vm,
    measurement_matrix_dt mmd,
    time_pose *tp,
    float sample_time)
{
    if(controller_initloop==false){
        return mmd;   
    }
    float *row_pointer_1 = PI.row[0];
    float *row_pointer_2 = PI.row[1];
    float *row_pointer_3 = PI.row[2];
    float vm_array[4] = {vm.velocity_1, vm.velocity_2, vm.velocity_3, vm.velocity_4};
    for(int i=0;i<4;i++){
        mmd.measurement_x += (*(row_pointer_1+i)) * vm_array[i];
        mmd.measurement_y += (*(row_pointer_2+i)) * vm_array[i];
        mmd.measurement_theta += (*(row_pointer_3+i)) * vm_array[i];        
    }
    time_pose tp_buffer = time_pose_call(execute_time);
    mmd.measurement_time_x = (tp_buffer.setpoint_x - tp->setpoint_x)/sample_time;
    mmd.measurement_time_y = (tp_buffer.setpoint_y - tp->setpoint_y)/sample_time;
    mmd.measurement_time_theta = (warp(tp_buffer.setpoint_theta) - tp->setpoint_theta)/sample_time;
    mmd.error_x_dt = mmd.measurement_time_x - mmd.measurement_x;
    mmd.error_y_dt = mmd.measurement_time_y - mmd.measurement_y;
    mmd.error_theta_dt = mmd.measurement_time_theta - warp(mmd.measurement_theta);
    tp->setpoint_x = tp_buffer.setpoint_x;
    tp->setpoint_y = tp_buffer.setpoint_y;
    tp->setpoint_theta = warp(tp_buffer.setpoint_theta);
return mmd;
} 

setpoint_queue_pointer queue_init(void){
    setpoint_queue_pointer sqp = {
        .front = NULL,
        .rear = NULL,
        .size = 0
    }; 
return sqp;
}

int queue_empty(setpoint_queue_pointer *sqp){
    if(sqp->rear==NULL && sqp->front==NULL && sqp->size==0){
        printf("queue empty\n");
        return 1;//empty
    }else{
        printf("queue not empty\n");
        return 0;//not empty
    }
}

int push_queue(
    setpoint_queue_pointer *sqp,
    setpoint_queue_data new_data)
{
    setpoint_queue_node *push_newnode = 
    (setpoint_queue_node*)malloc(sizeof(setpoint_queue_node));
    if(push_newnode==NULL){
        printf("Heap segment memory allocation error\n");
        return 0;
    }
    push_newnode->data = new_data;
    push_newnode->next = NULL;
    if(sqp->size==0 || sqp->front==NULL || sqp->rear==NULL){
        sqp->front = push_newnode;
        sqp->rear = push_newnode;
        sqp->size++;
        return 1;
    }else{ 
        sqp->rear->next = push_newnode;
        sqp->rear = push_newnode;
        sqp->size++;
        return 1;
    }  
}

int pop_queue(setpoint_queue_pointer *sqp)
{
    if(sqp->size==0 && sqp->front==NULL && sqp->rear==NULL){
        printf("Queue empty, cannot pop\n");
        return 0;
    }else if(sqp->front==sqp->rear){
        setpoint_queue_node *temp = sqp->front;
        sqp->front = NULL;
        sqp->rear = NULL;
        free(temp);
        sqp->size--;
        return 1;
    }
    else{
        setpoint_queue_node *temp = sqp->front;
        sqp->front = sqp->front->next;
        free(temp);
        sqp->size--;
        return 1;
    }
}

void out_to_pwm(out_matrix out)
{
    int direct_state=1;
    switch (direct_state)
    {
    case 1:
        if(out.out_1>0){
            scale_factor_1 = 2500+(out.out_1-0.04f)*30821;         
            direct_state++;
        }
        else if(out.out_1<0){
            scale_factor_1 = -(2500+(fabsf(out.out_1)-0.04f)*30821);
            direct_state++;
        }
        else{direct_state++;}   
    case 2:
       if(out.out_2>0){
            scale_factor_2 = 2500+(out.out_2-0.04f)*30821;
            direct_state++;
        }
        else if(out.out_2<0){
            scale_factor_2 = -(2500+(fabsf(out.out_2)-0.04f)*30821);
            direct_state++;
        }
        else{direct_state++;} 
    case 3:
       if(out.out_3>0){
            scale_factor_3 = 2500+(out.out_3-0.04f)*30821;
            direct_state++;
        }
        else if(out.out_3<0){
            scale_factor_3 = -(2500+(fabsf(out.out_3)-0.04f)*30821);
            direct_state++;
        }
        else{direct_state++;} 
    case 4:
       if(out.out_4>0){
            scale_factor_4 = 2500+(out.out_4-0.04f)*30821;
            break;
        }
        else if(out.out_4<0){
            scale_factor_4 = -(2500+(fabsf(out.out_4)-0.04f)*30821);
            break;
        }
        else{break;}
       
    }
    pwm_1=(int)(75000 + scale_factor_1);
    pwm_2=(int)(75000 + scale_factor_2);
    pwm_3=(int)(75000 + scale_factor_3);
    pwm_4=(int)(75000 + scale_factor_4);
    for(int j=0;j<4;j++){
        if(j==0){
           alt_write_word(pwm_width_addr_1,pwm_1); 
        }else if(j==1){
           alt_write_word(pwm_width_addr_2,pwm_2); 
        }else if(j==2){
           alt_write_word(pwm_width_addr_3,pwm_3); 
        }else if(j==3){
           alt_write_word(pwm_width_addr_4,pwm_4); 
        }
    }
      
return;
}

int check_goal_reached(
    peek_queue_t peek,
    setpoint_queue_pointer sqp,
    PIDController *pid)    
{
    float buffer_x = peek(sqp).setpoint_x - pid->prevMeasurement_x;
    float buffer_y = peek(sqp).setpoint_y - pid->prevMeasurement_y;
    float buffer_theta = peek(sqp).setpoint_theta - pid->prevMeasurement_theta;
    float dist = sqrtf((buffer_x*buffer_x)+(buffer_y*buffer_y));
    float dth = warp(buffer_theta);
    dth=fabsf(dth);  
    if((dist < 0.2f && dth < 1.0f)
    || (fabsf(pid->prevMeasurement_x) >= fabsf(peek(sqp).setpoint_x) 
    &&  fabsf(pid->prevMeasurement_y) >= fabsf(peek(sqp).setpoint_y)))
    {
        return 1;   
    }
    else{
        return 0;
    }
}


int check_time_reached(
    float execute_time,
    PIDController *pid)    
{
    //flower execute time 250s
    //ellipse execute time 155s
    if(execute_time >= 155.0f){//time limit stop condition
        printf("Time overhead,stop!!\n");
        return 1;
    }
    else if(execute_time > 100.0f){//circle tracking stop condition
        float buffer_x = pid->prevMeasurement_x;
        float buffer_y = pid->prevMeasurement_y;
        time_pose tp_init_buffer = time_pose_call(0.000111f);
        float dist = sqrtf((buffer_x - tp_init_buffer.setpoint_x)*(buffer_x - tp_init_buffer.setpoint_x) + 
        (buffer_y - tp_init_buffer.setpoint_y)*(buffer_y - tp_init_buffer.setpoint_y));
        float dth = warp(pid->prevMeasurement_theta);
        dth = fabsf(dth);
        if(dist < 0.01f && dth < 0.0001f){
            printf("position reached, stop!!\n");
            return 1;
        }
        else{
            return 0;
        }
    }
    else{
        return 0;
    }
}










