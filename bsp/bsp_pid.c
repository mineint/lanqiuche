#include "bsp_pid.h"
#include <math.h>
#include "main.h"

void PID_Init(PID_* pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit, float dead_zone) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;          
    pid->dead_zone = dead_zone;
    pid->integral = 0;                               //误差积分
	  pid->last_error = 0;                               //上一次的误差
}

float PID_compute(PID_* pid, float set, float get) 
{
    float error = set - get;
    if(fabs(error) <= pid->dead_zone) {
        error = 0;
    }
    float kp_out = pid->Kp * error;               //kp部分

    pid->integral += error;
    pid->integral = fmaxf(fminf(pid->integral, pid->integral_limit), -pid->integral_limit);       //防止超出限制
    float ki_out = pid->Ki * pid->integral;       //ki部分

    float kd_out = error - pid->last_error;
    kd_out = pid->Kd * kd_out;                    //kd部分

    float output = kp_out + ki_out + kd_out;
    output = fmaxf(fminf(output, pid->output_limit), -pid->output_limit);         //防止超出限制

    pid->last_error = error;

    return output;
}