#ifndef __BSP_PID_H
#define __BSP_PID_H


typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output_limit; // ����޷�ֵ������ֵ��
    float integral_limit; // �����޷�ֵ������ֵ��
    float dead_zone;    // ������ֵ
	  float integral;
    float last_error;
}PID_;

void PID_Init(PID_* pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit, float dead_zone);

float PID_compute(PID_* pid, float set, float get);



#endif