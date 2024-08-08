#ifndef motor_h
#define motor_h

#include <stdio.h>



void Motor_Forward();
void Motor_Back();
void Motor_Stop();

void Motor_FL();
void Motor_FR();
void Motor_BL();
void Motor_BR();
void Motor_SL();
void Motor_SR();

	
void PID_param_init();
void set_p_i_d(float p, float i, float d);
//void set_pid_target(float temp_val);

void Set_Speedtarget_L(float temp_val);
void Set_Speedtarget_R(float temp_val);

void set_pwmL(int pwm);
void set_pwmR(int pwm);

//通过PWM以及正负号控制左侧两个电机转动的方向和速度
void set_motor_rotateL(int pwm);
//通过PWM以及正负号控制右侧两个电机转动的方向和速度
void set_motor_rotateR(int pwm);

struct PID
{
    float target_val;
    float  output_val;
    float error;
    float err_last;
    float integral;
    float Kp;
    float Ki;
    float Kd;
};

float Speed_Pid_Cal(struct PID *pid,float Speed_Current);



#endif