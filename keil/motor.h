#ifndef motor_h
#define motor_h

#include <stdio.h>
#include "board.h"
#define ABS(a)      (a>0 ? a:(-a))

	
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

void motor_init(void);

extern int32_t Get_Encoder_countA,Get_Encoder_countB,encoderA_cnt,encoderB_cnt;

void motor_init(void);
void Set_PWM(int pwma,int pwmb);
void motor_stop();
void Motor_BL_go();
void Motor_BL_back();
void Motor_Stop_L();
void Motor_BR_go();
void Motor_BR_back();
void Motor_Stop_R();

typedef enum {
    FORWARD,  // 正向
    REVERSAL  // 反向
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //保存实时计数值
    int count;         						 //根据定时器时间更新的计数值
    ENCODER_DIR dir;            	 //旋转方向
	  float SPEED;    //转速
} ENCODER_RES;


void timer_init(void);
void encoder_init(void);

void encoder_init(void);
int get_encoder_count(void);
ENCODER_DIR get_encoder_dir(void);
void encoder_update(void);

int get_encoderR_count(void);
ENCODER_DIR get_encoderR_dir(void);
void encoderR_update(void);

#endif