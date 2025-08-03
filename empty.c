/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include <stdio.h>
//#include "bsp_mpu6050.h"
//#include "inv_mpu.h"
#include "board.h"
#include "bsp_gyro.h"
#include "mpu6050.h"
#include "oled.h"

#define MOTOR_STOP  1
int32_t Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt,PWMA,PWMB;
int32_t motor_speed=10;//电机速度控制，0为停止
int32_t motor_mode = MOTOR_STOP;//0默认停止  1速度环  2位置环  3循迹
static ENCODER_RES motor_encoder;
static ENCODER_RES motor_encoder1;


int boot1,boot2 = 0;
int sensor1,sensor2,sensor3,sensor4 = 1;
bool x[4];
int DelayTime;		
int	pL,pR=500;
int RtSum=0;
const int SpFast=500,SpMid=300,SpSlow=10,SpStop=2,SpN=800;
bool inTrace=0,InTrace=0;
int BlankTimes=0;
int BeepS=0;
bool right_1;
bool right_2;
bool right_3;
bool left_1;
bool left_2;
bool left_3;
float yaw = 0;
float pitch = 0;
float roll = 0;


uint8_t id;
int16_t AccX = 0;
int16_t AccY = 0;
int16_t AccZ = 0;
int16_t GyroX = 0;
int16_t GyroY = 0;
int16_t GyroZ = 0;

void MPU_task(){
  
     MPU6050_GetData(&AccX,&AccY, &AccZ, &GyroX,&GyroY, &GyroZ);
	   MPU6050_Read();
	   ComputeEulerAngles(&pitch, &roll, &yaw);
}




//pid
#define Speed_Integral_Start_Err 100
#define Speed_Integral_Max_Val 50
#define output_val_MAX    120.0f
#define output_val_RMAX    40.0f

float target_val;
float output_val;

float error;
float err_last;
float integral;
float Kp;
float Ki;
float Kd;

void set_PID(void){
	  Kp = 0;
	  Ki = 2;
	  Kd = 0.1;
	
}

void detect_Pid_Cal(float target_val ,float Speed_Current)
{
    error=target_val-Speed_Current;

    if((error >- Speed_Integral_Start_Err)&&(error<Speed_Integral_Start_Err))//积分限幅
    {
        integral+=error;
        if(integral<-50)
        {
            integral=-50;
        }
        if(integral>Speed_Integral_Max_Val)
        {
            integral=Speed_Integral_Max_Val;
        }
    }
    output_val=Kp * error + Ki * integral + Kd * (error-err_last);

		if(output_val < 0 && output_val< -output_val_MAX){
			
			output_val = -output_val_MAX;
		
		}
		
		if(output_val > 0 && output_val> output_val_MAX){
			
			output_val = output_val_MAX;
		
		}
		
    err_last=error;

		
}



//pid

///////detect

int cnt=0;
float detect_value = 0;
float last_detect_value = 0;
#define Mid_Value 3.0f
#define Mid_Value_LIMIT 25.0f
#define Normal_pwm_set 300.0f


float pwm_set_R = 0;
float pwm_set_L = 0;

void readTrace(void){  //目前引脚检测到黑线时电平为高
	left_1= !DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X1_PORT,GPIO_GRP_Detect_PIN_X1_PIN);
	left_2= !DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X2_PORT,GPIO_GRP_Detect_PIN_X2_PIN);
	left_3=  DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X5_PORT,GPIO_GRP_Detect_PIN_X5_PIN);
	right_1= !DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X3_PORT,GPIO_GRP_Detect_PIN_X3_PIN);
	right_2= !DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X4_PORT,GPIO_GRP_Detect_PIN_X4_PIN);
	right_3= DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X6_PORT,GPIO_GRP_Detect_PIN_X6_PIN);
	printf("left_1=%d,left_2=%d,left_3=%d,right_1=%d,right_2=%d,right_3=%d\n",left_1,left_2,left_3,right_1,right_2,right_3);
	
}

//void turn_left(){
//	
//	set_motor_rotateL(200);
//	set_motor_rotateR(10);

//}

void Search_Trace(void){
	
	readTrace();
	
//	if(left_3 == 1){
//		turn_left();
//	
//	}
	
	if(left_3 == 1){
		cnt ++;
	
	}
  if(left_2 == 1){
	
		cnt ++;
	}
	 if(left_1 == 1){
	
		cnt ++;
	}
	if(right_1 == 1){
		cnt ++;
	
	}
  if(right_2 == 1){
	
		cnt ++;
	}
	 if(right_3 == 1){
	
		cnt ++;
	}
	if(cnt == 0){
		cnt=1;
	
	}
	

	detect_value = (1*left_3 + 2*left_2 + 3*left_1 + 4*right_1 + 5*right_2 + 6*right_3)/cnt;
	
	
	
	if(detect_value != 0){
		last_detect_value = detect_value;
	
	}
	else{
		detect_value = last_detect_value;
		
		
	}
	printf("detect_value0 = %f\n",detect_value);
	
	detect_Pid_Cal(Mid_Value,detect_value);
	printf("detect_value1 = %f\n",detect_value);
	printf("output_val = %f\n",output_val);

		
	pwm_set_R = Normal_pwm_set - output_val;
	pwm_set_L = Normal_pwm_set - output_val;
	
	
	if( detect_value > Mid_Value){
		
			pwm_set_R = Normal_pwm_set + output_val;
	    pwm_set_L = Normal_pwm_set - output_val;
		
	
	}
	 if( detect_value < Mid_Value){
		
			pwm_set_R = Normal_pwm_set - output_val;
	    pwm_set_L = Normal_pwm_set + output_val;
			
			
		
	
	}

	
  if(detect_value == Mid_Value){
		if(output_val < 0 && output_val< -Mid_Value_LIMIT){
			
			output_val = -Mid_Value_LIMIT;
		
		}
		
		if(output_val > 0 && output_val> Mid_Value_LIMIT){
			
			output_val = Mid_Value_LIMIT;
		
		}
		
	
	}
	
	
	set_motor_rotateL(pwm_set_L);
	set_motor_rotateR(pwm_set_R);
//	

	
	printf("pwm_set_R = %f\n",pwm_set_R);
	printf("pwm_set_L = %f\n",pwm_set_L);
	
	cnt = 0;
	
	
   
}



int a = 0;
int last_a =0;
int set_circle = 0;
#define N_MAX  6
void KEY_READ(void){
	a = DL_GPIO_readPins(GPIO_GRP_0_PORT,GPIO_GRP_0_PIN_KEY1_PIN);
	if(last_a == 0 && a >4000){
		set_circle = (set_circle + 1 ) % N_MAX ;
	}
	last_a = a;
	OLED_ShowNum(1,3,set_circle,1);
	
}

//// 速度宏定义
//#define BASE_SPEED_L  300  // 基础速度
//#define BASE_SPEED_R  300  // 基础速度
//#define ADJUST_S    80   // 轻微调整量（用于S3/S4）
//#define ADJUST_M    150  // 中度调整量（用于S2/S5）
//#define ADJUST_L    50  // 转向调整量（用于左转/右转）

//int move_cnt;
//int turn_cnt=0;
//float pwmDuty_R;
//float pwmDuty_L;
//int flag_normal=0;
//int flag_turn = 1;
//int flag_turn_success = 0;

//void turn_left(void){
//	
//	set_motor_rotateL(50);
//	set_motor_rotateR(50);
//	delay_ms(2);
//	set_motor_rotateR(-300);
//	set_motor_rotateL(300);

//	
//	

//}


//void go_ahead(void){
//	
//		
//							pwmDuty_L = BASE_SPEED_R;
//							pwmDuty_R = BASE_SPEED_L;
//   

//				//状态一:机体轻微向右偏,运行方向的右电机加速[实际左电机函数]
//				if(left_2 == 0 && left_1 == 1 && right_1 == 0 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R + ADJUST_S;
//						pwmDuty_R = BASE_SPEED_L - ADJUST_S;
//				}
//				//状态二:机体轻微向左偏,运行方向的左电机加速[实际右电机函数]
//				else if (left_2 == 0 && left_1 == 0 && right_1 == 1 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R - ADJUST_S;
//						pwmDuty_R = BASE_SPEED_L + ADJUST_S;
//				}
//				//状态三:机体轻微向右偏中等,运行方向的右电机加速[实际左电机函数]
//				else if (left_2 == 1 && left_1 == 0 && right_1 == 0 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R + ADJUST_M;
//						pwmDuty_R = BASE_SPEED_L - ADJUST_M;
//				}
//				//状态四:机体轻微向左偏中等,运行方向的左电机加速[实际右电机函数]
//				else if (left_2 == 0 && left_1 == 0 && right_1 == 0 && right_2 == 1 ){
//					
//						pwmDuty_L = BASE_SPEED_R - ADJUST_M;
//						pwmDuty_R = BASE_SPEED_L + ADJUST_M+50;
//				}
//				
//				else if(left_2 == 0 && left_1 == 1 && right_1 == 1 && right_2 == 0 ){
//					
//							pwmDuty_L = BASE_SPEED_R;
//							pwmDuty_R = BASE_SPEED_L;				
//				
//				}
//        set_motor_rotateL(pwmDuty_L);
//        set_motor_rotateR(pwmDuty_R);


//}

//void Trace(void){
//		set_motor_rotateL(50);
//	set_motor_rotateR(50);
//	delay_ms(2);
//	if(left_3 ==1){
//		 set_motor_rotateL(50);
//	   set_motor_rotateR(50);
//	   delay_ms(2);
//		while(flag_turn){
//			
//			turn_left();
//			
//		
//		}
//		flag_turn=1;
//		
//		turn_cnt++;
//	}
//	else if(flag_turn == 1 && (left_2 ==1 | left_1==1| right_1 ==1)){
//		
//		go_ahead();
//		flag_turn = 0;

//  }
//	if(flag_turn == 0){

//     		go_ahead();
//	}
//	
//	

//}



//void Tracking_Run(int circle)
//void Detect(void){

//}
//void Tracking_Run(void){
//	
//	  readTrace();
//	

////    if (turn_cnt<4*set_circle+1&&set_circle!=0)
////    {
//	
//							pwmDuty_L = BASE_SPEED_R;
//							pwmDuty_R = BASE_SPEED_L;
//   

//				//状态一:机体轻微向右偏,运行方向的右电机加速[实际左电机函数]
//				if(left_2 == 0 && left_1 == 1 && right_1 == 0 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R + ADJUST_S;
//						pwmDuty_R = BASE_SPEED_L - ADJUST_S;
//				}
//				//状态二:机体轻微向左偏,运行方向的左电机加速[实际右电机函数]
//				else if (left_2 == 0 && left_1 == 0 && right_1 == 1 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R - ADJUST_S;
//						pwmDuty_R = BASE_SPEED_L + ADJUST_S;
//				}
//				//状态三:机体轻微向右偏中等,运行方向的右电机加速[实际左电机函数]
//				else if (left_2 == 1 && left_1 == 0 && right_1 == 0 && right_2 == 0 ){
//					
//						pwmDuty_L = BASE_SPEED_R + ADJUST_M;
//						pwmDuty_R = BASE_SPEED_L - ADJUST_M;
//				}
//				//状态四:机体轻微向左偏中等,运行方向的左电机加速[实际右电机函数]
//				else if (left_2 == 0 && left_1 == 0 && right_1 == 0 && right_2 == 1 ){
//					
//						pwmDuty_L = BASE_SPEED_R - ADJUST_M;
//						pwmDuty_R = BASE_SPEED_L + ADJUST_M+50;
//				}
//				
////				else if (left_3 == 1 && left_2 == 0 && left_1 == 0 && right_1 == 0 && right_2 == 0 && right_3 ==0){
////					
////						pwmDuty_L = BASE_SPEED_R + ADJUST_M + 50;
////						pwmDuty_R = BASE_SPEED_L - ADJUST_M;
////				}
////				
////				else if (left_3 == 0 && left_2 == 0 && left_1 == 0 && right_1 == 0 && right_2 == 0 && right_3 ==1){
////					
////						pwmDuty_L = BASE_SPEED_R - ADJUST_M;
////						pwmDuty_R = BASE_SPEED_L + ADJUST_M;
////				}
//			

////				// 逻辑6：S4+S5高 或 S4+S5+S6高→右转，左加速右减速
////				else if ((right_1 == 1 && right_2 == 1 && left_2 == 0 && left_1 == 0)) {
////						pwmDuty_L  = BASE_SPEED_R -ADJUST_M;
////						pwmDuty_R = BASE_SPEED_L +ADJUST_M;
////				}
////				else if (left_2 == 1 && left_1 == 1 && right_1 == 0 && right_2 == 0) {
////						pwmDuty_L  = BASE_SPEED_R + ADJUST_M;
////						pwmDuty_R = BASE_SPEED_L - ADJUST_M;;
////				}
////				 else if ((right_3 == 1 && right_2 == 1) || (right_3 == 1 && right_2 == 1 && right_1 == 1)) {
////						pwmDuty_L  = BASE_SPEED_R -ADJUST_S;
////						pwmDuty_R = BASE_SPEED_L +ADJUST_S;
////				}


//				else if(left_2 == 0 && left_1 == 1 && right_1 == 1 && right_2 == 0 ){
//					
//							pwmDuty_L = BASE_SPEED_R;
//							pwmDuty_R = BASE_SPEED_L;				
//				
//				}
//				
//				if (left_3 == 1) {
//					
//					   	           
//							pwmDuty_L =250;
//							pwmDuty_R =-200;
//					  
//					   

//				}
//				else if((flag_turn ==1 ) && (left_2 == 1 | left_1==1 | right_1 == 1 | right_2 ==1)){
//					
//					flag_turn = 0;
//					turn_cnt++;
//					flag_turn_success = 1;
//					pwmDuty_L = BASE_SPEED_R - ADJUST_M;
//					pwmDuty_R = BASE_SPEED_L + ADJUST_M;					
//					
//				
//				}
//	
////				else if (left_3 == 1 && left_2 == 1 && left_1 == 1 && right_1 == 1 && right_2 == 0 && right_3 ==0) {
////					
////	           
////		        set_motor_rotateL(200);
////		        set_motor_rotateR(1);	
////					  
////					   

////				}		
////				else if (left_3 == 1 && left_2 == 1 && left_1 == 1 && right_1 == 1 && right_2 == 1 && right_3 ==0) {
////					
////	
////		       pwmDuty_L =200;
////					 pwmDuty_R =1;
////	
////					  
////					   

////				}	
////				else if (left_3 == 1 && left_2 == 1 && left_1 == 1 && right_1 == 1 && right_2 == 1 && right_3 ==1) {
////					
////	
////					pwmDuty_L =200;
////					pwmDuty_R =1;
////					 
////					  
////					   

////				}	
////				else if (left_3 == 0 && left_2 == 0&& left_1 == 0&& right_1 == 0 && right_2 == 0 && right_3 ==0) {
////					
////					
////					pwmDuty_L =250;
////					pwmDuty_R =-100;
////					
////					   

////				}				
////				else if (left_3 == 0 && left_2 == 0 && left_1 == 0 && right_1 == 0 && right_2 == 0 && right_3 ==0){  //左转
////						 pwmDuty_L = 0;
////						 pwmDuty_R = BASE_SPEED_L + ADJUST_L;
////				
////				}
////				else
////        {
////							pwmDuty_L = BASE_SPEED_R;
////							pwmDuty_R = BASE_SPEED_L;
//////							move_cnt++;
////          }

//					
//					
//					
//		
//    set_motor_rotateL(pwmDuty_L);
//    set_motor_rotateR(pwmDuty_R);
//	
//}

int main(void)
{

//
      SYSCFG_DL_init();	
      board_init();
//	    timer_init();
//	    encoder_init();
			DL_TimerA_startCounter(PWM_AB_INST);
	    set_PID();
	    OLED_Init();

	    OLED_ShowString(1,1,"N=");


      while (1)
      {
				    KEY_READ();


				    MPU_task();
				
//				    readTrace();
				
				    Search_Trace();
				
//            printf("a= %d\n",1);
//				    OLED_ShowString(1,1,"Hello");

//            delay_ms(100);
//				    Trace();
//		        set_motor_rotateL(250);
//		        set_motor_rotateR(-250);

      }
} 




