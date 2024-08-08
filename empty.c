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
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "board.h"

int flag = 0;
char s_yaw[9];
uint8_t ret = 1;
float pitch = 0,roll = 0,yaw = 0;   //欧拉角
int boot1,boot2 = 0;
int sensor1,sensor2,sensor3,sensor4 = 1;
bool a[4];
int DelayTime;		
int	pL,pR=500;
int RtSum=0;
const int SpFast=500,SpMid=300,SpSlow=10,SpStop=2,SpN=800;
bool inTrace=0,InTrace=0;
int BlankTimes=0;
int BeepS=0;

void Task1(){
				if (flag == 1){
					while(1){}
				}
				set_motor_rotateL(400);
				set_motor_rotateR(460);
				//获取欧拉角
        if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        { 
            printf("%d\n", (int)yaw);
        }     
				
					sensor1 = DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X1_PORT,GPIO_GRP_Detect_PIN_X1_PIN);
					sensor2 = DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X2_PORT,GPIO_GRP_Detect_PIN_X2_PIN);
					sensor3 = DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X3_PORT,GPIO_GRP_Detect_PIN_X3_PIN);
					sensor4 = DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X4_PORT,GPIO_GRP_Detect_PIN_X4_PIN);
		
				if(sensor1 == 0 ||sensor2 == 0 ||sensor3 == 0 ||sensor4 == 0 ){
					flag = 1;
					Motor_Back();
					delay_ms(50);
					Motor_Stop();
					DL_GPIO_setPins(GPIOA ,GPIO_GRP_0_PIN_Buzzer_PIN);
					delay_ms(2000);
					DL_GPIO_clearPins(GPIOA ,GPIO_GRP_0_PIN_Buzzer_PIN);
					delay_ms(1000);
				}
			}

void readTrace(bool a[]){
	a[1]=!DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X1_PORT,GPIO_GRP_Detect_PIN_X1_PIN);
	a[0]=!DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X2_PORT,GPIO_GRP_Detect_PIN_X2_PIN);
	a[2]=!DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X3_PORT,GPIO_GRP_Detect_PIN_X3_PIN);
	a[3]=!DL_GPIO_readPins(GPIO_GRP_Detect_PIN_X4_PORT,GPIO_GRP_Detect_PIN_X4_PIN);
}

void Task2(){
	if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        { 
            printf("%d\n", (int)yaw);
        }     
				
	readTrace(a);
	switch(a[0]*8+a[1]*4+a[2]*2+a[3]*1){
		case 0:
			if(RtSum<14){
				RtSum++;
				pL=335;
				pR=SpSlow;
				DelayTime=1;
			}
			else{
				pL=SpN;
				pR=SpN;
				DelayTime=1;
//				if(!BlankTimes)BlankTimes=10;
			}
//			inTrace=0;
			if(RtSum==14){
				inTrace=0;
			}
			break;
		case 8:
			pL=SpSlow;
			pR=SpN;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 4:
			pL=SpSlow;
			pR=SpFast;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 2:
			pL=SpFast;
			pR=SpSlow;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 1:
			pL=SpN;
			pR=SpSlow;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 12:
			pL=SpStop;
			pR=SpFast;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 3:
			pL=SpFast;
			pR=SpStop;
			DelayTime=1;
			RtSum=0;
			inTrace=1;
			break;
		case 6:
			pL=SpMid;
			pR=SpMid;
			inTrace=1;
			break;
		case 15:
			pL=1;
			pR=1;
			DelayTime=50;
			RtSum=0;
			inTrace=1;
			break;
		case 7:
			pL=SpStop;
			pR=SpN;
			inTrace=1;
			break;
		case 14:
			pL=SpN;
			pR=SpStop;
			inTrace=1;
			break;
		default:
			inTrace=1;
			break;
	}
	set_motor_rotateL(pL);
	set_motor_rotateR(pR);
	printf("%d%d%d%d,%d\nL=%d,R=%d\n",a[0],a[1],a[2],a[3],(int)yaw,pL,pR);
	delay_ms(1);
	if(inTrace!=InTrace)BlankTimes=1;
	else if(BlankTimes)BlankTimes++;
	if(BlankTimes>=4){
		BlankTimes=0;
		BeepS=15;
	}
	if(BeepS){
		DL_GPIO_setPins(GPIOA ,GPIO_GRP_0_PIN_Buzzer_PIN);
		BeepS--;
		printf("\n\n---%d\n\n",BeepS);
	}
	if(!BeepS)DL_GPIO_clearPins(GPIOA ,GPIO_GRP_0_PIN_Buzzer_PIN);
	InTrace=inTrace;
}


int main(void)
{
    SYSCFG_DL_init();
	
	  DL_TimerA_startCounter(PWM_AB_INST);
//	  NVIC_EnableIRQ(GPIO_GRP_EncoderA_GPIOA_INT_IRQN);//使能左侧外部中断
//	  NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);//使能右侧外部中断
//	
//	  NVIC_EnableIRQ(TIMER_10ms_INST_INT_IRQN);  // 使能中断
//	  DL_TimerA_startCounter(TIMER_10ms_INST);	 // 开始计时
	
	 //开发板初始化
//    board_init();
		MPU6050_Init();
	 //DMP初始化
    while( mpu_dmp_init() )
    {
        printf("dmp error\r\n");
        delay_ms(200);
    }

    printf("Initialization Data Succeed \r\n");
	  printf("Initialze Complete\r\n");

		boot1 = DL_GPIO_readPins(GPIOA ,GPIO_GRP_0_PIN_KEY1_PIN  );
		boot2 = DL_GPIO_readPins(GPIOB ,GPIO_GRP_0_PIN_KEY2_PIN  );
		printf("BOOT1= %d\r\n",boot1);
		printf("BOOT2= %d\r\n",boot2);
		

//		Motor_Forward();
//		pL=500;
//		delay_ms(1000);
//		Motor_Stop();
//		delay_ms(1000);
//		Motor_Forward();

		
		
//	  DL_TimerG_setCaptureCompareValue(PWM_AB_INST, 1500, GPIO_PWM_AB_C0_IDX);
//		DL_TimerG_setCaptureCompareValue(PWM_AB_INST, 1500, GPIO_PWM_AB_C1_IDX);
//		Motor_Forward();
//		set_motor_rotateL(400);
//		set_motor_rotateR(420);
		
		

//	    delay_ms(10000);



	while(1){
		if (boot1 == 0){
			if(boot2 == 0){
				printf("P00\r\n");
			}
			else if(boot2 == 262144){
				printf("P01\r\n");
				Task2();
			}	
		}
		else if(boot1 == 4096){
			if(boot2 == 0){
				printf("P10\r\n");
				
				}
			else if(boot2 == 262144){
				printf("P11\r\n");
				Task1();
				}
		}
	}
	
} 

