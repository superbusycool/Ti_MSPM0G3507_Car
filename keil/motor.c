#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "motor.h"

#include "board.h"

extern uint32_t PWMA,PWMB;
extern int32_t motor_speed;
extern int32_t motor_mode;//0默认停止  1速度环  2位置环  3循迹

void motor_init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	
		NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
	NVIC_EnableIRQ(GPIOA_INT_IRQn);
	
//		/*使能编码器输入捕获中断*/
//	NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);
//	DL_TimerA_startCounter(ENCODER_GPIOA_INST);
//	NVIC_EnableIRQ(ENCODER2A_INST_INT_IRQN);
//	DL_TimerG_startCounter(ENCODER2A_INST);

  //定时器中断
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	//若没有在sysconfig勾选Start Timer则需手动开启
   DL_TimerG_startCounter(TIMER_0_INST);                 //开启计数

	printf("Motor initialized successfully\r\n");
}

void Motor_BL_go(){

	DL_GPIO_setPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
	DL_GPIO_clearPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);

}

void Motor_BL_back(){
	
	DL_GPIO_setPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
	DL_GPIO_clearPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);

}

void Motor_Stop_L(){
	
	DL_GPIO_clearPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
	DL_GPIO_clearPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);

}

void Motor_BR_go(){

	DL_GPIO_setPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
	DL_GPIO_clearPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);

}

void Motor_BR_back(){

	DL_GPIO_setPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
	DL_GPIO_clearPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);

}

void Motor_Stop_R(){
	DL_GPIO_clearPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
	DL_GPIO_clearPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
}

#define Speed_Integral_Start_Err 100
#define Speed_Integral_Max_Val 50


struct PID pid_speed;
void PID_param_init()
{
    pid_speed.target_val=0.0;
    pid_speed.output_val=0.0;
    pid_speed.error=0.0;
    pid_speed.err_last=0.0;

    pid_speed.Kd=0.0;
    pid_speed.Ki=0.0;
    pid_speed.Kp=0.0;

}

/**
  * @brief  设置比例
  * @param  P
  * @param  i
  * @param  d
  *	@note 	无
  * @retval 无
  */
void set_p_i_d(float p, float i, float d)
{
    pid_speed.Kp = p;    //  P
    pid_speed.Ki = i;    //  I
    pid_speed.Kd = d;    //  D
}

///**
//  * @brief  设置目标值
//  * @param  val		目标值
//  *	@note 	无
//  * @retval 无
//  */
//void set_pid_target(float temp_val)
//{
//    pid_speed.target_val = temp_val;    // 设置当前的目标值
//}


//设置PWM输出的占空比，注意这里的最大2000是根据ARR自动重装载的值
void set_pwmL(int pwm)
{
    if(pwm < 0 || pwm > 1999)
    {
        printf("pwm para err!!!\r\n");
        return; /*只能是0~1999*/
    }

    DL_TimerG_setCaptureCompareValue(PWM_AB_INST, 2000 - pwm, GPIO_PWM_AB_C0_IDX);
}

//设置PWM输出的占空比，注意这里的最大2000是根据ARR自动重装载的值
void set_pwmR(int pwm)//右侧不能给零
{
    if(pwm < 0 || pwm > 1999)
    {
        printf("pwm para err!!!\r\n");
        return; /*只能是0~1999*/
    }

    DL_TimerG_setCaptureCompareValue(PWM_AB_INST, 2000-pwm, GPIO_PWM_AB_C1_IDX);
}


//通过PWM以及正负号控制左侧两个电机转动的方向和速度
void set_motor_rotateL(int pwm)
{

        if(pwm <  0){
					 Motor_BL_back();
					 set_pwmL(-pwm);
				}
				else{
				   Motor_BL_go();
					set_pwmL(pwm);
				}


}

//通过PWM以及正负号控制右侧两个电机转动的方向和速度
void set_motor_rotateR(int pwm)
{
        if(pwm <  0){
					 Motor_BR_back();
					 set_pwmR(-pwm);
				}
				else{
				   Motor_BR_go();
					set_pwmR(pwm);
				}

}

/**
  * @brief  设置左侧两轮目标值
  * @param  val		目标值
  *	@note 	无
  * @retval 无
  */
void Set_Speedtarget_L(float temp_val)
{
    pid_speed.target_val = temp_val;    // 设置当前左侧的目标值
}



/**
  * @brief  设置右侧两轮目标值
  * @param  val		目标值
  *	@note 	无
  * @retval 无
  */
void Set_Speedtarget_R(float temp_val)
{
    pid_speed.target_val = temp_val;    // 设置当前右侧的目标值
}



float Speed_Pid_Cal(struct PID *pid,float Speed_Current)
{
    pid->error=pid->target_val-Speed_Current;

    if((pid->error>-Speed_Integral_Start_Err)&&(pid->error<Speed_Integral_Start_Err))//积分限幅
    {
        pid->integral+=pid->error;
        if(pid->integral<-250)
        {
            pid->integral=-250;
        }
        if(pid->integral>Speed_Integral_Max_Val)
        {
            pid->integral=Speed_Integral_Max_Val;
        }
    }
    pid->output_val=pid->Kp * pid->error +
                    pid->Ki * pid->integral +
                    pid->Kd * (pid->error-pid->err_last);
    pid->err_last=pid->error;
    return pid->output_val;
}



static ENCODER_RES motor_encoderL;
static ENCODER_RES motor_encoderR;
//编码器初始化
void encoder_init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
	
		//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
	NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

//获取编码器的值
int get_encoderL_count(void)
{
	return motor_encoderL.count;
}
//获取编码器的方向
ENCODER_DIR get_encoder_dir(void)
{
	return motor_encoderL.dir;
}

//编码器数据更新
//请间隔一定时间更新
void encoderL_update(void)
{
	motor_encoderL.count = motor_encoderL.temp_count;

	//确定方向
	motor_encoderL.dir = ( motor_encoderL.count >= 0 ) ? FORWARD : REVERSAL;

	motor_encoderL.temp_count = 0;//编码器计数值清零
}
//获取编码器的值
int get_encoderR_count(void)
{
	return motor_encoderR.count;
}
//获取编码器的方向
ENCODER_DIR get_encoderR_dir(void)
{
	return motor_encoderR.dir;
}

//编码器数据更新
//请间隔一定时间更新
void encoderR_update(void)
{
	motor_encoderR.count = motor_encoderR.temp_count;

	//确定方向
	motor_encoderR.dir = ( motor_encoderR.count >= 0 ) ? FORWARD : REVERSAL;

	motor_encoderR.temp_count = 0;//编码器计数值清零
}


//外部中断处理函数
void GROUP1_IRQHandler(void)
{
	uint32_t gpio_status;

	//获取中断信号情况
	gpio_status = DL_GPIO_getEnabledInterruptStatus(ENCODER_E1A_PORT, ENCODER_E1A_PIN | ENCODER_E1B_PIN);
	//编码器A相上升沿触发
	if((gpio_status & ENCODER_E1A_PIN) == ENCODER_E1A_PIN)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(ENCODER_E1A_PORT,ENCODER_E1B_PIN))
		{
			motor_encoderL.temp_count++;
		}
		else
		{
			motor_encoderL.temp_count--;
		}
	}//编码器B相上升沿触发
	else if((gpio_status & ENCODER_E1B_PIN)==ENCODER_E1B_PIN)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(ENCODER_E1A_PORT,ENCODER_E1A_PIN))
		{
			motor_encoderL.temp_count--;
		}
		else
		{
			motor_encoderL.temp_count++;
		}
	}
	//清除状态
	DL_GPIO_clearInterruptStatus(ENCODER_E1A_PORT,ENCODER_E1A_PIN|ENCODER_E1B_PIN);

  uint32_t gpio_status1;

	//获取中断信号情况
	gpio_status1 = DL_GPIO_getEnabledInterruptStatus(ENCODER_E2A_PORT, ENCODER_E2A_PIN | ENCODER_E2B_PIN);
	//编码器A相上升沿触发
	if((gpio_status1 & ENCODER_E2A_PIN) == ENCODER_E2A_PIN)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(ENCODER_E2A_PORT,ENCODER_E2B_PIN))
		{
			motor_encoderR.temp_count++;
		}
		else
		{
			motor_encoderR.temp_count--;
		}
	}//编码器B相上升沿触发
	else if((gpio_status1 & ENCODER_E2B_PIN)==ENCODER_E2B_PIN)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(ENCODER_E2A_PORT,ENCODER_E2A_PIN))
		{
			motor_encoderR.temp_count--;
		}
		else
		{
			motor_encoderR.temp_count++;
		}
	}
	//清除状态
	DL_GPIO_clearInterruptStatus(ENCODER_E2A_PORT,ENCODER_E2A_PIN|ENCODER_E2B_PIN);
}

void timer_init(void)
{
    //定时器中断
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
	DL_TimerA_startCounter(TIMER_0_INST);	 // 开始计时
}

//电机编码器脉冲计数
void TIMER_0_INST_IRQHandler(void)
{

	if( DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO )
	{
		//编码器更新
		encoderL_update();
		encoderR_update();
		
//		motor_encoderR.SPEED = ((motor_encoderR.count / 520) * (3.14 * 0.024)) / 0.01;
		motor_encoderR.SPEED = motor_encoderR.count * 0.0145 ;
		motor_encoderL.SPEED = motor_encoderL.count * 0.0145 ;
		printf("EncoderR = %f\n",motor_encoderR.SPEED);
		printf("EncoderL = %f\n",motor_encoderL.SPEED);
		

		
		

	}
}
