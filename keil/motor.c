#include <stdio.h>
#include "ti_msp_dl_config.h"


void Motor_Forward(){
	
	DL_GPIO_setPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左侧
	

	
	DL_GPIO_setPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);//右
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);
	
}

void Motor_Back(){
	
	DL_GPIO_setPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);
	

	
	DL_GPIO_setPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);//右侧
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);
	
	
}

void Motor_Stop(){

	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);
	
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);//右
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);
	
	
}

void Motor_FL(){
	
	DL_GPIO_setPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左侧
	
}



void Motor_BR(){//FR改成BR
	
	DL_GPIO_setPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);//右侧
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);

}



void Motor_BL(){

	DL_GPIO_setPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);

}



void Motor_FR(){//BR改成FR

	DL_GPIO_setPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);//右
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);
	
}


void Motor_SL(){

	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain1_PIN);//左
	DL_GPIO_clearPins(GPIO_GRP_A_PORT,GPIO_GRP_A_PIN_Ain2_PIN);
	
}


void Motor_SR(){

	DL_GPIO_setPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin1_PIN);//右
	DL_GPIO_clearPins(GPIO_GRP_B_PORT,GPIO_GRP_B_PIN_Bin2_PIN);
	
}


#define Speed_Integral_Start_Err 100
#define Speed_Integral_Max_Val 50


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

    DL_TimerG_setCaptureCompareValue(PWM_AB_INST, 2000-pwm, GPIO_PWM_AB_C0_IDX);
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
    if(pwm > 0)
    {
        set_pwmL(pwm);
        Motor_FL();
    }
    else if(pwm < 0)
    {
        set_pwmL(-pwm);
        Motor_BL();
    }
    else
    {
        set_pwmL(0);
        Motor_SL();
    }
}

//通过PWM以及正负号控制右侧两个电机转动的方向和速度
void set_motor_rotateR(int pwm)
{
    if(pwm > 0)
    {
        set_pwmR(pwm);
        Motor_FR();
    }
    else if(pwm < 0)
    {
        set_pwmR(-pwm);
        Motor_BR();
    }
    else
    {
        set_pwmR(0);
        Motor_SR();
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

int gpioA=0;
int gEncoderCountA=0;

//void GROUPEncoderA_IRQHandler(void)
//{
//    //获取中断信号
//    gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,
//    GPIO_GRP_EncoderA_PIN_AL_PIN  | GPIO_GRP_EncoderA_PIN_BL_PIN );
// 
//    //如果是GPIO_EncoderA_PIN_0_PIN产生的中断
//    if((gpioA & GPIO_GRP_EncoderA_PIN_AL_PIN) == GPIO_GRP_EncoderA_PIN_AL_PIN)
//    {
//        //Pin0上升沿，看Pin1的电平，为低电平则判断为反转，高电平判断为正转
//        if(!DL_GPIO_readPins(GPIOA,GPIO_GRP_EncoderA_PIN_BL_PIN))//P1为低电平
//        {
//            gEncoderCountA--;
//        }
//        else//P1为高电平
//        {
//            gEncoderCountA++;
//        }
//    }
//    
//    //类似于Stm32中编码器模式的AB两相都测，可得到2倍的计数
//    else if((gpioA & GPIO_GRP_EncoderA_PIN_BL_PIN ) == GPIO_GRP_EncoderA_PIN_BL_PIN )
//    {
//        //Pin1上升沿
//        if(!DL_GPIO_readPins(GPIOA,GPIO_GRP_EncoderA_PIN_AL_PIN))//P0为低电平
//        {
//            gEncoderCountA++;
//        }
//        else//P1为高电平
//        {
//            gEncoderCountA--;
//        }
//    }
//    
//    //最后清除中断标志位
//    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_EncoderA_PIN_AL_PIN|GPIO_GRP_EncoderA_PIN_BL_PIN);
//}

//int gpioB=0;
//int gEncoderCountB=0;
//float Speed_L = 0;
//float Speed_R = 0;

//void GROUPEncoderB_IRQHandler(void)
//{
//    //获取中断信号
//    gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,
//    GPIO_GRP_EncoderB_PIN_AR_PIN | GPIO_GRP_EncoderB_PIN_BR_PIN );
// 
//    //如果是GPIO_EncoderA_PIN_0_PIN产生的中断
//    if((gpioA & GPIO_GRP_EncoderB_PIN_AR_PIN) == GPIO_GRP_EncoderB_PIN_AR_PIN)
//    {
//        //Pin0上升沿，看Pin1的电平，为低电平则判断为反转，高电平判断为正转
//        if(!DL_GPIO_readPins(GPIOA,GPIO_GRP_EncoderB_PIN_BR_PIN))//P1为低电平
//        {
//            gEncoderCountB--;
//        }
//        else//P1为高电平
//        {
//            gEncoderCountB++;
//        }
//    }
//    
//    //类似于Stm32中编码器模式的AB两相都测，可得到2倍的计数
//    else if((gpioA & GPIO_GRP_EncoderB_PIN_BR_PIN) == GPIO_GRP_EncoderB_PIN_BR_PIN)
//    {
//        //Pin1上升沿
//        if(!DL_GPIO_readPins(GPIOA,GPIO_GRP_EncoderB_PIN_AR_PIN))//P0为低电平
//        {
//            gEncoderCountB++;
//        }
//        else//P1为高电平
//        {
//            gEncoderCountB--;
//        }
//    }
//    
//    //最后清除中断标志位
//    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_EncoderB_PIN_AR_PIN|GPIO_GRP_EncoderB_PIN_BR_PIN);
//}


///**
// * @函数介绍: 定时器10ms中断
// * @输入参数: 无
// * @输出参数: 无
// * @说明: 每1ms进一次中断，请勿在该函数中添加delay等死循环函数
// *        无需将该函数放在main函数中
// */
//void TIMER_10MS_INST_IRQHandler(void)
//{
//	/* 这里可以放置用户的函数 */
//	Speed_L = (gEncoderCountA*0.048f*3.14f)/(10.4f);
//	Speed_R = (gEncoderCountB*0.048f*3.14f)/(10.4f);
//	
	
	
	
	
	

