#include "ti_msp_dl_config.h"
#include "myi2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	  
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	MyI2C_Init();									//先初始化底层的I2C
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}


#define RtA (57.2957795f)  // 弧度转角度，180/π ≈ 57.2957795
#define Ki  0.005f         // 积分系数
#define DT  0.005f          // 计算周期的一半，单位s
 
// 加速度和角速度
int16_t AX, AY, AZ, GX, GY, GZ;
 
typedef struct {
    float c1, c2, c3, a1, a2, a3, a;
} Degree;
 
Degree d;
 
typedef struct {
    float q0, q1, q2, q3;
    float exInt, eyInt, ezInt;
} Quater;
 
Quater q = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
 
void Dmp_Init(void) {
    MPU6050_Init();
}
 
void MPU6050_Read(void) {
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    
    // 转换为物理量
    d.c1 = GX / 131.0f;  // 角速度（°/s）
    d.c2 = GY / 131.0f;
    d.c3 = GZ / 131.0f;
    
    // 转换为rad/s
    d.c1 /= RtA;
    d.c2 /= RtA;
    d.c3 /= RtA;
    
    d.a1 = AX / 16384.0f;  // 加速度（g）
    d.a2 = AY / 16384.0f;
    d.a3 = AZ / 16384.0f;
    
    // 加速度归一化
    float norm = sqrtf(d.a1 * d.a1 + d.a2 * d.a2 + d.a3 * d.a3);
    if (norm > 1e-6f) {  // 防止除零
        float inv_norm = 1.0f / norm;
        d.a1 *= inv_norm;
        d.a2 *= inv_norm;
        d.a3 *= inv_norm;
        d.a = 1.0f;
    } else {
        d.a = 0.0f;
    }
}
 
void ComputeEulerAngles(float *pitch, float *roll, float *yaw) {
    MPU6050_Read();
    
    // 计算姿态误差
    float gx = d.c1, gy = d.c2, gz = d.c3;
    float gravity_x = 2.0f * (q.q1 * q.q3 - q.q0 * q.q2);
    float gravity_y = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    float gravity_z = q.q0*q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3*q.q3;
    
    float error_x = d.a2 * gravity_z - d.a3 * gravity_y;
    float error_y = d.a3 * gravity_x - d.a1 * gravity_z;
    float error_z = d.a1 * gravity_y - d.a2 * gravity_x;
    
    // 更新四元数
    float Kp = 0.5f;
    
    // 误差积分
    q.exInt += Ki * error_x ;
    q.eyInt += Ki * error_y ;
    q.ezInt += Ki * error_z ;
    
    // 修正角速度
    gx += Kp * error_x + q.exInt;
    gy += Kp * error_y + q.eyInt;
    gz += Kp * error_z + q.ezInt;
    
    // 四元数微分方程
    float q0_dot = (-q.q1 * gx - q.q2 * gy - q.q3 * gz) * DT; 
    float q1_dot = (q.q0 * gx - q.q3 * gy + q.q2 * gz) * DT;
    float q2_dot = (q.q3 * gx + q.q0 * gy - q.q1 * gz) * DT;
    float q3_dot = (-q.q2 * gx + q.q1 * gy + q.q0 * gz) * DT;
    
    // 更新四元数
    q.q0 += q0_dot;
    q.q1 += q1_dot;
    q.q2 += q2_dot;
    q.q3 += q3_dot;
    
    // 归一化
    float norm = sqrtf(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        q.q0 *= inv_norm;
        q.q1 *= inv_norm;
        q.q2 *= inv_norm;
        q.q3 *= inv_norm;
    }
    
    // 计算欧拉角
    float q0q0 = q.q0 * q.q0;
    float q1q1 = q.q1 * q.q1;
    float q2q2 = q.q2 * q.q2;
    float q3q3 = q.q3 * q.q3;
    
    *roll = atan2f(2.0f * (q.q2 * q.q3 + q.q0 * q.q1), q0q0 - q1q1 - q2q2 + q3q3);
    *pitch = asinf(-2.0f * (q.q1 * q.q3 - q.q0 * q.q2));
    *yaw = atan2f(2.0f * (q.q1 * q.q2 + q.q0 * q.q3), q0q0 + q1q1 - q2q2 - q3q3);
    
//    // 转换为度
//    *roll *= RtA;
//    *pitch *= RtA;
//    *yaw *= RtA;
}
