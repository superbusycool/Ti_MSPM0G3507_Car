///*
// * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
// * 开发板官网：www.lckfb.com
// * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
// * 立创论坛：https://oshwhub.com/forum
// * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
// * 不靠卖板赚钱，以培养中国工程师为己任
// * Change Logs:
// * Date           Author       Notes
// * 2024-07-30     LCKFB        陀螺仪
// */

//#include "bsp_gyro.h"
//#include "stdio.h"
//#include "string.h"

//volatile Gyro_Struct Gyro_Structure;

//void jy61pInit(void)
//{
//	
//	/*================Z轴归零==================*/
//	
//	// 寄存器解锁
//	uint8_t unlock_reg1[2] = {0x88,0xB5};
//	writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg1,2);
//	delay_ms(200);
//	// Z轴归零
//	uint8_t z_axis_reg[2] = {0x04,0x00};
//	writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,z_axis_reg,2);	
//	delay_ms(200);	
//	// 保存
//	uint8_t save_reg1[2] = {0x00,0x00};
//	writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg1,2);
//	delay_ms(200);
//	
//	/*================角度归零==================*/
//	
//	// 寄存器解锁
//	uint8_t unlock_reg[2] = {0x88,0xB5};
//	writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg,2);
//	delay_ms(200);
//	// 角度归零
//	uint8_t angle_reg[2] = {0x08,0x00};
//	writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,angle_reg,2);	
//	delay_ms(200);
//	// 保存
//	uint8_t save_reg[2] = {0x00,0x00};
//	writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg,2);
//	delay_ms(200);
//		
//	// 清空结构体
//	memset(&Gyro_Structure,0,sizeof(Gyro_Structure));
//}



///******************************************************************
// * 函 数 名 称：IIC_Start
// * 函 数 说 明：IIC起始时序
// * 函 数 形 参：无
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//void IIC_Start(void)
//{
//        SDA_OUT();

//        SCL(0);
//        SDA(1);
//        SCL(1);

//        delay_us(5);

//        SDA(0);
//        delay_us(5);
//        SCL(0);
//        delay_us(5);


//}
///******************************************************************
// * 函 数 名 称：IIC_Stop
// * 函 数 说 明：IIC停止信号
// * 函 数 形 参：无
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//void IIC_Stop(void)
//{
//        SDA_OUT();
//        SCL(0);
//        SDA(0);

//        SCL(1);
//        delay_us(5);
//        SDA(1);
//        delay_us(5);

//}

///******************************************************************
// * 函 数 名 称：IIC_Send_Ack
// * 函 数 说 明：主机发送应答或者非应答信号
// * 函 数 形 参：0发送应答  1发送非应答
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//void IIC_Send_Ack(unsigned char ack)
//{
//        SDA_OUT();
//        SCL(0);
//        SDA(0);
//        delay_us(5);
//        if(!ack) SDA(0);
//        else     SDA(1);
//        SCL(1);
//        delay_us(5);
//        SCL(0);
//        SDA(1);
//}


///******************************************************************
// * 函 数 名 称：I2C_WaitAck
// * 函 数 说 明：等待从机应答
// * 函 数 形 参：无
// * 函 数 返 回：0有应答  1超时无应答
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//unsigned char I2C_WaitAck(void)
//{

//        char ack = 0;
//        char ack_flag = 50;
//	
//        SDA_IN();

//        SDA(1);
//        while( (SDA_GET()==1) && ( ack_flag ) )
//        {
//                ack_flag--;
//                delay_us(5);
//        }

//        if( ack_flag == 0 )
//        {
//                IIC_Stop();
//                return 1;
//        }
//        else
//        {
//				SCL(1);
//				delay_us(5);
//                SCL(0);
//                SDA_OUT();
//        }
//        return ack;
//}

///******************************************************************
// * 函 数 名 称：Send_Byte
// * 函 数 说 明：写入一个字节
// * 函 数 形 参：dat要写人的数据
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//void Send_Byte(uint8_t dat)
//{
//	int i = 0;
//	SDA_OUT();
//	SCL(0);//拉低时钟开始数据传输

//	for( i = 0; i < 8; i++ )
//	{
//		SDA( (dat & 0x80) >> 7 );
//		delay_us(2);
//	
//		SCL(1);
//		delay_us(5);
//	
//		SCL(0);
//		delay_us(5);
//	
//		dat<<=1;
//	}
//}

///******************************************************************
// * 函 数 名 称：Read_Byte
// * 函 数 说 明：IIC读时序
// * 函 数 形 参：无
// * 函 数 返 回：读到的数据
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//unsigned char Read_Byte(void)
//{
//    unsigned char i,receive=0;
//    SDA_IN();//SDA设置为输入
//    for(i=0;i<8;i++ )
//    {
//        SCL(0);
//        delay_us(5);
//        SCL(1);
//        delay_us(5);
//        receive<<=1;
//        if( SDA_GET() )
//        {
//            receive|=1;
//        }
//        delay_us(5);
//    }

//    return receive;
//}

///******************************************************************
// * 函 数 名 称：writeDataJy61p
// * 函 数 说 明：写数据
// * 函 数 形 参：dev 设备地址
//				reg 寄存器地址
//				data 数据首地址
//				length 数据长度
// * 函 数 返 回：返回0则写入成功
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
//{
//    uint32_t count = 0;
//	
//    IIC_Start();
//	
//    Send_Byte(dev<<1);	 
//    if(I2C_WaitAck() == 1)return 0;
//	
//    Send_Byte(reg);   
//    if(I2C_WaitAck() == 1)return 0;
//	
//    for(count=0; count<length; count++)
//    {
//        Send_Byte(data[count]);
//        if(I2C_WaitAck() == 1)return 0;
//    }
//	
//    IIC_Stop();

//    return 1; 
//}

///******************************************************************
// * 函 数 名 称：readDataJy61p
// * 函 数 说 明：读数据数据
// * 函 数 形 参：dev 设备地址
//				reg 寄存器地址
//				data 数据存储地址
//				length 数据长度
// * 函 数 返 回：返回0则写入成功
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
//{
//    uint32_t count = 0;

//    IIC_Start();
//	
//    Send_Byte((dev<<1)|0);	
//    if(I2C_WaitAck() == 1)return 0;
//	
//    Send_Byte(reg);
//    if(I2C_WaitAck() == 1)return 0;
//	
//	delay_us(5);
//	
//    IIC_Start();
//	
//    Send_Byte((dev<<1)|1); 
//    if(I2C_WaitAck() == 1)return 0;
//	
//    for(count=0; count<length; count++)
//    {
//        if(count!=length-1)
//		{
//			data[count]=Read_Byte();
//			IIC_Send_Ack(0);
//		}
//        else
//		{
//			data[count]=Read_Byte();
//			IIC_Send_Ack(1);
//		} 
//				 
//    }
//	
//    IIC_Stop();
//	
//    return 1; 
//}

///******************************************************************
// * 函 数 名 称：get_angle
// * 函 数 说 明：读角度数据
// * 函 数 形 参：无
// * 函 数 返 回：返回结构体
// * 作       者：LC
// * 备       注：无
//******************************************************************/
//float get_angle(void)
//{
//	// 数据缓存
//	volatile uint8_t sda_angle[6] = {0};
//	
//	int ret = 0;

//	// 清空数据缓存
//	memset(sda_angle,0,sizeof(sda_angle));
//	
//	// 读取寄存器数据
//	ret	= readDataJy61p(IIC_ADDR,0x3D,sda_angle,6);
//	
//	if(ret == 0)
//	{	
//		// 读取失败
//		printf("Read Error\r\n");
//	}
//	
//	#if GYRO_DEBUG
//	
//	printf("RollL = %x\r\n",sda_angle[0]);
//	printf("RollH = %x\r\n",sda_angle[1]);
//	printf("PitchL = %x\r\n",sda_angle[2]);
//	printf("PitchH = %x\r\n",sda_angle[3]);
//	printf("YawL = %x\r\n",sda_angle[4]);
//	printf("YawH = %x\r\n",sda_angle[5]);
//	
//	#endif
//	
//    // 计算 RollX, PitchY 和 YawZ 并确保它们在 -180 到 180 的范围内
//    float RollX = (float)(((sda_angle[1] << 8) | sda_angle[0]) / 32768.0 * 180.0);
//    if (RollX > 180.0)
//    {
//        RollX -= 360.0;
//    }
//    else if (RollX < -180.0)
//    {
//        RollX += 360.0;
//    }

//    float PitchY = (float)(((sda_angle[3] << 8) | sda_angle[2]) / 32768.0 * 180.0);
//    if (PitchY > 180.0)
//    {
//        PitchY -= 360.0;
//    }
//    else if (PitchY < -180.0)
//    {
//        PitchY += 360.0;
//    }

//    float YawZ = (float)(((sda_angle[5] << 8) | sda_angle[4]) / 32768.0 * 180.0);
//    if (YawZ > 180.0)
//    {
//        YawZ -= 360.0;
//    }
//    else if (YawZ < -180.0)
//    {
//        YawZ += 360.0;
//    }

//    // 将计算结果保存到结构体中
//    Gyro_Structure.x = RollX;
//    Gyro_Structure.y = PitchY;
//    Gyro_Structure.z = YawZ;
//	
//	// 返回角度数据
//	return YawZ;
//}

