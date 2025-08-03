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
//#ifndef	__BSP_GYRO_H__
//#define __BSP_GYRO_H__

//#include "board.h"

//// 调试开关
//#define GYRO_DEBUG	0

//// 定义一个结构体来存储
//typedef struct {
//    float x;
//    float y;
//    float z;
//} Gyro_Struct;

//extern volatile Gyro_Struct Gyro_Structure;

//// 模块地址
//#define	IIC_ADDR		0x50
//// 航向角地址
//#define YAW_REG_ADDR	0x3F	
//// 寄存器解锁
//#define UN_REG			0x69
//// 保存寄存器
//#define SAVE_REG		0x00
//// 角度参考寄存器
//#define ANGLE_REFER_REG	0x01

////设置SDA输出模式
//#define SDA_OUT()   {                                                  \
//                        DL_GPIO_initDigitalOutput(GPIO_SDA_IOMUX);     \
//                        DL_GPIO_setPins(GPIO_PORT, GPIO_SDA_PIN);      \
//                        DL_GPIO_enableOutput(GPIO_PORT, GPIO_SDA_PIN); \
//                    }
////设置SDA输入模式
//#define SDA_IN()    { DL_GPIO_initDigitalInput(GPIO_SDA_IOMUX); }

////获取SDA引脚的电平变化
//#define SDA_GET()   ( ( ( DL_GPIO_readPins(GPIO_PORT,GPIO_SDA_PIN) & GPIO_SDA_PIN ) > 0 ) ? 1 : 0 )

////SDA与SCL输出
//#define SDA(x)      ( (x) ? (DL_GPIO_setPins(GPIO_PORT,GPIO_SDA_PIN)) : (DL_GPIO_clearPins(GPIO_PORT,GPIO_SDA_PIN)) )
//#define SCL(x)      ( (x) ? (DL_GPIO_setPins(GPIO_PORT,GPIO_SCL_PIN)) : (DL_GPIO_clearPins(GPIO_PORT,GPIO_SCL_PIN)) )

//void jy61pInit(void);
//uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
//uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
//float get_angle(void);

//#endif
