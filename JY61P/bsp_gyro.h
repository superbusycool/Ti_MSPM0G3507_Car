///*
// * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
// * �����������www.lckfb.com
// * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
// * ������̳��https://oshwhub.com/forum
// * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
// * ��������׬Ǯ���������й�����ʦΪ����
// * Change Logs:
// * Date           Author       Notes
// * 2024-07-30     LCKFB        ������
// */
//#ifndef	__BSP_GYRO_H__
//#define __BSP_GYRO_H__

//#include "board.h"

//// ���Կ���
//#define GYRO_DEBUG	0

//// ����һ���ṹ�����洢
//typedef struct {
//    float x;
//    float y;
//    float z;
//} Gyro_Struct;

//extern volatile Gyro_Struct Gyro_Structure;

//// ģ���ַ
//#define	IIC_ADDR		0x50
//// ����ǵ�ַ
//#define YAW_REG_ADDR	0x3F	
//// �Ĵ�������
//#define UN_REG			0x69
//// ����Ĵ���
//#define SAVE_REG		0x00
//// �ǶȲο��Ĵ���
//#define ANGLE_REFER_REG	0x01

////����SDA���ģʽ
//#define SDA_OUT()   {                                                  \
//                        DL_GPIO_initDigitalOutput(GPIO_SDA_IOMUX);     \
//                        DL_GPIO_setPins(GPIO_PORT, GPIO_SDA_PIN);      \
//                        DL_GPIO_enableOutput(GPIO_PORT, GPIO_SDA_PIN); \
//                    }
////����SDA����ģʽ
//#define SDA_IN()    { DL_GPIO_initDigitalInput(GPIO_SDA_IOMUX); }

////��ȡSDA���ŵĵ�ƽ�仯
//#define SDA_GET()   ( ( ( DL_GPIO_readPins(GPIO_PORT,GPIO_SDA_PIN) & GPIO_SDA_PIN ) > 0 ) ? 1 : 0 )

////SDA��SCL���
//#define SDA(x)      ( (x) ? (DL_GPIO_setPins(GPIO_PORT,GPIO_SDA_PIN)) : (DL_GPIO_clearPins(GPIO_PORT,GPIO_SDA_PIN)) )
//#define SCL(x)      ( (x) ? (DL_GPIO_setPins(GPIO_PORT,GPIO_SCL_PIN)) : (DL_GPIO_clearPins(GPIO_PORT,GPIO_SCL_PIN)) )

//void jy61pInit(void);
//uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
//uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
//float get_angle(void);

//#endif
