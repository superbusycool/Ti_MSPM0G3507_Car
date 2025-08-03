//#include "ti_msp_dl_config.h"
//#include "oled.h"
//#include "mpu6050.h"
//#include "ti/driverlib/dl_gpio.h"
//int16_t AX,AY,AZ,GX,GY,GZ;
//uint8_t id,i;
//int main(void)
//{
//    SYSCFG_DL_init();
//    OLED_Init();
//    OLED_CLS();
//    for(i=0;i<5;i++)
//    {
//        OLED_ShowChinese(1,i+1,12+i);
//    }
//    OLED_ShowChar(1,11,':');
//    MPU6050_Init();
//    //MPU6050_WriteReg(0x19,0x00);
//    id=MPU6050_GetID();
//    OLED_ShowNum(1,12,id,3);
//    while (1)
//    {
//        MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
//        OLED_ShowString(2,1,"AX:");
//		OLED_ShowSignedNum(2,4,AX,4);

//        OLED_ShowString(3,1,"AY:");
//		OLED_ShowSignedNum(3,4,AY,4);

//        OLED_ShowString(4,1,"AZ:");
//		OLED_ShowSignedNum(4,4,AZ,4);

//        OLED_ShowString(2,9,"GX:");
//		OLED_ShowSignedNum(2,12,AX,4);

//        OLED_ShowString(3,9,"GY:");
//		OLED_ShowSignedNum(3,12,GY,4);

//        OLED_ShowString(4,9,"GZ:");
//		OLED_ShowSignedNum(4,12,GZ,4);

//    }
//}
