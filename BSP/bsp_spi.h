#ifndef _BSP_SPI_H__
#define _BSP_SPI_H__

#include "board.h"

//CS���ŵ�������� 
//x=0ʱ����͵�ƽ
//x=1ʱ����ߵ�ƽ
#define SPI_CS(x)  ( (x) ? DL_GPIO_setPins(CS_PORT,CS_PIN_PIN) : DL_GPIO_clearPins(CS_PORT,CS_PIN_PIN) )
uint8_t spi_read_write_byte(uint8_t dat);
#endif
