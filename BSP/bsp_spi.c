#include "bsp_spi.h"

uint8_t spi_read_write_byte(uint8_t dat)
{
        uint8_t data = 0;
        
        //��������
        DL_SPI_transmitData8(SPI_INST,dat);
        //�ȴ�SPI���߿���
        while(DL_SPI_isBusy(SPI_INST));
        //��������
        data = DL_SPI_receiveData8(SPI_INST);
        //�ȴ�SPI���߿���
        while(DL_SPI_isBusy(SPI_INST));
        
        return data;
}
































