/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-26     LCKFB     first version
 */
#ifndef	__BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

void board_init(void);

/* ��ʱ���� */
void delay_us(uint32_t __us);
void delay_ms(uint32_t __ms);

void delay_1us(uint32_t __us);
void delay_1ms(uint32_t __ms);

void uart0_send_char(char ch);
void uart0_send_string(char* str);



#endif
