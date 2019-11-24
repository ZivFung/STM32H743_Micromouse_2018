#ifndef _DELAY_H
#define _DELAY_H
#ifdef __cplusplus
extern  "C" {
#endif
#include <sys.h>	  
//////////////////////////////////////////////////////////////////////////////////  
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32H7������
//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й���(֧��ucosii)
//����delay_us,delay_ms
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/6/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
////////////////////////////////////////////////////////////////////////////////// 

void delay_init(u16 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#ifdef __cplusplus
}
#endif
#endif
