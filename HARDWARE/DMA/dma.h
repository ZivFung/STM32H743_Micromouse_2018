#ifndef __DMA_H
#define __DMA_H
#ifdef __cplusplus
extern  "C" {
#endif
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//DMA��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA���

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx);
#ifdef __cplusplus
	}
#endif
#endif
