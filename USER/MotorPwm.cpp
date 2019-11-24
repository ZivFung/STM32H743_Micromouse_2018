#include "MotorPwm.h"

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
    GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_TIM3_CLK_ENABLE();			//ʹ�ܶ�ʱ��3
    __HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOBʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_0 |GPIO_PIN_1;           	//PB1
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //����
		GPIO_Initure.Alternate=GPIO_AF2_TIM3;	//PB1����ΪTIM3_CH4
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	}
}
//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==TIM3)
//	{
//		__HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
//		HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
//		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�   
//	}  
//}
namespace MotorControl{
TIM_HandleTypeDef TIM3_Handler;      	//��ʱ�����
TIM_OC_InitTypeDef TIM3_CHHandler;     //��ʱ��3ͨ��1���
//TIM_OC_InitTypeDef TIM3_CH2Handler;     //��ʱ��3ͨ��2���
//TIM_OC_InitTypeDef TIM3_CH3Handler;     //��ʱ��3ͨ��3���	
//TIM_OC_InitTypeDef TIM3_CH4Handler;     //��ʱ��3ͨ��4���
//ͨ�ö�ʱ��3�жϳ�ʼ��,��ʱ��3��APB1�ϣ�APB1�Ķ�ʱ��ʱ��Ϊ200MHz
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!(��ʱ��3����APB1�ϣ�ʱ��ΪHCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //ͨ�ö�ʱ��3
    TIM3_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM3_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}

//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 arr,u16 psc)
{ 
    TIM3_Handler.Instance = TIM3;            //��ʱ��3
		TIM3_Handler.Init.Prescaler = psc;       //��ʱ����Ƶ
    TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM3_Handler.Init.Period = arr;          //�Զ���װ��ֵ
    TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM3_Handler);       //��ʼ��PWM
    
    TIM3_CHHandler.OCMode = TIM_OCMODE_PWM2; //ģʽѡ��PWM1
    TIM3_CHHandler.Pulse = (arr + 1)/2 - 1;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM3_CHHandler.OCPolarity = TIM_OCPOLARITY_LOW; //����Ƚϼ���Ϊ�� 
		TIM3_CHHandler.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_1);//����TIM3ͨ��1
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_2);//����TIM3ͨ��2
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_3);//����TIM3ͨ��3
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_4);//����TIM3ͨ��4
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_1);//����PWMͨ��1
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_2);//����PWMͨ��2
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_3);//����PWMͨ��3
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_4);//����PWMͨ��4
}

void MotorPwm::MotorPwmCoast()
{   // p - low, n - low
    // zero - fall
    // motor right

}

void MotorPwm::MotorPwmInit()
{
	TIM3_PWM_Init(1999,0);		//200MHz/2000
}

MotorPwm::MotorPwm()
{
    MotorDutyNow.l = 0;
    MotorDutyNow.r = 0;
    MotorPwmInit();
}

// r,l -> [-319, 319]
void MotorPwm::MotorPwmSetDuty(MotorDuty duty)
{
    MotorDutyNow = duty;
    if(MotorDutyNow.r < 0)
    {
			TIM3->CCR3 = (uint32_t)0x0;
			TIM3->CCR4 = (uint32_t)(-MotorDutyNow.r);
    }
    else
    {
			TIM3->CCR4 = (uint32_t)0x0;
			TIM3->CCR3 = (uint32_t)MotorDutyNow.r;
    }
    if(MotorDutyNow.l < 0)
    {
			TIM3->CCR1 = (uint32_t)(-MotorDutyNow.l);
			TIM3->CCR2 = (uint32_t)0x0;
    }
    else
    {
			TIM3->CCR2 = (uint32_t)MotorDutyNow.l;
			TIM3->CCR1 = (uint32_t)0x0;
    }
}

}

