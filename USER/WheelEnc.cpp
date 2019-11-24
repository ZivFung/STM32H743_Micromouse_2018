#include "WheelEnc.h"




void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
		GPIO_InitTypeDef GPIO_Initure;
//		if(htim->Instance == TIM2){
			__HAL_RCC_TIM2_CLK_ENABLE();			//使能定时器1
			__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOB时钟
			GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1;           	//PB1
			GPIO_Initure.Mode=GPIO_MODE_AF_OD;  	//复用推完输出
			GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
			GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
			GPIO_Initure.Alternate=GPIO_AF1_TIM2;	//PB1复用为TIM3_CH4
			HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//		}
//		else if(htim->Instance == TIM4){
			__HAL_RCC_TIM4_CLK_ENABLE();			//使能定时器1
			__HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOB时钟
			GPIO_Initure.Pin=GPIO_PIN_12 | GPIO_PIN_13;           	//PB1
			GPIO_Initure.Mode=GPIO_MODE_AF_OD;  	//复用推完输出
			GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
			GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
			GPIO_Initure.Alternate=GPIO_AF2_TIM4;	//PB1复用为TIM3_CH4
			HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//		}
}
namespace MotorControl{
TIM_HandleTypeDef Encoder_TIM_Handler;      	//定时器句柄
TIM_Encoder_InitTypeDef TIM_Encoder_InitType;
//HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel)
//TIM1 &TIM5 Encoder部分初始化 
//arr：自动重装值
//psc：时钟预分频数
void WheelEnc::Encoder_TIM_Init(u16 arr,u16 psc)
{ 
	
		
    Encoder_TIM_Handler.Instance = TIM2;            //定时器3
    Encoder_TIM_Handler.Init.Prescaler = psc;       //定时器分频
    Encoder_TIM_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数模式
    Encoder_TIM_Handler.Init.Period = arr;          //自动重装载值
    Encoder_TIM_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
		TIM_Encoder_InitType.EncoderMode = TIM_ENCODERMODE_TI12;
		TIM_Encoder_InitType.IC1Filter = 0x05;
		TIM_Encoder_InitType.IC1Polarity = TIM_ICPOLARITY_RISING;
		TIM_Encoder_InitType.IC1Prescaler = TIM_ICPSC_DIV1;
		TIM_Encoder_InitType.IC1Selection = TIM_ICSELECTION_DIRECTTI;
		TIM_Encoder_InitType.IC2Filter = 0x05;
		TIM_Encoder_InitType.IC2Polarity = TIM_ICPOLARITY_RISING;
		TIM_Encoder_InitType.IC2Prescaler = TIM_ICPSC_DIV1;
		TIM_Encoder_InitType.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	
		HAL_TIM_Encoder_Init(&Encoder_TIM_Handler, &TIM_Encoder_InitType);
		HAL_TIM_Encoder_Start(&Encoder_TIM_Handler, TIM_CHANNEL_1);
		HAL_TIM_Encoder_Start(&Encoder_TIM_Handler, TIM_CHANNEL_2);
		Encoder_TIM_Handler.Instance = TIM4; 
		HAL_TIM_Encoder_Init(&Encoder_TIM_Handler, &TIM_Encoder_InitType);
		HAL_TIM_Encoder_Start(&Encoder_TIM_Handler, TIM_CHANNEL_1);
		HAL_TIM_Encoder_Start(&Encoder_TIM_Handler, TIM_CHANNEL_2);
	
}


WheelEnc::WheelEnc()
{
		WheelEncValueL.old_encoder = 0;
		WheelEncValueL.encoder = 0;
		WheelEncValueL.count = 0;
	
		WheelEncValueR.old_encoder = 0;
		WheelEncValueR.encoder = 0;
		WheelEncValueR.count = 0;
		EncVal = .0f;
    Encoder_TIM_Init(0xffff,0);
}

void WheelEnc::WheelEncGetVel()
{
		WheelEncValueL.old_encoder = WheelEncValueL.encoder;
		WheelEncValueL.encoder = (int16_t)TIM2->CNT;
		WheelEncValueL.count = WheelEncValueL.encoder - WheelEncValueL.old_encoder;
		if(WheelEncValueL.count < -32768)WheelEncValueL.count += 65536;
		else if(WheelEncValueL.count > 32768)WheelEncValueL.count -= 65536;
		WheelEncValueR.old_encoder = WheelEncValueR.encoder;
		WheelEncValueR.encoder = (int16_t)TIM4->CNT;
		WheelEncValueR.count = WheelEncValueR.encoder - WheelEncValueR.old_encoder;
		if(WheelEncValueR.count < -32768)WheelEncValueR.count += 65536;
		else if(WheelEncValueR.count > 32768)WheelEncValueR.count -= 65536;
		EncVal = ((float)WheelEncValueR.count + (float)WheelEncValueL.count) * PP::EncVelUnit * 0.5f;
//		EncVal = ((float)WheelEncValueR.count ) * PP::EncVelUnit * CP::EncoderUnitCompensation;
}

}
//---------------------------------------------------

