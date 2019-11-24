#ifndef _INFRARED_H_
#define _INFRARED_H_
#include "includes.h"
#include "usart.h"
#include "physparams.h"




// IR_FL -> PE2 L1 
// IR_SL -> PE3 L0
// IR_FR -> PC11 R0
// IR_SR -> PC10 R1
// IR_FR -> PC0 ADC3_INP10
// IR_SR -> PC1 ADC3_INP11
// IR_FL -> PC2 ADC3_INP12
// IR_SL -> PC3 ADC3_INP13
namespace InfraredControl{
#define IR_SL(n)		(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET))
#define IR_SL_Toggle (HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2)) 
#define IR_SR(n)		(n?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET))
#define IR_SR_Toggle (HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11)) 
#define IR_FL(n)		(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET))
#define IR_FL_Toggle (HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3)) 
#define IR_FR(n)		(n?HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET))
#define IR_FR_Toggle (HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10)) 
	
#define ADC_REF 3.0f
typedef struct{
		uint16_t FL;
		uint16_t FR;
		uint16_t SL;
		uint16_t SR;
		float dFL;
		float dFR;
		float dSL;
		float dSR;
		float YawL;
		float YawR;
		float YawFLR;
		float LFallStp;
		float RFallStp;
}IRValue;

enum IR_ADCChannel
{	SL_CH = ADC_CHANNEL_0,
	SR_CH = ADC_CHANNEL_10,
	FL_CH = ADC_CHANNEL_1,
	FR_CH = ADC_CHANNEL_11,
	VBAT_CH = ADC_CHANNEL_14
};

extern TIM_HandleTypeDef TIMIRTick_Handler;      	//¶¨Ê±Æ÷¾ä±ú

class Infrared{
	private:
		void IRADCInit(void);
		void IRGPIOInit(void);
		u16 GetIRAdc(u32 ch);
		void GetVBATAdc(void);
		void IRTickTimerInit(u16 arr,u16 psc);
	protected:
	public:
		Infrared();
		IRValue IRInts;
//		void IRPortsInit(void);
		void IRTimerStart(uint16_t xus);	
		void GetIRValue(void);
		void IRSelfTest();
		void IRDistSelfTest();
	
		inline float IrDistFwd(){
			return (IRInts.dFL + IRInts.dFR) * .5f;
		}
		
};
	


}


#endif
