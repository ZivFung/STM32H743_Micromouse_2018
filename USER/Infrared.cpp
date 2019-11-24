#include "Infrared.h"
__attribute__((section("ccmram"))) float BATV = 4.168f;
extern OS_SEM MotorTick;
extern InfraredControl::Infrared *IR;
extern OS_ERR err;


//ADC�ײ��������������ã�ʱ��ʹ��
//�˺����ᱻHAL_ADC_Init()����
//hadc:ADC���
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();           //ʹ��ADC3ʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE();			//����GPIOCʱ��
		__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //ADC����ʱ��ѡ��
		__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;            //PC0123
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //ģ��
    GPIO_Initure.Pull=GPIO_NOPULL;          //����������
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
		GPIO_Initure.Pin = GPIO_PIN_2;
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}
//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();            //ʹ��TIM3ʱ��
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //����ITM3�ж�   
	}  
}

namespace InfraredControl{
OS_SEM IRCHTick;
ADC_HandleTypeDef IR_ADC_Handler;
ADC_HandleTypeDef VBAT_ADC_Handler;
ADC_ChannelConfTypeDef IR_ADC_ChanConf;
ADC_ChannelConfTypeDef VBAT_ADC_ChanConf;
TIM_HandleTypeDef TIMIRTick_Handler;


//static uint8_t IRScanCnts = 0;

void Infrared::IRGPIOInit(void)
{
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_Initure;
    GPIO_Initure.Pin=GPIO_PIN_10 | GPIO_PIN_11;            //IR_R0,1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
    GPIO_Initure.Pull=GPIO_PULLUP;          //����������
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
		GPIO_Initure.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		HAL_GPIO_Init(GPIOE,&GPIO_Initure);
		
}

void Infrared::IRADCInit(void)
{
    IR_ADC_Handler.Instance=ADC3;
    IR_ADC_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4��Ƶ��ADCCLK=PER_CK/4=64/4=16MHZ
    IR_ADC_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16λģʽ
    IR_ADC_Handler.Init.ScanConvMode=DISABLE;                    	//��ɨ��ģʽ
    IR_ADC_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//�ر�EOC�ж�
		IR_ADC_Handler.Init.LowPowerAutoWait=DISABLE;					//�Զ��͹��Ĺر�				
    IR_ADC_Handler.Init.ContinuousConvMode=DISABLE;               //�ر�����ת��
    IR_ADC_Handler.Init.NbrOfConversion=1;                        //1��ת���ڹ��������� Ҳ����ֻת����������1 
    IR_ADC_Handler.Init.DiscontinuousConvMode=DISABLE;            //��ֹ����������ģʽ
    IR_ADC_Handler.Init.NbrOfDiscConversion=0;                    //����������ͨ����Ϊ0
    IR_ADC_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //�������
    IR_ADC_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//ʹ���������
		IR_ADC_Handler.Init.BoostMode=DISABLE;							//BOOTģʽ�ر�
		IR_ADC_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			//���µ����ݵ�����ֱ�Ӹ��ǵ�������
		IR_ADC_Handler.Init.OversamplingMode=DISABLE;					//�������ر�
		IR_ADC_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DR;  //����ͨ�������ݽ���������DR�Ĵ�������
		IR_ADC_Handler.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    HAL_ADC_Init(&IR_ADC_Handler);                                 //��ʼ�� 
		HAL_ADCEx_Calibration_Start(&IR_ADC_Handler,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED); //ADCУ׼

    VBAT_ADC_Handler.Instance=ADC1;
    VBAT_ADC_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4��Ƶ��ADCCLK=PER_CK/4=64/4=16MHZ
    VBAT_ADC_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16λģʽ
    VBAT_ADC_Handler.Init.ScanConvMode=DISABLE;                    	//��ɨ��ģʽ
    VBAT_ADC_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//�ر�EOC�ж�
		VBAT_ADC_Handler.Init.LowPowerAutoWait=DISABLE;					//�Զ��͹��Ĺر�				
    VBAT_ADC_Handler.Init.ContinuousConvMode=DISABLE;               //�ر�����ת��
    VBAT_ADC_Handler.Init.NbrOfConversion=1;                        //1��ת���ڹ��������� Ҳ����ֻת����������1 
    VBAT_ADC_Handler.Init.DiscontinuousConvMode=DISABLE;            //��ֹ����������ģʽ
    VBAT_ADC_Handler.Init.NbrOfDiscConversion=1;                    //����������ͨ����Ϊ0
    VBAT_ADC_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //�������
    VBAT_ADC_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//ʹ���������
		VBAT_ADC_Handler.Init.BoostMode=DISABLE;							//BOOTģʽ�ر�
		VBAT_ADC_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			//���µ����ݵ�����ֱ�Ӹ��ǵ�������
		VBAT_ADC_Handler.Init.OversamplingMode=DISABLE;					//�������ر�
		VBAT_ADC_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DR;  //����ͨ�������ݽ���������DR�Ĵ�������
		VBAT_ADC_Handler.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    HAL_ADC_Init(&VBAT_ADC_Handler);                                 //��ʼ�� 
		HAL_ADCEx_Calibration_Start(&VBAT_ADC_Handler,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED); //ADCУ׼
		
		VBAT_ADC_ChanConf.Channel = VBAT_CH;
		VBAT_ADC_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1������
    VBAT_ADC_ChanConf.SamplingTime=ADC_SAMPLETIME_387CYCLES_5;      	//����ʱ��       
		VBAT_ADC_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
		VBAT_ADC_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
		VBAT_ADC_ChanConf.Offset=0; 
		HAL_ADC_ConfigChannel(&VBAT_ADC_Handler,&VBAT_ADC_ChanConf);    //ͨ������

		
    IR_ADC_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1������
    IR_ADC_ChanConf.SamplingTime=ADC_SAMPLETIME_387CYCLES_5;      	//����ʱ��       
		IR_ADC_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
		IR_ADC_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
		IR_ADC_ChanConf.Offset=0;   
		

}

//���ADCֵ
//ch: ͨ��ֵ 0~16��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
//����ֵ:ת�����
u16 Infrared::GetIRAdc(u32 ch)   
{	
		IR_ADC_ChanConf.Channel=ch;                                   //ͨ��
	  HAL_ADC_ConfigChannel(&IR_ADC_Handler,&IR_ADC_ChanConf);      //ͨ������
    HAL_ADC_Start(&IR_ADC_Handler);                               //����ADC
    HAL_ADC_PollForConversion(&IR_ADC_Handler,10);                //��ѯת��
		return (u16)HAL_ADC_GetValue(&IR_ADC_Handler);	            //�������һ��ADC1�������ת�����
}

void Infrared::GetVBATAdc(void)
{
		HAL_ADC_Start(&VBAT_ADC_Handler);                               //����ADC
    HAL_ADC_PollForConversion(&VBAT_ADC_Handler,20);                //��ѯת��
		u16 test = (u16)HAL_ADC_GetValue(&VBAT_ADC_Handler);
		BATV = test / 65536.f * ADC_REF + BATV * 0.5f;
}

void Infrared::IRTickTimerInit(u16 arr,u16 psc)
{
    TIMIRTick_Handler.Instance=TIM6;                          //ͨ�ö�ʱ��6
    TIMIRTick_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIMIRTick_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIMIRTick_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
		TIMIRTick_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		
    TIMIRTick_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
		HAL_TIM_Base_Init(&TIMIRTick_Handler);
//		HAL_TIM_Base_Start_IT(&TIMIRTick_Handler);
    HAL_TIM_OnePulse_Init(&TIMIRTick_Handler,TIM_OPMODE_SINGLE);
    __HAL_TIM_ENABLE_IT(&TIMIRTick_Handler, TIM_IT_UPDATE);
//    HAL_TIM_OnePulse_Start_IT(&TIMIRTick_Handler,TIM_CHANNEL_1); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE  
}

void Infrared::IRTimerStart(uint16_t xus)
{
		TIM6->ARR=xus-1;  	//??????????
		TIM6->CR1|=0x01;    //?????6
}

Infrared::Infrared()
{
	 
//	OSSemCreate(&IRTick,(char*)"IRTick",0,&err);
	OSSemCreate(&IRCHTick,(char*)"IRCHTick",0,&err);
	IR_FL(0);
	IR_FR(0);
	IR_SL(0);
	IR_SR(0);
	IRTickTimerInit(20,20);		//500k
	IRGPIOInit();
	IRADCInit();
	OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
}
void Infrared::GetIRValue(void)
{
			IR_FL(1);
			IRTimerStart(70);
			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IRInts.FL = IR->GetIRAdc(FL_CH);
			IR_FL(0);
//			IRTimerStart(10);
//			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IR_FR(1);
			IRTimerStart(70);
			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IRInts.FR = IR->GetIRAdc(FR_CH);
			IR_FR(0);
//			IRTimerStart(10);
//			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IR_SL(1);
			IRTimerStart(50);
			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IRInts.SL = IR->GetIRAdc(SL_CH);
			IR_SL(0);
//			IRTimerStart(10);
//			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
			IR_SR(1);
			IRTimerStart(50);
			OSSemPend(&IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			IRInts.SR = IR->GetIRAdc(SR_CH);
			IR_SR(0);
			IR->GetVBATAdc();
}

void Infrared::IRSelfTest(){
	OS_ERR  err;
	while(1){
		OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
//	printf("IR FR ADC Value: %d \r\n",IR->IRInts.FR);
//		printf("%d\r\n",IR->IRInts.FR);
//		printf("%d\t%d\r\n",IR->IRInts.FL,IR->IRInts.FR);
		printf("%d\t%d\t%d\t%d\t\r\n",IR->IRInts.FL,IR->IRInts.FR,IR->IRInts.SL,IR->IRInts.SR);
//	printf("IR FL ADC Value: %d \r\n",IR->IRInts.FL);
//	printf("IR SR ADC Value: %d \r\n",IR->IRInts.SR);
//	printf("IR SL ADC Value: %d \r\n",IR->IRInts.SL);
//	printf("VBAT Value: %f \r\n",BATV);
	}
}




}




//��ʱ��3�жϷ���������
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
			OSSemPost(&InfraredControl::IRCHTick,OS_OPT_POST_ALL,&err);
		}
		else if(htim->Instance == TIM7)
    {
//			OSSemPost(&MotorTick,OS_OPT_POST_ALL,&err);
			
//			OSSemPost(&MotorTick,OS_OPT_POST_ALL,&err);
		}
}

extern "C" void TIM6_DAC_IRQHandler(void)
{
	#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
	#endif
  HAL_TIM_IRQHandler(&InfraredControl::TIMIRTick_Handler);
	
	#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
	#endif
}