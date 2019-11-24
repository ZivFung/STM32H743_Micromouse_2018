#include "MotorPwm.h"

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
    GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_TIM3_CLK_ENABLE();			//使能定时器3
    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_0 |GPIO_PIN_1;           	//PB1
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推完输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
		GPIO_Initure.Alternate=GPIO_AF2_TIM3;	//PB1复用为TIM3_CH4
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	}
}
//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance==TIM3)
//	{
//		__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
//		HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
//		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断   
//	}  
//}
namespace MotorControl{
TIM_HandleTypeDef TIM3_Handler;      	//定时器句柄
TIM_OC_InitTypeDef TIM3_CHHandler;     //定时器3通道1句柄
//TIM_OC_InitTypeDef TIM3_CH2Handler;     //定时器3通道2句柄
//TIM_OC_InitTypeDef TIM3_CH3Handler;     //定时器3通道3句柄	
//TIM_OC_InitTypeDef TIM3_CH4Handler;     //定时器3通道4句柄
//通用定时器3中断初始化,定时器3在APB1上，APB1的定时器时钟为200MHz
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE    
}

//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{ 
    TIM3_Handler.Instance = TIM3;            //定时器3
		TIM3_Handler.Init.Prescaler = psc;       //定时器分频
    TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数模式
    TIM3_Handler.Init.Period = arr;          //自动重装载值
    TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM3_Handler);       //初始化PWM
    
    TIM3_CHHandler.OCMode = TIM_OCMODE_PWM2; //模式选择PWM1
    TIM3_CHHandler.Pulse = (arr + 1)/2 - 1;            //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM3_CHHandler.OCPolarity = TIM_OCPOLARITY_LOW; //输出比较极性为低 
		TIM3_CHHandler.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_1);//配置TIM3通道1
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_2);//配置TIM3通道2
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_3);//配置TIM3通道3
		HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CHHandler,TIM_CHANNEL_4);//配置TIM3通道4
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_1);//开启PWM通道1
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_2);//开启PWM通道2
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_3);//开启PWM通道3
		HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_4);//开启PWM通道4
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

