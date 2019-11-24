#include "Motor.h"
extern OS_SEM MotorTick;
extern OS_ERR err;
extern float BATV;
//uint8_t crashDet = 0;

#define Test_Toggle (HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6)) 

void TestGpioInit()
{
		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_Initure;
    GPIO_Initure.Pin=GPIO_PIN_6;            //IR_R0,1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
    GPIO_Initure.Pull=GPIO_PULLUP;          //不带上下拉
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
}
//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM7)
	{
		__HAL_RCC_TIM7_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM7_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM7_IRQn);          //开启ITM3中断   
	}  
	  else if(htim->Instance==TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //开启ITM3中断   
	}  
		else if(htim->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM3中断   
	}  
}

namespace MotorControl{

TIM_HandleTypeDef TIMMotorTick_Handler;      	//定时器句柄
volatile MotorSpeedDef MotorSpeedArray[MotorSpeedQueueLength];
void MotorCtl::MortorTickTimerInit(u16 arr,u16 psc)
{
    TIMMotorTick_Handler.Instance=TIM7;                          //通用定时器3
    TIMMotorTick_Handler.Init.Prescaler=psc;                     //分频
    TIMMotorTick_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIMMotorTick_Handler.Init.Period=arr;                        //自动装载值
    TIMMotorTick_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIMMotorTick_Handler);
    
    HAL_TIM_Base_Start_IT(&TIMMotorTick_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE  
}
	uint8_t stopcnt = 0;
MotorCtl::MotorCtl()
{
		GyroZZero = 0.f;
		diff = 0.0f;
		AccelYZero = 0.f;
		LvAdj = 0.f;
		OmgAdj = 0.f;
		dOmgAdj = 0.f;
		QueueEmptyOp = 0;
    MotorDesireSpeed = new Queue((uint8_t *)MotorSpeedArray, MotorSpeedQueueLength, sizeof(MotorSpeedDef));
    Imu = new ICM20602();
    WheelEncoder = new WheelEnc();
    MPWM = new MotorPwm();
    LvPid = new Pid(lvPidP, lvPidI, lvPidD, lvPidN, PP::Ts, -1999.f, 1999.f);
    AvPid = new Pid(avPidP, avPidI, avPidD, avPidN, PP::Ts, -1999.f, 1999.f);
    lvEstKal = new Kalman1Var(
            1.0f,   // A
						PP::Ts, // B
            1.0f,   // H
            //                  rms noise         quantify noise            drift?  enlarge to anti drift
//            PP::Ts * PP::Ts * (1.57f * AcclFc * AcclPacc * AcclPacc + 25.f*Accla0* Accla0),   // Q
						PP::Ts * PP::Ts * (0.00307745f + PP::AcclUnit * PP::AcclUnit / 12.f + 0.117649f*1128.f), //Q
						PP::EncVelVariance * .5f,   // R
            0.f,    // x0
            0.f     // P0
            );

    MMode = Idle;
		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
//		TestGpioInit();
		MortorTickTimerInit(199,999);		//200M / 10; 20M / 20000; 1ms
		OSSemCreate(&MotorTick,(char*)"MotorTick",0,&err);
}

int staticCnt = 0, staticCntCyced;
int32_t gyroZZeroAcc = 0;
int32_t acclYZeroAcc = 0;
void MotorCtl::MotorRunTick()
{
    WheelEncoder->WheelEncGetVel();
//		printf("Enc Right Count: %d\r\n",WheelEncoder->WheelEncValueR.count);
//		printf("Enc Left Count: %d\r\n",WheelEncoder->WheelEncValueL.count);
//		printf("Enc Val: %f\r\n",WheelEncoder->EncVal);
//		if(WheelEncoder->WheelEncValueR.count == 0 && WheelEncoder->WheelEncValueL.count == 0)stopcnt++;
//		if(stopcnt > 15){MMode = Cal;stopcnt = 0;}
    crashDet = Imu->ImuReadData();
		NowSpeed.Av = (Imu->IMU_Data.GYRO_Z * PP::GyroUnit - GyroZZero);
//		printf("Av:%f\r\n",NowSpeed.Av);
    Accl = (Imu->IMU_Data.ACCEL_Y * PP::AcclUnit - AccelYZero);
//		printf("AcclY: %f\r\n",Accl);
    lvEstKal->Predict(Accl);
    NowSpeed.Lv = lvEstKal->Correct(WheelEncoder->EncVal);
//		printf("Lv:%f\r\n",NowSpeed.Lv);
    DistanceAcc += NowSpeed.Lv * PP::Ts;
    AngleAcc += NowSpeed.Av * PP::Ts;
#ifdef ANGULAR_VELOCITY_PID
    if(((dOmgAdj - OmgAdj) > -.003f)&&((dOmgAdj - OmgAdj) < .003f))dOmgAdj=OmgAdj;
    else if(dOmgAdj < OmgAdj)dOmgAdj+=.0026f;
    else if(dOmgAdj > OmgAdj)dOmgAdj-=.0028f;
    MotorDesireSpeed->Pop((uint8_t *)&DesireSpeed);
    desireAngle += ( + DesireSpeed.Av) * .001f;
    lvPidOut = LvPid->Tick(LvAdj + DesireSpeed.Lv - NowSpeed.Lv);
    avPidOut = AvPid->Tick(dOmgAdj + desireAngle - AngleAcc);
#else
		LastDesireSpeed.Av = DesireSpeed.Av;
		LastDesireSpeed.Lv = DesireSpeed.Lv;
		if(!MotorDesireSpeed->Pop((uint8_t *)&DesireSpeed))
	{
		if(QueueEmptyOp == 0){
			DesireSpeed.Lv = LastDesireSpeed.Lv;
			DesireSpeed.Av = LastDesireSpeed.Av;
			MotorDesireSpeed->Clear();
		}
		else {
			DesireSpeed.Lv = 0;
			DesireSpeed.Av = 0;
			MotorDesireSpeed->Clear();
		}
	}
//		printf("Desire Lv Speed: %f\r\n",DesireSpeed.Lv);
//		printf("Desire Av Speed: %f\r\n",DesireSpeed.Av);
//    desireAngle += ( + DesireSpeed.Av) * .001f;
//		diff = LvAdj + DesireSpeed.Lv - NowSpeed.Lv;
    lvPidOut = LvPid->Tick(LvAdj + DesireSpeed.Lv - NowSpeed.Lv);
//		printf("lvPid:%f\r\n",lvPidOut);
    avPidOut = AvPid->Tick(dOmgAdj + DesireSpeed.Av - NowSpeed.Av);
//		printf("avPid:%f\r\n",avPidOut);
#endif 
    rPwm = (lvPidOut - avPidOut)*(4.168f/BATV);
    lPwm = (lvPidOut + avPidOut)*(4.168f/BATV);
    PwmDuty.r = (int)saturate(rPwm, 1999.f, -1999.f);
    PwmDuty.l = (int)saturate(lPwm, 1999.f, -1999.f);
//		printf("rpwm:%d\r\n",PwmDuty.r);
//		printf("lpwm:%d\r\n",PwmDuty.l);
    MPWM->MotorPwmSetDuty(PwmDuty);
        if(WheelEncoder->WheelEncValueL.count == 0 && WheelEncoder->WheelEncValueR.count == 0)
        {
            staticCnt++;
            staticCntCyced = (staticCnt & 0x3FFF);  // cyced per 16.384s(@Ts=1ms)
            if(staticCntCyced == 255)               // desire static 256ms(@Ts=1ms), adj zero start
            {
                gyroZZeroAcc = 0.f;
                acclYZeroAcc = 0.f;
            }
            else if(staticCntCyced >= 256 && staticCntCyced < 512)
            {
                gyroZZeroAcc += Imu->IMU_Data.GYRO_Z;
                acclYZeroAcc += Imu->IMU_Data.ACCEL_Y;
            }
            else if(staticCntCyced == 512)   // adj zero finish
            {
                GyroZZero = (float)gyroZZeroAcc * (1.f / 256.f) * PP::GyroUnit;
                AccelYZero = (float)acclYZeroAcc * (1.f / 256.f) * PP::AcclUnit;
								staticCnt = 0;
//                TskHif::DbgPuts((char *)"Acq 0s.\n");
            }
        }
        else
            staticCnt = 0;

}


void MotorCtl::MotorTestTick(){

    WheelEncoder->WheelEncGetVel();
//		printf("Enc Right Count: %d\r\n",WheelEncoder->WheelEncValueR.count);
//		printf("Enc Left Count: %d\r\n",WheelEncoder->WheelEncValueL.count);
//		printf("Enc Val: %f\r\n",WheelEncoder->EncVal);
		if(WheelEncoder->WheelEncValueR.count == 0 && WheelEncoder->WheelEncValueL.count == 0)stopcnt++;
		if(stopcnt > 15){MMode = Cal;stopcnt = 0;}
    crashDet = Imu->ImuReadData();
		NowSpeed.Av = (Imu->IMU_Data.GYRO_Z * PP::GyroUnit - GyroZZero);
//		printf("Av:%f\r\n",NowSpeed.Av);
    Accl = (Imu->IMU_Data.ACCEL_Y * PP::AcclUnit - AccelYZero);
//		printf("AcclY: %f\r\n",Accl);
    lvEstKal->Predict(Accl);
    NowSpeed.Lv = lvEstKal->Correct(WheelEncoder->EncVal);
//		printf("Lv:%f\r\n",NowSpeed.Lv);
    DistanceAcc += NowSpeed.Lv * PP::Ts;
    AngleAcc += NowSpeed.Av * PP::Ts;
#ifdef ANGULAR_VELOCITY_PID
    if(((dOmgAdj - OmgAdj) > -.003f)&&((dOmgAdj - OmgAdj) < .003f))dOmgAdj=OmgAdj;
    else if(dOmgAdj < OmgAdj)dOmgAdj+=.0026f;
    else if(dOmgAdj > OmgAdj)dOmgAdj-=.0028f;
    MotorDesireSpeed->Pop((uint8_t *)&DesireSpeed);
    desireAngle += ( + DesireSpeed.Av) * .001f;
    lvPidOut = LvPid->Tick(LvAdj + DesireSpeed.Lv - NowSpeed.Lv);
    avPidOut = AvPid->Tick(dOmgAdj + desireAngle - AngleAcc);
#else
		if(!MotorDesireSpeed->Pop((uint8_t *)&DesireSpeed))
	{
		DesireSpeed.Lv = .0f;
		DesireSpeed.Av = .0f;
		MotorDesireSpeed->Clear();
	}
//		printf("Desire Lv Speed: %f\r\n",DesireSpeed.Lv);
//		printf("Desire Av Speed: %f\r\n",DesireSpeed.Av);
//    MotorDesireSpeed->De(DesireSpeed);
//    desireAngle += ( + DesireSpeed.Av) * .001f;
    lvPidOut = -LvPid->Tick(LvAdj + DesireSpeed.Lv - NowSpeed.Lv);
//		printf("lvPid:%f\r\n",lvPidOut);
    avPidOut = AvPid->Tick(dOmgAdj + DesireSpeed.Av - NowSpeed.Av);
//		printf("avPid:%f\r\n",avPidOut);
#endif 
    rPwm = -(lvPidOut + avPidOut)*1.f;
    lPwm = -(lvPidOut - avPidOut)*1.f;
    PwmDuty.r = (int)saturate(rPwm, 1999.f, -1999.f);
    PwmDuty.l = (int)saturate(lPwm, 1999.f, -1999.f);
//    MPWM->MotorPwmSetDuty(PwmDuty);
        // imu adj zeros, auto adj zeros if static 0.64s
	

}

void MotorCtl::MotorCalTick()
{
	  volatile int Accl_Y_ZeroCnt = 0,Gyro_ZeroCnt = 0, i = 0;
		LvAdj = dOmgAdj = .0f;
		PwmDuty.l = 0;
		PwmDuty.r = 0;
		AccelYZero = 0;
		GyroZZero = 0;
		MPWM->MotorPwmSetDuty(PwmDuty);
	  lvPidOut = avPidOut = 0;
		rPwm = lPwm = 0;
//		MotorDesireSpeed->Clear();
		LED_R(1);LED_G(1);LED_B(1);
		OSTimeDlyHMSM(0, 0, 0,100, OS_OPT_TIME_HMSM_STRICT, &err);
		Imu->ImuReadData();
		Accl = (Imu->IMU_Data.ACCEL_Y * PP::AcclUnit - AccelYZero);
		WheelEncoder->WheelEncGetVel();
	  lvEstKal->Predict(Accl);
    NowSpeed.Lv = lvEstKal->Correct(WheelEncoder->EncVal);
		NowSpeed.Av = (Imu->IMU_Data.GYRO_Z * PP::GyroUnit - GyroZZero);
//		Imu->ImuReadData();
//		while(WheelEncoder->WheelEncValueL.count!= 0 || WheelEncoder->WheelEncValueR.count!= 0){
////			Imu->ImuReadData();
//			WheelEncoder->WheelEncGetVel();
//		}
//	int stop_cnt = 0;
//	while(stop_cnt < 128){
//		OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//		if((WheelEncoder->WheelEncValueL.count == 0) && (WheelEncoder->WheelEncValueR.count == 0))stop_cnt++;
//	}
		while((fabsf(NowSpeed.Lv) > 0.01f) || (fabsf(NowSpeed.Av) > 0.005f)){
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			Imu->ImuReadData();
			Accl = (Imu->IMU_Data.ACCEL_Y * PP::AcclUnit - AccelYZero);
			WheelEncoder->WheelEncGetVel();
			lvEstKal->Predict(Accl);
			NowSpeed.Lv = lvEstKal->Correct(WheelEncoder->EncVal);
			NowSpeed.Av = (Imu->IMU_Data.GYRO_Z * PP::GyroUnit - GyroZZero);
//			printf("Now Speed Lv:%f\r\n",NowSpeed.Lv);
//			printf("Now Speed Av:%f\r\n",NowSpeed.Av);
		}
		printf("Motor Cal Mode\r\n");
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		for(i = 0 ; i < 256; i++)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			Imu->ImuReadData();
			Accl_Y_ZeroCnt +=	Imu->IMU_Data.ACCEL_Y;
			Gyro_ZeroCnt +=	Imu->IMU_Data.GYRO_Z;
		}
		Accl_Y_ZeroCnt /=256.f;
		Gyro_ZeroCnt /=256.f;
    AccelYZero = Accl_Y_ZeroCnt * PP::AcclUnit;
    GyroZZero = Gyro_ZeroCnt * PP::GyroUnit;
//		Accl_Y_ZeroCnt = Gyro_ZeroCnt = 0;
    DistanceAcc = 0.f;
		LvPid->Reset();
		AvPid->Reset();
		//printf("Fifo len1:%d\r\n",MotorDesireSpeed->Length());
		LED_R(0);LED_G(0);LED_B(1);
    if(MMode == Cal)MMode = Run;
}
void MotorCtl::MotorIdleTick()
{
    MotorDesireSpeed->Clear();

    PwmDuty.l = 0;
    PwmDuty.r = 0;
		MPWM->MotorPwmSetDuty(PwmDuty);
    LvPid->Reset();
    AvPid->Reset();
    DistanceAcc = .0f;
    AngleAcc = .0f;		
		LvAdj = dOmgAdj = .0f;
}
}
//定时器3中断服务函数
//extern "C" void TIM7_IRQHandler(void)
//{
//	#if SYSTEM_SUPPORT_OS	 	//使用OS
//	OSIntEnter();    
//	#endif
//  HAL_TIM_IRQHandler(&MotorControl::TIMMotorTick_Handler);
//	#if SYSTEM_SUPPORT_OS	 	//使用OS
//	OSIntExit();  											 
//	#endif
//}

extern "C" void TIM7_IRQHandler(void)
{
	#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
	#endif
	  /* TIM Update event */
//  if(__HAL_TIM_GET_FLAG(&MotorControl::TIMMotorTick_Handler, TIM_FLAG_UPDATE) != RESET)
//  {
    if(__HAL_TIM_GET_IT_SOURCE(&MotorControl::TIMMotorTick_Handler, TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&MotorControl::TIMMotorTick_Handler, TIM_IT_UPDATE);
      OSSemPost(&MotorTick,OS_OPT_POST_ALL,&err);
			
    }
//  }
	#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
	#endif
}

////定时器3中断服务函数调用
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//}
