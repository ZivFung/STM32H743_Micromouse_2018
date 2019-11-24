#include "app.h"
using namespace MotorControl;
using namespace InfraredControl;
using namespace MouseAction;

//#define ExecuteAction(x)	{	OSTaskQPost(&ActionTaskTCB,(Act::ActType*)(x),4,OS_OPT_POST_FIFO,&err);\
//														OSTaskQPend(0,OS_OPT_PEND_BLOCKING,&actionEndMsgSize,NULL,&err);	}

extern volatile MouseAction::WallStatus cur_wall;

OS_ERR ActionExErr;
OS_MSG_SIZE actionEndMsgSize;
//Task TCB
OS_TCB StartTaskTCB;
//Task Stack
CPU_STK START_TASK_STK[START_STK_SIZE];



//任务优先级
#define TEST_TASK_PRIO		19
//任务堆栈大小	
#define TSET_STK_SIZE		1024
//Task TCB
OS_TCB	TESTTaskTCB;
//Task Stack
CPU_STK	TEST_TASK_STK[TSET_STK_SIZE];

void test_task(void *p_arg);

//任务优先级
#define MOTOR_TASK_PRIO		10
//任务堆栈大小	
#define MOTOR_STK_SIZE		2048
//Task TCB
OS_TCB	MotorTaskTCB;
//Task Stack
CPU_STK	MOTOR_TASK_STK[MOTOR_STK_SIZE];

void TskMotor(void *p_arg);


//任务优先级
#define IRIMU_TASK_PRIO		15
//任务堆栈大小	
#define IRIMU_STK_SIZE		512
//Task TCB
OS_TCB	IrImuTaskTCB;
//Task Stack
CPU_STK	IRIMU_TASK_STK[IRIMU_STK_SIZE];

void TskIrImu(void *p_arg);


//任务优先级
#define IMUTEST_TASK_PRIO		10
//任务堆栈大小	
#define IMUTEST_STK_SIZE		512
//Task TCB
OS_TCB	ImuTestTaskTCB;
//Task Stack
CPU_STK	IMUTEST_TASK_STK[IMUTEST_STK_SIZE];

void TskTestImu(void *p_arg);


//任务优先级
#define IRCORR_TASK_PRIO		11
//任务堆栈大小	
#define IRCORR_STK_SIZE		512
//Task TCB
OS_TCB	IrCorrTaskTCB;
//Task Stack
CPU_STK	IRCORR_TASK_STK[IRCORR_STK_SIZE];
void TskIrCorr(void *p_arg);


//任务优先级
#define ACTION_TASK_PRIO		16
//任务堆栈大小	
#define ACTION_STK_SIZE		2048
//Task TCB
OS_TCB	ActionTaskTCB;
//Task Stack
CPU_STK	ACTION_TASK_STK[ACTION_STK_SIZE];
void TskAction(void *p_arg);



////任务优先级
//#define MotorTest_TASK_PRIO		20
////任务堆栈大小	
//#define MotorTest_STK_SIZE		512
////Task TCB
//OS_TCB	MotorTestTCB;
////Task Stack
//CPU_STK	MotorTest_TASK_STK[MotorTest_STK_SIZE];

//void MotorTest(void *p_arg);
extern float BATV;
OS_SEM MotorTick;
OS_ERR err;
MotorPwm *MPWM;
MotorCtl *Motor;
Infrared *IR;
ICM20602 *Imu;
WheelEnc *Encoder;
IRCorrection *Ircorr;


//IrApproxCoef TestIrACs = {
//2.48148f, -0.64591f, 0.01312f,
//2.9494f, -0.72441f, 0.01690f,
//-7.94628f, 1.47859f, -0.09174f,
//1.53259f, -0.38549f, -0.00135f
//};
volatile uint8_t MotorInitFinish = 0;
void TskMotor(void *p_arg)
{
//		STMFLASH_Write(FLASH_SAVE_ADDR,(u32 *)&TestIrACs,6);
//		STMFLASH_Write(FLASH_SAVE_ADDR1,(u32 *)&TestIrACs.k[2],6);
		Ircorr = new IRCorrection();
		Motor = new MotorCtl();
		MotorInitFinish = 1;
//		MotorCtl Motor;
    while(1)
    {	
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
        switch(Motor->MMode)
        {
        case Cal:
            Motor->MotorCalTick();
            break;
        case Run:	
            Motor->MotorRunTick();
            break;
        case Idle:
            Motor->MotorIdleTick();
            break;
        case Free:
            break;
				case Test:
						Motor->MotorTestTick();
						break;
        }
//				OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}
Act::ActType LastAct;
void RamdomRun()
{
	Motor->MMode = Cal;
//	LED_R(0);LED_G(0);LED_B(1);
	OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
	while(Motor->MMode == Run);
	ExcuteAction(Act::Start);
//	LED_R(0);LED_G(1);LED_B(1);
	while(1){
		GetWallInfo();
		if((!cur_wall.WallStatusStruct.fwd)&&cur_wall.WallStatusStruct.left&&cur_wall.WallStatusStruct.right){
			printf("FWDis:%f",IR->IrDistFwd());
			ExcuteAction(Act::Fwd);
			LastAct = Act::Fwd;
		}
		else if(cur_wall.WallStatusStruct.fwd&&(!cur_wall.WallStatusStruct.left)&&cur_wall.WallStatusStruct.right){
			ExcuteAction(Act::L90);
			LastAct = Act::L90;
		}
		else if(cur_wall.WallStatusStruct.fwd&&cur_wall.WallStatusStruct.left&&(!cur_wall.WallStatusStruct.right)){
			ExcuteAction(Act::R90);
			LastAct = Act::R90;
		}
		else if(cur_wall.WallStatusStruct.fwd&&(!cur_wall.WallStatusStruct.left)&&(!cur_wall.WallStatusStruct.right)){
			ExcuteAction(Act::R90);
			LastAct = Act::R90;
		}
		else if((!cur_wall.WallStatusStruct.fwd)&&cur_wall.WallStatusStruct.left&&(!cur_wall.WallStatusStruct.right)){
			if(LastAct == Act::R90 || LastAct == Act::L90){
				printf("FWDis:%f",IR->IrDistFwd());
				ExcuteAction(Act::Fwd);
				LastAct = Act::Fwd;
			}
			else {
				ExcuteAction(Act::R90);
				LastAct = Act::R90;
			}
		}
		else if((!cur_wall.WallStatusStruct.fwd)&&(!cur_wall.WallStatusStruct.left)&&cur_wall.WallStatusStruct.right){
			if(LastAct == Act::R90 || LastAct == Act::L90){
				printf("FWDis:%f",IR->IrDistFwd());
				ExcuteAction(Act::Fwd);
				LastAct = Act::Fwd;
			}
			else {
			ExcuteAction(Act::L90);
			LastAct = Act::L90;
			}
		}
		else if((!cur_wall.WallStatusStruct.fwd)&&(!cur_wall.WallStatusStruct.left)&&(!cur_wall.WallStatusStruct.right)){
			printf("FWDis:%f",IR->IrDistFwd());
			ExcuteAction(Act::Fwd);
			LastAct = Act::Fwd;
		}
		else if(!(cur_wall.WallStatusStruct.fwd||cur_wall.WallStatusStruct.left||cur_wall.WallStatusStruct.right)){
			ExcuteAction(Act::Stop);
			ExcuteAction(Act::Back);
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
			ExcuteAction(Act::Restart);
			printf("FWDis:%f",IR->IrDistFwd());
			LastAct = Act::Restart;
		}
		else if(cur_wall.WallStatusStruct.fwd&&cur_wall.WallStatusStruct.left&&cur_wall.WallStatusStruct.right){
			ExcuteAction(Act::Stop);
			ExcuteAction(Act::Back);
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
			ExcuteAction(Act::Restart);
			printf("FWDis:%f",IR->IrDistFwd());
			LastAct = Act::Restart;
		}
		else {
			ExcuteAction(Act::Stop);
			LastAct = Act::Stop;
			break;
		}
//		OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &err);
		LED_B_Toggle;
		LED_G_Toggle;
	}

}

void RushTest(){
		Motor->MMode = Cal;
		while(Motor->MMode != Run){}
			
		ExcuteAction(Act::RushStart);
		ExcuteAction(Act::MidUp);
		ExcuteAction(Act::MidUp1);
		ExcuteAction(Act::MidDown);
		ExcuteAction(Act::MidDown1);
		ExcuteAction(Act::R135i);
		ExcuteAction(Act::XMidUp);
		ExcuteAction(Act::XMid1);
		ExcuteAction(Act::XMidDown1);
		ExcuteAction(Act::XR90);
		ExcuteAction(Act::L135o);
		ExcuteAction(Act::MidUp);
		ExcuteAction(Act::MidUp1);
		ExcuteAction(Act::HighUp);
		ExcuteAction(Act::HighUp1);
		ExcuteAction(Act::RushHigh);
		ExcuteAction(Act::RushHigh);
		ExcuteAction(Act::RushHigh);
		ExcuteAction(Act::HighDown);
		ExcuteAction(Act::HighDown1);
		ExcuteAction(Act::MidDown);
		ExcuteAction(Act::MidDown1);
		ExcuteAction(Act::RL90);
		ExcuteAction(Act::RushLow);
		ExcuteAction(Act::L45i);
		ExcuteAction(Act::XMidUp);
		ExcuteAction(Act::XMidUp1);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMidDown);
		ExcuteAction(Act::XMidDown1);
		ExcuteAction(Act::XL90);
		ExcuteAction(Act::XMidUp);
		ExcuteAction(Act::XMidUp1);
		ExcuteAction(Act::XMid);
		ExcuteAction(Act::XMidDown);
		ExcuteAction(Act::XMidDown1);
		ExcuteAction(Act::XL90);
		ExcuteAction(Act::R45o);
		ExcuteAction(Act::RL90);
		ExcuteAction(Act::L45i);
		ExcuteAction(Act::R45o);
		ExcuteAction(Act::L180);
		ExcuteAction(Act::RushStop);
		Motor->MMode = Idle;
}

static uint8_t IrTestState = 0;
static uint8_t ActionPrintf = 0;
//static uint8_t IrTestState1 = 0;

void RGBGpioInit()
{
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_Initure;
    GPIO_Initure.Pin=GPIO_PIN_10 | GPIO_PIN_11;            //IR_R0,1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
    GPIO_Initure.Pull=GPIO_PULLUP;          //不带上下拉
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
		GPIO_Initure.Pin = GPIO_PIN_15;
		HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}
void IrDistTest()
{
	OS_ERR  err;
	while(1){
			IR->IRInts.dFL = log((float)IR->IRInts.FL);
			IR->IRInts.dFL = exp(Ircorr->IrACs.k[0][0]+Ircorr->IrACs.k[0][1]*IR->IRInts.dFL+Ircorr->IrACs.k[0][2]*IR->IRInts.dFL*IR->IRInts.dFL);
			IR->IRInts.dFR = log((float)IR->IRInts.FR);
			IR->IRInts.dFR = exp(Ircorr->IrACs.k[1][0]+Ircorr->IrACs.k[1][1]*IR->IRInts.dFR+Ircorr->IrACs.k[1][2]*IR->IRInts.dFR*IR->IRInts.dFR);
			IR->IRInts.dSL = log((float)IR->IRInts.SL);
			IR->IRInts.dSL = exp(Ircorr->IrACs.k[2][0]+Ircorr->IrACs.k[2][1]*IR->IRInts.dSL+Ircorr->IrACs.k[2][2]*IR->IRInts.dSL*IR->IRInts.dSL);
			IR->IRInts.dSR = log((float)IR->IRInts.SR);
			IR->IRInts.dSR = exp(Ircorr->IrACs.k[3][0]+Ircorr->IrACs.k[3][1]*IR->IRInts.dSR+Ircorr->IrACs.k[3][2]*IR->IRInts.dSR*IR->IRInts.dSR);
			IRFallDistStampDet(IR->IRInts.SL,&IR->IRInts.LFallStp,IR->IRInts.SR,&IR->IRInts.RFallStp);
			IR->IRInts.YawL = (PP::CenterToWall - IR->IRInts.dSL) / PP::IrSFwd;
			IR->IRInts.YawR = (IR->IRInts.dSR - PP::CenterToWall) / PP::IrSFwd;
			IR->IRInts.YawFLR = (IR->IRInts.dFL - IR->IRInts.dFR) / (2.f * PP::IrFSide);
			printf("Distance FR: %f\r\n",IR->IRInts.dFR);
			printf("Distance FL: %f\r\n",IR->IRInts.dFL);
			printf("Distance SR: %f\r\n",IR->IRInts.dSR);
			printf("Distance SL: %f\r\n",IR->IRInts.dSL);
			printf("Distance YawR: %f\r\n",IR->IRInts.YawR);
			printf("Distance YawL: %f\r\n",IR->IRInts.YawL);
			printf("Distance YawFLR: %f\r\n",IR->IRInts.YawFLR);
			printf("LFallStp: %f\r\n",IR->IRInts.LFallStp);
			printf("RFallStp: %f\r\n",IR->IRInts.RFallStp);
			OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
		}
}

void ActionCorrection()
{
		Motor->MMode = Cal;
		while(Motor->MMode != Run){}
//		ExcuteAction(Act::L90);
//		ExcuteAction(Act::L90);
//		ExcuteAction(Act::Start);
//		ExcuteAction(Act::Fwd);	
//		ExcuteAction(Act::Fwd);	
//		ExcuteAction(Act::L90);	
			ExcuteAction(Act::RushStart);
			ExcuteAction(Act::L45o);
			
//			ExcuteAction(Act::RushLow);
//		ExcuteAction(Act::MidUp);
//		ExcuteAction(Act::MidUp1);
//		ExcuteAction(Act::RushMidLow);
//		ExcuteAction(Act::MidDown);
//		ExcuteAction(Act::MidDown1);
//		ExcuteAction(Act::RL90);
//		ExcuteAction(Act::RL90);
			
//		ExcuteAction(Act::MidUp);
//		ExcuteAction(Act::MidDown1);
//		ExcuteAction(Act::L45i);
//		ExcuteAction(Act::XMidUp);
//		ExcuteAction(Act::XMidUp1);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMidDown);
//		ExcuteAction(Act::XMidDown1);
//		ExcuteAction(Act::XL90);

//		ExcuteAction(Act::XMidUp);
//		ExcuteAction(Act::XMid1);
//		ExcuteAction(Act::XMidDown1);
//		ExcuteAction(Act::XL90);
//		ExcuteAction(Act::XR90);
//		ExcuteAction(Act::L45o);
//		ExcuteAction(Act::RL90);
//		ExcuteAction(Act::MidUp);
//		ExcuteAction(Act::MidDown1);
//		ExcuteAction(Act::RL90);
//		ExcuteAction(Act::RL90);
		
//		ExcuteAction(Act::MidUp);
//		ExcuteAction(Act::MidUp1);
//		ExcuteAction(Act::HighUp);
//		ExcuteAction(Act::HighUp1);
//		ExcuteAction(Act::RushHigh);
//		ExcuteAction(Act::RushHigh);
//		ExcuteAction(Act::RushHigh);
//		ExcuteAction(Act::RushHigh);
//		ExcuteAction(Act::HighDown);
//		ExcuteAction(Act::HighDown1);
//		ExcuteAction(Act::MidDown);
//		ExcuteAction(Act::MidDown1);
//		ExcuteAction(Act::RL90);
//		ExcuteAction(Act::RushLow);
//		ExcuteAction(Act::L45i);
//		ExcuteAction(Act::XMidUp);
//		ExcuteAction(Act::XMidUp1);
//		ExcuteAction(Act::XMid);
//		ExcuteAction(Act::XMidDown);
//		ExcuteAction(Act::XMidDown1);

		ExcuteAction(Act::RushStop);
		Motor->MMode = Idle;
}
	
void EncoderTest(){
	
//	MPWM = new MotorPwm();
		while(!MotorInitFinish){}
		Motor->MMode = Free;
		MotorDuty duty;
		duty.l = 300;
		duty.r = 300;
		Motor->MPWM->MotorPwmSetDuty(duty);
	
		while(1){
			Motor->WheelEncoder->WheelEncGetVel();
			printf("Encoder Left count: %d \r\n",Motor->WheelEncoder->WheelEncValueL.count);
			printf("Encoder Right count: %d \r\n",Motor->WheelEncoder->WheelEncValueR.count);

			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		}

}

void ModeEntry(MouseMode::ModeType mode)
{
		switch(mode)
    {
				case MouseMode::Idle:
						LED_R(0);LED_G_Toggle;LED_B(0);
						break;
				case MouseMode::IrValueTest:
						LED_R(1);LED_G(0);LED_B(0);
						break;
				case MouseMode::IrDistTest:
						LED_R(0);LED_G(0);LED_B(1);
						break;
				case MouseMode::ActionCorrection:
						LED_R(0);LED_G_Toggle;LED_B(1);
						break;
				case MouseMode::RandomRun:
						LED_R_Toggle;LED_G(1);LED_B(0);
						break;
				case MouseMode::FirstStart:
						LED_R(0);LED_G(0);LED_B_Toggle;
						break;
				case MouseMode::SecondStart:
						LED_R_Toggle;LED_G(0);LED_B(1);
						break;
				case MouseMode::IrCorrection:
						LED_R_Toggle;LED_G(0);LED_B(0);
						break;
				case MouseMode::RushTest:
						LED_R(0);LED_G(0);LED_B_Toggle;
						break;
				case MouseMode::FlashWriteTest:
						LED_R(0);LED_G_Toggle;LED_B_Toggle;
						break;
				case MouseMode::EncTest:
						LED_R_Toggle;LED_G(1);LED_B_Toggle;
						break;
				default:
						LED_R(0);LED_G(0);LED_B(0);
						break;
    }
		
		if(IR->IRInts.FR > 35000)IrTestState = 1;
		else if(IR->IRInts.FR > 35000 && IrTestState == 1)IrTestState = 1;
		else if(IrTestState == 1 && IR->IRInts.FR < 35000)IrTestState = 2;
		else if(IrTestState == 2) IrTestState = IrTestState;
		else IrTestState = 0;
	
		if(IrTestState == 2)
		{
				LED_R(0);
				switch(mode)
				{
						case MouseMode::Idle:
								break;
						case MouseMode::IrValueTest:
								printf("Mode IR Value Test.\r\n");
								IR->IRSelfTest();
								break;
						case MouseMode::IrDistTest:
								printf("Mode IR Dist Test.\r\n");
								IrDistTest();
								break;
						case MouseMode::ActionCorrection:
								printf("Mode Action Correction start.\r\n");
								ActionCorrection();
								break;
						case MouseMode::RandomRun:
								printf("Mode Random Run start.\r\n");
								RamdomRun();
								break;
						case MouseMode::FirstStart:
								printf("First search start.\r\n");
								GameBegin(false);
								break;
						case MouseMode::SecondStart:
								printf("Second search start.\r\n");
								GameBegin(true);
								break;
						case MouseMode::IrCorrection:
								printf("Mode IR Correction1 start.\r\n");
								Ircorr->doIrCorrection1();
								break;
						case MouseMode::RushTest:
								printf("Mode Rush Test Statr.\r\n");
								RushTest();
								break;
						case MouseMode::FlashWriteTest:
								printf("Mode Flash Test Start.\r\n");
								FlashTest();
								break;
						case MouseMode::EncTest:
								printf("Mode Encoder Test Start.\r\n");
								EncoderTest();
								break;
						default:
								printf((char*)"Undefined mode.\r\n");

								break;
				}
				mode = MouseMode::Idle;
				IrTestState = 0;
		}
}

void TskTop(void *p_arg)
{
		OS_ERR  err;
		MouseMode::ModeType Mode = MouseMode::Idle,lastMode = MouseMode::Idle;
		
		OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		RGBGpioInit();
		while(!MotorInitFinish){}
		while(1){
				lastMode = Mode;
				Mode = (MouseMode::ModeType)((-(int16_t)TIM4->CNT >= 0) ? ((-(int16_t)TIM4->CNT)/36) : 0);
				if(Mode != lastMode)printf("%d\r\n",Mode);
				ModeEntry(Mode);
				OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
				//GameBegin(true);
		}
}

void test_task(void *p_arg)
{
//	MotorPwm MPWM;
//	MPWM = new MotorPwm();
//	MotorDuty duty;
//	duty.l = 300;
//	duty.r = 300;
//	MPWM->MotorPwmSetDuty(duty);
//	Encoder = new WheelEnc();
	OS_MSG_SIZE MsgSize;
	OS_ERR msg_err;
	OS_MSG_SIZE msg_size;
	u8 *p;
	volatile MotorSpeedDef speed;
	speed.Av = 0;
	speed.Lv = 0;
	volatile int i;
//	Act::ActType actions[256];
//	*actions = Act::Start;
//	*(actions + 1) = Act::Fwd;
//	*(actions + 2) = Act::Fwd;
//	
//	Imu = new ICM20602();
//	IR = new Infrared();
//	OSSemPend(&InfraredControl::IRCHTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	while(1)
	{	
//		MPWM.MotorPwmSetDuty(duty);
//		printf("PWM Duty Right = %2.1f%%\n",(float)duty.r/2000.0f);
//		printf("PWM Duty Left = %2.1f%%\n",(float)duty.l/2000.0f);
//		p = (u8 *)OSTaskQPend(0,OS_OPT_PEND_BLOCKING,&MsgSize,NULL,&err);
//		printf("recive : %s\n",p);
		if(MotorInitFinish){
//		Imu->ImuSelfTest();
//		IR->GetIRValue();
//		printf("Vbat: %fV \r\n",BATV);
//		printf("IR FR ADC Value: %d \r\n",IR->IRInts.FR);
//		if(IR->IRInts.FR > 8200 && Encoder->WheelEncValueR.count > 15){duty.l = -1200; duty.r = -1200;MPWM->MotorPwmSetDuty(duty);}
//		else if(IR->IRInts.FR > 8200 && Encoder->WheelEncValueR.count <= 15){duty.l = 0; duty.r = 0;MPWM->MotorPwmSetDuty(duty);}
//		else {duty.l = 250; duty.r = 250;MPWM->MotorPwmSetDuty(duty);}
//		Encoder->WheelEncGetVel();
//		printf("Encoder Left Encoder: %d \r\n",Encoder->WheelEncValueL.encoder);
//		printf("Encoder Left old Encoder: %d \r\n",Encoder->WheelEncValueL.old_encoder);
//		printf("Encoder Left count: %d \r\n",Encoder->WheelEncValueL.count);
//		printf("Encoder Right Encoder: %d \r\n",Encoder->WheelEncValueR.encoder);
//		printf("Encoder Right old Encoder: %d \r\n",Encoder->WheelEncValueR.old_encoder);
//		printf("Encoder Right count: %d \r\n",Encoder->WheelEncValueR.count);
//		printf("GyroZ:%d\n",Imu->IMU_Data.GYRO_Z);
//		Imu->IMU_WriteRegister(testSeq,2);
//		GetWallInfo();
//		Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
		if(IR->IRInts.FR > 35000)IrTestState = 1;
		else if(IR->IRInts.FR > 35000 && IrTestState == 1)IrTestState = 1;
		else if(IrTestState == 1 && IR->IRInts.FR < 35000)IrTestState = 2;
		else if(IrTestState == 2) IrTestState = IrTestState;
		else IrTestState = 0;
		
		
//		if(IR->IRInts.FR > 35000)IrTestState = 1;
//		else if(IR->IRInts.FR > 35000 && IrTestState == 1)IrTestState = 1;
//		else if(IrTestState == 1 && IR->IRInts.FR < 30000)IrTestState = 2;
//		else if(IrTestState == 2) IrTestState = IrTestState;
//		else IrTestState = 0;
		
		if(IrTestState == 2){
		Motor->MMode = Cal;
		LED_R(0);LED_G(0);LED_B(1);
		OSTimeDlyHMSM(0, 0, 0, 200, OS_OPT_TIME_HMSM_STRICT, &err);
//		printf("/*************************************************/\r\n");
//		printf("Motor PID Test Start:\r\n");
//		speed.Lv = 1.2f;
////		speed.Av = 4 * PP::Pi;
//		Motor->QueueEmptyOp = 1;
//		while(Motor->MMode != Run){}
//		for(i = 0; i < 300; i++)Motor->MotorDesireSpeed->Push((uint8_t *)&speed);
//		speed.Lv = 0.0f;
//		speed.Av = 0.f;
//		//for(i = 0; i < 200; i++)Motor->MotorDesireSpeed->Push((uint8_t *)&speed);
//		
////			printf("Accl:%f\r\n",Motor->AccelYZero);
////			printf("Gyro:%f\r\n",Motor->GyroZZero);
////			printf("Fifo len2:%d\r\n",Motor->MotorDesireSpeed->Length());
//			
//		while(Motor->MotorDesireSpeed->Length() > 0){
//			printf("%f\r\n",Motor->NowSpeed.Lv);
////			printf("%f\r\n",Motor->diff);
////			printf("%f\r\n",Motor->lvPidOut);
////			printf("%f\r\n",Motor->NowSpeed.Av);
//			//printf("%d\r\n",Motor->PwmDuty.l);
//		}
//		LED_R(0);LED_G(1);LED_B(1);
//		for(int i = 0; i < 250;i++){
//			//printf("%f\r\n",Motor->NowSpeed.Lv);
////			printf("%f\r\n",Motor->diff);
////			printf("%f\r\n",Motor->lvPidOut);
//			printf("%f\r\n",Motor->NowSpeed.Av);
//		}
//		OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//		printf("Motor PID Test Finish:\r\n");
//		Motor->MotorDesireSpeed->Clear();
//		 RamdomRun();
//		 
//		while(Motor->MMode != Run){}
//		ActionPrintf = 0;
//		ExcuteAction(Act::Restart);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Back);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Restart);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Fwd);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::RushStart);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::MidUp);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::RushMid);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::MidDown);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R45i);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XLow);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XR90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XL90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R135o);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L45i);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R45o);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R45i);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XLow);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R45o);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::R135i);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L45o);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::RushStop);
//		LED_R_Toggle;
//		printf("\r\n");

//		ExcuteAction(Act::XMidUp);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XMidDown);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XL90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XMidUp);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XMid);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::XMidDown);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L45o);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::RushLow);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::RushStop);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Fwd);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Back);
//		LED_B_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Start);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Fwd);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Back);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		LED_R_Toggle;
//		printf("\r\n");
//		ActionPrintf = 0;
//		ExcuteAction(Act::Start);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Fwd);
//		LED_R_Toggle;
//		ActionPrintf = 1;

//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		LED_R_Toggle;
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		printf("\r\n");
//		ExcuteAction(Act::Back);
//		printf("\r\n");
//		ExcuteAction(Act::Restart);
//		printf("\r\n");
//		ExcuteAction(Act::L90);
//		printf("\r\n");
//		ExcuteAction(Act::Fwd);
//		printf("\r\n");
//		ExcuteAction(Act::Stop);
//		ActionPrintf = 0;
		
//			while(Motor->MMode != 1);
//			GameBegin(false);
		RushTest();	
//			printf("Distance YawR: %f\r\n",IR->IRInts.YawR);
//			printf("Distance YawL: %f\r\n",IR->IRInts.YawL);
//			printf("OmgAdj: %f\r\n",Motor->dOmgAdj);
//			printf("rpwm:%d\r\n",Motor->PwmDuty.r);
//			printf("lpwm:%d\r\n",Motor->PwmDuty.l);
//			printf("Now Lv:%f\r\n",Motor->NowSpeed.Lv);
//			printf("Now Av:%f\r\n",Motor->NowSpeed.Av);
//		OSTimeDlyHMSM(0, 0, 2, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//			printf("Distance YawR: %f\r\n",IR->IRInts.YawR);
//			printf("Distance YawL: %f\r\n",IR->IRInts.YawL);
//			printf("OmgAdj: %f\r\n",Motor->dOmgAdj);
//			printf("rpwm:%d\r\n",Motor->PwmDuty.r);
//			printf("lpwm:%d\r\n",Motor->PwmDuty.l);
//			printf("Now Lv:%f\r\n",Motor->NowSpeed.Lv);
//			printf("Now Av:%f\r\n",Motor->NowSpeed.Av);
		IrTestState = 0;
		LED_R(0);LED_G(1);LED_B(0);
		Motor->MMode = Idle;
		}
		else if(IrTestState == 1){
			LED_R(0);LED_G(0);LED_B(0);
//			Motor->MMode = Cal;
		}
		else{
//			actPowerupDirCorr();
//			GetWallInfo();
//			Motor->dOmgAdj = 1.f*actHeadingDirCorrBySideIrSide(&cur_wall);
//			printf("Distance YawR: %f\r\n",IR->IRInts.YawR);
//			printf("Distance YawL: %f\r\n",IR->IRInts.YawL);
//			printf("OmgAdj: %f\r\n",Motor->dOmgAdj);
//			printf("rpwm:%d\r\n",Motor->PwmDuty.r);
//			printf("lpwm:%d\r\n",Motor->PwmDuty.l);
//			printf("Now Lv:%f\r\n",Motor->NowSpeed.Lv);
//			printf("Now Av:%f\r\n",Motor->NowSpeed.Av);
			OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
		}
		
	}
	else{
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		}
	}

}


void IRFallDistStampDet(uint16_t LIRint,float *LDistStamp,uint16_t RIRint,float *RDistStamp)
{		
		static int LMax,RMax;
		
		if(Motor->MMode == Run)
		{	
				LMax -= 1;
				if(LIRint > LMax)LMax = LIRint;
			 else if((LMax > SCP::LDISTSTP_HTH)&&(LIRint < (LMax * .65f))&&(LIRint < SCP::LDISTSTP_LTH))
				{
						LMax = 0;
						*LDistStamp = Motor->DistanceAcc;
				}
				
				RMax -= 1;
				if(RIRint > RMax)RMax = RIRint;
				else if((RMax > SCP::RDISTSTP_HTH)&&(RIRint < (RMax * .65f))&&(RIRint < SCP:: RDISTSTP_LTH))
				{
						RMax = 0;
						*RDistStamp = Motor->DistanceAcc;
				}
		}
		else{
				*LDistStamp = -1.f;
				*RDistStamp = -1.f;
		}
}

void TskIrImu(void *p_arg)
{
		OS_ERR err;

		IR = new Infrared();
		IR->IRInts.LFallStp = .0f;
		IR->IRInts.RFallStp = .0f;

		while(1)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			if(MotorInitFinish){
				IR->GetIRValue();
//				printf("Vbat: %fV \r\n",BATV);
				IR->IRInts.dFL = log((float)IR->IRInts.FL);
				IR->IRInts.dFL = exp(Ircorr->IrACs.k[0][0]+Ircorr->IrACs.k[0][1]*IR->IRInts.dFL+Ircorr->IrACs.k[0][2]*IR->IRInts.dFL*IR->IRInts.dFL);
				IR->IRInts.dFR = log((float)IR->IRInts.FR);
				IR->IRInts.dFR = exp(Ircorr->IrACs.k[1][0]+Ircorr->IrACs.k[1][1]*IR->IRInts.dFR+Ircorr->IrACs.k[1][2]*IR->IRInts.dFR*IR->IRInts.dFR);
				IR->IRInts.dSL = log((float)IR->IRInts.SL);
				IR->IRInts.dSL = exp(Ircorr->IrACs.k[2][0]+Ircorr->IrACs.k[2][1]*IR->IRInts.dSL+Ircorr->IrACs.k[2][2]*IR->IRInts.dSL*IR->IRInts.dSL);
				IR->IRInts.dSR = log((float)IR->IRInts.SR);
				IR->IRInts.dSR = exp(Ircorr->IrACs.k[3][0]+Ircorr->IrACs.k[3][1]*IR->IRInts.dSR+Ircorr->IrACs.k[3][2]*IR->IRInts.dSR*IR->IRInts.dSR);
			  IRFallDistStampDet(IR->IRInts.SL,&IR->IRInts.LFallStp,IR->IRInts.SR,&IR->IRInts.RFallStp);
				IR->IRInts.YawL = (PP::CenterToWall - IR->IRInts.dSL) / PP::IrSFwd;
				IR->IRInts.YawR = (IR->IRInts.dSR - PP::CenterToWall) / PP::IrSFwd;
				IR->IRInts.YawFLR = (IR->IRInts.dFL - IR->IRInts.dFR) / (2.f * PP::IrFSide) - 0.02;
//				printf("LFallStp:%f\r\n",IR->IRInts.LFallStp);
//				printf("RFallStp:%f\r\n",IR->IRInts.RFallStp);
//				Motor->crashDet = Motor->Imu->ImuReadData();
				if(ActionPrintf){
//					printf("%f\r\n",Motor->NowSpeed.Lv);
					printf("%f\r\n",Motor->NowSpeed.Av);
//					printf("DSr:%f\r\n",IR->IRInts.dSR);
//					printf("DSl:%f\r\n",IR->IRInts.dSL);
//					printf("Yawl:%f\r\n",IR->IRInts.YawL);
//					printf("Yawr:%f\r\n",IR->IRInts.YawR);
//					printf("omgadj:%f\r\n",Motor->dOmgAdj);
				}
				if(Motor->crashDet > 40)
				{
						while(1)
						{
								Motor->MMode = Idle;
								Motor->crashDet = Motor->Imu->ImuReadData();
//								printf("Imu Gyro_Z: %5d\r\n", Motor->Imu->IMU_Data.GYRO_Z);
//								printf("Imu Accl_Y: %5d\r\n", Motor->Imu->IMU_Data.ACCEL_Y);
//								Motor->Imu->ImuSelfTest();
								OSTimeDlyHMSM(0, 0, 0, 120, OS_OPT_TIME_HMSM_STRICT, &err);
								LED_R(1);LED_G(0);LED_B(0);
								OSTimeDlyHMSM(0, 0, 3, 0, OS_OPT_TIME_HMSM_STRICT, &err);
								LED_R(0);LED_G(1);LED_B(0);
								Motor->MMode = Cal;
								printf("CrashData: %d",Motor->crashDet);
								break;
						}
				}
				else if(BATV < 3.2f){
					Motor->MMode = Idle;
					LED_R(1);LED_G(1);LED_B(0);
					while(1){
						IR->GetIRValue();
						printf("Vbat: %fV \r\n",BATV);
						if(BATV > 3.6f){
							Motor->MMode = Cal;
							break;
						}
					}
				}
//			OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_HMSM_STRICT, &err);
		}
		else{
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		}
	}
	
}


Act::ActType actCurrAct;
//Pid *yawPid;
void TskAction(void *p_arg){
		OS_MSG_SIZE msg_size = 1;
		OS_ERR err;
		OS_ERR msg_err;
		WallStatus stopWall, nextWall, wall;
		ActMsg::MsgType end_msg;
    actCurrAct = Act::Null;
    Act::ActType * act;
		stopWall.WallStatusStruct.fwd =1;
		stopWall.WallStatusStruct.left =1;
		stopWall.WallStatusStruct.right =1;
		while(1){
			if(MotorInitFinish){
				act = (Act::ActType *)OSTaskQPend (0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
//				OSTaskQPost(&TESTTaskTCB,(uint8_t*)0x46,1,OS_OPT_POST_FIFO,&msg_err);
				actCurrAct = (Act::ActType)((uint32_t)act & 0xFF000000);
				LED_G_Toggle;
				switch(actCurrAct)
				{
						
						case Act::Null:
								printf("\nn\n");
								break;
						case Act::Start:
								printf("\ns\n");
								actStart(true);
								break;
						case Act::Stop:
								printf("\np\n");
								//LED_R(1);LED_G(0);LED_B(1);
								stopWall.msk = actStop(true);
								break;
						case Act::Back:
								printf("\nb\n");
								actBack(&stopWall,true);
								break;
						case Act::Restart:
								printf("\nr\n");
								actRestart(true);
								break;
						case Act::Fwd:
								printf("\n^\n");
								actFwd(true);
								break;
						case Act::L90:
								printf("\n<\n");				
								actLR90(actCurrAct,true);
								break;
						case Act::R90:
								printf("\n>\n");
								actLR90(actCurrAct,true);
								break;
						case Act::RotateL90:
								actRotateL90(true);
								break;
						case Act::RotateR90:
								actRotateR90(true);
								break;
						
						case Act::RushStart:
								actRushStart();
								break;
						case Act::MidUp:
								actRush(PP::RushSpeedLow,PP::RushSpeedMidLow);
								break;
						case Act::MidUp1:
								actRush(PP::RushSpeedMidLow,PP::RushSpeedMid);
								break;
						case Act::HighUp:
								actRush(PP::RushSpeedMid,PP::RushSpeedHighLow);
								break;
						case Act::HighUp1:
								actRush(PP::RushSpeedHighLow,PP::RushSpeedHigh);
								break;
						case Act::RushHigh:
								actRush(PP::RushSpeedHigh,PP::RushSpeedHigh);
								break;
						case Act::RushMid:
								actRush(PP::RushSpeedMid,PP::RushSpeedMid);
								break;
						case Act::RushMidLow:
								actRush(PP::RushSpeedMidLow,PP::RushSpeedMidLow);
								break;
						case Act::RushHighLow:
								actRush(PP::RushSpeedHighLow,PP::RushSpeedHighLow);
								break;
						case Act::RushLow:
								actRush(PP::RushSpeedLow,PP::RushSpeedLow);
								break;
						case Act::HighDown:
								actRush(PP::RushSpeedHigh,PP::RushSpeedHighLow);
								break;
						case Act::HighDown1:
								actRush(PP::RushSpeedHighLow,PP::RushSpeedMid);
								break;
						case Act::MidDown:
								actRush(PP::RushSpeedMid,PP::RushSpeedMidLow);
								break;
						case Act::MidDown1:
								actRush(PP::RushSpeedMidLow,PP::RushSpeedLow);
								break;
						case Act::RushStop:
								stopWall.msk = actRushStop();
								break;
				case Act::XLow:
						actXRush(PP::RushSpeedLow, PP::RushSpeedLow);
						break;
				case Act::XMid:
						actXRush(PP::XSpeedMid, PP::XSpeedMid);
						break;
				case Act::XMid1:
						actXRush(PP::XSpeedMidLow,PP::XSpeedMidLow);
						break;
				case Act::XMidUp:
						actXRush(PP::RushSpeedLow, PP::XSpeedMidLow);
						break;
				case Act::XMidDown:
						actXRush(PP::XSpeedMid, PP::XSpeedMidLow);
						break;
				case Act::XMidUp1:
						actXRush(PP::XSpeedMidLow, PP::XSpeedMid);
						break;
				case Act::XMidDown1:
						actXRush(PP::XSpeedMidLow, PP::RushSpeedLow);
						break;
				case Act::CRush:
						break;
				case Act::TRush:
						break;
				case Act::Turn:
						actBack(&stopWall,true);
						break;
				case Act::RushOut:
						break;
				case Act::L45i:
						actL45i();
						break;
				case Act::L45o:
						actL45o();
						break;
				case Act::R45i:
						actR45i();
						break;
				case Act::R45o:
						actR45o();
						break;
				case Act::RL90:
						actRL90();
						break;
				case Act::RR90:
						actRR90();
						break;
				case Act::XL90:
						actXL90();
						break;
				case Act::XR90:
						actXR90();
						break;
				case Act::L135i:
						actL135i();
						break;
				case Act::L135o:
						actL135o();
						break;
				case Act::R135i:
						actR135i();
						break;
				case Act::R135o:
						actR135o();
						break;
				case Act::L180:
						actL180();
						break;
				case Act::R180:
						actR180();
						break;
				default:
						break; 
				}
				OSTaskQPost(&TESTTaskTCB,&end_msg,1,OS_OPT_POST_FIFO,&err);
			}
		else{
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		}			
		}


}

void TskTestImu(void *p_arg)
{
		Imu = new ICM20602();
		RGBGpioInit();
		LED_R(0);LED_G(1);LED_B(0);
//		MPWM = new MotorPwm();
//		MotorDuty duty;
//		duty.l = 300;
//		duty.r = +300;
//		MPWM->MotorPwmSetDuty(duty);
//		Encoder = new WheelEnc();
		IR = new Infrared();
		while(1){
			Imu->ImuSelfTest();
//			Encoder->WheelEncGetVel();
//			printf("Encoder Left count: %d \r\n",Encoder->WheelEncValueL.count);
//			printf("Encoder Right count: %d \r\n",Encoder->WheelEncValueR.count);
//			IR->IRSelfTest();
			IR->GetIRValue();
			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		}
}






static uint8_t IrCorrectionFinish = 0;
void TskIrCorr(void *p_arg)
{
//	TestIrACs.k = test;
//		STMFLASH_Write(FLASH_SAVE_ADDR,(u32 *)&TestIrACs.k[0],6);
//		STMFLASH_Write(FLASH_SAVE_ADDR1,(u32 *)&TestIrACs.k[2],6);
		Ircorr = new IRCorrection();
		Ircorr->doIrCorrection1();
		while(1){
			IR->IRInts.dFL = log((float)IR->IRInts.FL);
			IR->IRInts.dFL = exp(Ircorr->IrACs.k[0][0]+Ircorr->IrACs.k[0][1]*IR->IRInts.dFL+Ircorr->IrACs.k[0][2]*IR->IRInts.dFL*IR->IRInts.dFL);
			IR->IRInts.dFR = log((float)IR->IRInts.FR);
			IR->IRInts.dFR = exp(Ircorr->IrACs.k[1][0]+Ircorr->IrACs.k[1][1]*IR->IRInts.dFR+Ircorr->IrACs.k[1][2]*IR->IRInts.dFR*IR->IRInts.dFR);
			IR->IRInts.dSL = log((float)IR->IRInts.SL);
			IR->IRInts.dSL = exp(Ircorr->IrACs.k[2][0]+Ircorr->IrACs.k[2][1]*IR->IRInts.dSL+Ircorr->IrACs.k[2][2]*IR->IRInts.dSL*IR->IRInts.dSL);
			IR->IRInts.dSR = log((float)IR->IRInts.SR);
			IR->IRInts.dSR = exp(Ircorr->IrACs.k[3][0]+Ircorr->IrACs.k[3][1]*IR->IRInts.dSR+Ircorr->IrACs.k[3][2]*IR->IRInts.dSR*IR->IRInts.dSR);
//			IRFallDistStampDet(IR->IRInts.SL,&IR->IRInts.LFallStp,IR->IRInts.SR,&IR->IRInts.RFallStp);
				IR->IRInts.YawL = (PP::CenterToWall - IR->IRInts.dSL) / PP::IrSFwd;
				IR->IRInts.YawR = (IR->IRInts.dSR - PP::CenterToWall) / PP::IrSFwd;
				IR->IRInts.YawFLR = (IR->IRInts.dFL - IR->IRInts.dFR) / (2.f * PP::IrFSide);
			printf("Distance FR: %f\r\n",IR->IRInts.dFR);
			printf("Distance FL: %f\r\n",IR->IRInts.dFL);
			printf("Distance SR: %f\r\n",IR->IRInts.dSR);
			printf("Distance SL: %f\r\n",IR->IRInts.dSL);
			printf("Distance YawR: %f\r\n",IR->IRInts.YawR);
			printf("Distance YawL: %f\r\n",IR->IRInts.YawL);
			printf("Distance YawFLR: %f\r\n",IR->IRInts.YawFLR);
			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		}
}

void MotorTest(void *p_arg){


while(1){
	if(MotorInitFinish){
		printf("Enc Right Count: %d\r\n",Motor->WheelEncoder->WheelEncValueR.count);
		printf("Enc Left Count: %d\r\n",Motor->WheelEncoder->WheelEncValueL.count);
		printf("Enc Val: %f\r\n",Motor->WheelEncoder->EncVal);
		printf("Av:%f\r\n",Motor->NowSpeed.Av);
		printf("AcclY: %f\r\n",Motor->Accl);
		printf("Lv:%f\r\n",Motor->NowSpeed.Lv);
		printf("Desire Lv Speed: %f\r\n",Motor->DesireSpeed.Lv);
		printf("Desire Av Speed: %f\r\n",Motor->DesireSpeed.Av);
		printf("lvPid:%f\r\n",Motor->lvPidOut);
		printf("avPid:%f\r\n",Motor->avPidOut);
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
	}
	else OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);

}
}

//??????
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//????                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//?????????????
    CPU_IntDisMeasMaxCurReset();	
#endif

#if OS_CFG_APP_HOOKS_EN				//??????
	App_OS_SetAllHooks();			
#endif
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //???????????
	 //???????????,??????????s
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	//Math_Init();
	OS_CRITICAL_ENTER();	//?????
//	//??LED0??
//	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
//				 (CPU_CHAR	* )"led0 task", 		
//                 (OS_TASK_PTR )led0_task, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )LED0_TASK_PRIO,     
//                 (CPU_STK   * )&LED0_TASK_STK[0],	
//                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
//                 (CPU_STK_SIZE)LED0_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,					
//                 (void   	* )0,					
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP,
//                 (OS_ERR 	* )&err);				
//				 
//	//??LED1??
//	OSTaskCreate((OS_TCB 	* )&Led1TaskTCB,		
//				 (CPU_CHAR	* )"led1 task", 		
//                 (OS_TASK_PTR )led1_task, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )LED1_TASK_PRIO,     	
//                 (CPU_STK   * )&LED1_TASK_STK[0],	
//                 (CPU_STK_SIZE)LED1_STK_SIZE/10,	
//                 (CPU_STK_SIZE)LED1_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,					
//                 (void   	* )0,				
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
//                 (OS_ERR 	* )&err);
//				 
//	//????????
//	OSTaskCreate((OS_TCB 	* )&FloatTaskTCB,		
//				 (CPU_CHAR	* )"float test task", 		
//                 (OS_TASK_PTR )float_task, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )FLOAT_TASK_PRIO,     	
//                 (CPU_STK   * )&FLOAT_TASK_STK[0],	
//                 (CPU_STK_SIZE)FLOAT_STK_SIZE/10,	
//                 (CPU_STK_SIZE)FLOAT_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,					
//                 (void   	* )0,				
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
//                 (OS_ERR 	* )&err);

	OSTaskCreate(  (OS_TCB 	* )&TESTTaskTCB,		
								 (CPU_CHAR	* )"test task", 		
								 (OS_TASK_PTR )TskTop, 			
								 (void		* )0,					
								 (OS_PRIO	  )TEST_TASK_PRIO,     	
								 (CPU_STK   * )&TEST_TASK_STK[0],	
								 (CPU_STK_SIZE)TSET_STK_SIZE/10,	
								 (CPU_STK_SIZE)TSET_STK_SIZE,		
								 (OS_MSG_QTY  )0,					
								 (OS_TICK	  )0,					
								 (void   	* )0,				
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
								 (OS_ERR 	* )&err);
	//								 
	OSTaskCreate((OS_TCB 	* )&MotorTaskTCB,		
				 (CPU_CHAR	* )"Motor task", 		
								 (OS_TASK_PTR )TskMotor, 			
								 (void		* )0,					
								 (OS_PRIO	  )MOTOR_TASK_PRIO,     	
								 (CPU_STK   * )&MOTOR_TASK_STK[0],	
								 (CPU_STK_SIZE)MOTOR_STK_SIZE/10,	
								 (CPU_STK_SIZE)MOTOR_STK_SIZE,		
								 (OS_MSG_QTY  )0,					
								 (OS_TICK	  )0,					
								 (void   	* )0,				
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
								 (OS_ERR 	* )&err);
//								 
	OSTaskCreate((OS_TCB 	* )&IrImuTaskTCB,		
				 (CPU_CHAR	* )"IrImu task", 		
								 (OS_TASK_PTR )TskIrImu, 			
								 (void		* )0,					
								 (OS_PRIO	  )IRIMU_TASK_PRIO,     	
								 (CPU_STK   * )&IRIMU_TASK_STK[0],	
								 (CPU_STK_SIZE)IRIMU_STK_SIZE/10,	
								 (CPU_STK_SIZE)IRIMU_STK_SIZE,		
								 (OS_MSG_QTY  )0,					
								 (OS_TICK	  )0,					
								 (void   	* )0,				
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
								 (OS_ERR 	* )&err);
	
//	OSTaskCreate((OS_TCB 	* )&ImuTestTaskTCB,		
//				 (CPU_CHAR	* )"Imu Test task", 		
//								 (OS_TASK_PTR )TskTestImu, 			
//								 (void		* )0,					
//								 (OS_PRIO	  )IMUTEST_TASK_PRIO,     	
//								 (CPU_STK   * )&IMUTEST_TASK_STK[0],	
//								 (CPU_STK_SIZE)IMUTEST_STK_SIZE/10,	
//								 (CPU_STK_SIZE)IMUTEST_STK_SIZE,		
//								 (OS_MSG_QTY  )0,					
//								 (OS_TICK	  )0,					
//								 (void   	* )0,				
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
//								 (OS_ERR 	* )&err);
//	OSTaskCreate((OS_TCB 	* )&IrCorrTaskTCB,		
//				 (CPU_CHAR	* )"IrCorr task", 		
//								 (OS_TASK_PTR )TskIrCorr, 			
//								 (void		* )0,					
//								 (OS_PRIO	  )IRCORR_TASK_PRIO,     	
//								 (CPU_STK   * )&IRCORR_TASK_STK[0],	
//								 (CPU_STK_SIZE)IRCORR_STK_SIZE/10,	
//								 (CPU_STK_SIZE)IRCORR_STK_SIZE,		
//								 (OS_MSG_QTY  )0,					
//								 (OS_TICK	  )0,					
//								 (void   	* )0,				
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
//								 (OS_ERR 	* )&err);		

	OSTaskCreate((OS_TCB 	* )&ActionTaskTCB,		
				 (CPU_CHAR	* )"Action task", 		
                 (OS_TASK_PTR )TskAction, 			 
                 (void		* )0,					
                 (OS_PRIO	  )ACTION_TASK_PRIO,     	
                 (CPU_STK   * )&ACTION_TASK_STK[0],	
                 (CPU_STK_SIZE)ACTION_STK_SIZE/10,	
                 (CPU_STK_SIZE)ACTION_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);		

//	OSTaskCreate((OS_TCB 	* )&MotorTestTCB,		
//				 (CPU_CHAR	* )"Imu Test task", 		
//                 (OS_TASK_PTR )MotorTest, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )MotorTest_TASK_PRIO,     	
//                 (CPU_STK   * )&MotorTest_TASK_STK[0],	
//                 (CPU_STK_SIZE)MotorTest_STK_SIZE/10,	
//                 (CPU_STK_SIZE)MotorTest_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,					
//                 (void   	* )0,				
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
//                 (OS_ERR 	* )&err);								 
								 
	OS_CRITICAL_EXIT();	//?????				 
	OSTaskDel((OS_TCB*)&StartTaskTCB,&err);
//	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//??????			 
}

