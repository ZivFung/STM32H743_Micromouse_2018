#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "includes.h"
#include "physparams.h"
#include "Queue.h"
#include "usart.h"
#include "MotorPwm.h"
#include "icm20602.h"
#include "WheelEnc.h"
#include "Kalman1Var.h"
#include "Pid.h"
#include "Infrared.h"
#include "Action.h"
#include "app.h"
//float  BATV = 3.f;
namespace MotorControl{

#define MotorSpeedQueueLength PP::SeqArrayLen

#define VELOCITY_PID
//#define ANGULAR_VELOCITY_PID

//const float lvPidP = 1780.6355274452f;
//const float lvPidI = 26400.4896294783f;
//const float lvPidD = 0.f;
//const float lvPidN = 0.f;

//const float lvPidP = 2010.3355274452f;									//0.9f //32768
//const float lvPidI = 53900.4896294783f;
//const float lvPidD = 0.0f;
//const float lvPidN = 1.f;	

//const float lvPidP = 2010.3355274452f;									//0.9f //16384
//const float lvPidI = 53900.4896294783f;
//const float lvPidD = 0.051f;
//const float lvPidN = 1.f;	
	
//const float lvPidP = 1800.3355274452f;									//0.9f //16384
//const float lvPidI = 54050.4896294783f;
//const float lvPidD = 0.051f;
//const float lvPidN = 1.f;	
	
//const float lvPidP = 1670.3355274452f;									//0.9f //4096
//const float lvPidI = 37500.4896294783f;
//const float lvPidD = 0.05f;
//const float lvPidN = 1.f;	
	
//const float lvPidP = 1591.9355274452f;									//0.9f //1024
//const float lvPidI = 39101.4896294783f;
//const float lvPidD = 8.98f;
//const float lvPidN = 1.f;

//const float lvPidP = 1572.9355274452f;									//0.9f //1128
//const float lvPidI = 40121.4896294783f;
//const float lvPidD = 0.05f;
//const float lvPidN = 1.f;

const float lvPidP = 1669.9355274452f;									//0.9f //1128
const float lvPidI = 46121.4896294783f;
const float lvPidD = 0.05f;
const float lvPidN = 1.f;

//const float lvPidP = 1571.3355274452f;									//0.9f //1024
//const float lvPidI = 40121.4896294783f;
//const float lvPidD = 0.51f;
//const float lvPidN = 1.f;

//const float lvPidP = 1991.0355274452f;		//1970				//0.6		//1500
//const float lvPidI = 33528.93f;//33525.93f
//const float lvPidD = 400.f;//25.f
//const float lvPidN = 1.f;//2.f


//const float avPidP = 0.f;
//const float avPidI = 0.0f;
//const float avPidD = 0.0f;
//const float avPidN = 0.f;
//const float avPidP = 244.0618774845219f;
//const float avPidI = 786.3570730776f;
//const float avPidD = -1.5368095212600255f;
//const float avPidN = 15.816743973314f;
const float avPidP = 330.9618774845219f;
const float avPidI = 655.3570730776f;
const float avPidD = 0.f;
const float avPidN = 0.f;


const float YawCorrPidP = 1.f;
const float YawCorrPidI = 0.f;
const float YawCorrPidD = 0.f;
const float YawCorrPidN = 399.f;

typedef struct {
    volatile float Lv,Av;
}MotorSpeedDef;
enum MotorMode {Cal = 0, Run, Idle, Free, Test};

class MotorCtl{
private:
		void MortorTickTimerInit(u16 arr,u16 psc);
protected:
public:
	  MotorDuty PwmDuty;
    volatile float Accl,GyroZZero,AccelYZero,lvPidOut,avPidOut,rPwm,lPwm,desireAngle,diff;
		float LvAdj,OmgAdj,dOmgAdj;
    float DistanceAcc,AngleAcc;
		volatile int QueueEmptyOp;
		MotorSpeedDef DesireSpeed, NowSpeed,LastDesireSpeed;	
    MotorCtl();
		Queue *MotorDesireSpeed;
    Pid *LvPid,*AvPid;
    Kalman1Var *lvEstKal;
    ICM20602 *Imu;
    WheelEnc *WheelEncoder;
    MotorPwm *MPWM;
    volatile MotorMode MMode;
    uint8_t crashDet;

    void MotorRunTick();
    void MotorCalTick();
    void MotorIdleTick();
		void MotorTestTick();
};

inline float saturate(float v, float max, float min)
{
    return  v > max ? max :
            v < min ? min : v;
}

}

#endif
