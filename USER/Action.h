#ifndef _ACTION_H_
#define _ACTION_H_
#include "includes.h"
#include "Motor.h"
#include "Infrared.h"
#include "Queue.h"
#include "physparams.h"
#include "pid.h"

extern OS_ERR ActionExErr;
extern OS_MSG_SIZE actionEndMsgSize;
extern OS_TCB ActionTaskTCB;

namespace MouseAction{
struct CorrsInfo
{
    unsigned int fwdEnd;
    float irSideDist;
    unsigned int turnWait;
    void Reset()
    {
        this->fwdEnd = 0;
        this->irSideDist = 0.0f;
        this->turnWait = 0;
    }
    CorrsInfo()
    {
        this->Reset();
    }

};

union WallStatus
{
    unsigned int msk;
    struct
    {
        uint16_t fwd    :1;   // 1: wall exist; 0: no wall
        uint16_t left   :1;
        uint16_t right  :1;
//		unsigned int flns	:1;
//		unsigned int frns	:1;
    } WallStatusStruct;
};
//extern volatile WallStatus cur_wall;
class ActMsg
{
public:
    enum MsgType
    {
        Action_ed     = 0x00010000,
        Action_ing    = 0x00020000
//        DoCorrection    = 0x00030000
    };
};

class Act
{
public: enum ActType
    {
				Corr     = 0x00000001,
        Null     = 0x01000000,
        Start    = 0x02000000,
        Stop     = 0x03000000,
        Back     = 0x04000000,
        Restart  = 0x05000000,
        Fwd      = 0x06000000,
        L90      = 0x07000000,
        R90      = 0x08000000,
        XLow     = 0x10000000,
        CRush    = 0x11000000,
        TRush    = 0x12000000,
        Turn     = 0x13000000,
        RushOut  = 0x14000000,
        L45i     = 0x15000000,
        L45o     = 0x16000000,
        R45i     = 0x17000000,
        R45o     = 0x18000000,
        RL90     = 0x19000000,
        RR90     = 0x1A000000,
        XL90     = 0x1B000000,
        XR90     = 0x1C000000,
        L135i    = 0x1D000000,
        L135o    = 0x1E000000,
        R135i    = 0x1F000000,
        R135o    = 0x20000000,
        L180     = 0x21000000,
        R180     = 0x22000000,
				TBack    = 0x23000000,
				RushStart= 0x24000000,
				MidUp		 = 0x25000000,
				HighUp	 = 0x26000000,
				RushHigh = 0x27000000,
				HighDown = 0x28000000,
				MidDown	 = 0x29000000,
				RushStop = 0x2A000000,
				RushLow  = 0x2B000000,
				RushMid  = 0x2C000000,
				XMid		 = 0x2D000000,
				XMidUp	 = 0x2E000000,
				XMidDown = 0x2F000000,
				XMidUp1	 = 0x30000000,
				XMidDown1 = 0x31000000,
				RotateL90 = 0x32000000,
				RotateR90 = 0x33000000,
				XMid1 = 0x34000000,
				HighUp1 = 0x35000000,
				HighDown1 = 0x36000000,
				MidUp1 = 0x37000000,
				MidDown1 = 0x38000000,
				RushMidLow = 0x39000000,
				RushHighLow = 0x3A000000
    };
};
void Init_Action();
int MotionCalcFwd(float v0, float v1, float s, float *vs);
int MotionCalcRotate(float ang, float mu, float *omgs);
int MotionCalcTurn(float v, float ang, float mu, float *omgs, float *requ);
float actHeadingDirCorrBySideIrSide(volatile WallStatus *wall);
void GetWallInfo();
void actPowerupDirCorr();
void actStart(bool corr);
unsigned int actStop(bool corr);
void actFwd(bool corr);
void actBack(WallStatus *wall, bool corr);
void actLR90(Act::ActType act, bool corr);
void actRotateL90(bool corr);
void actRotateR90(bool corr);
void actBack();
void actRestart(bool corr);
void actRestart();
void actTurnBack();
void actFwd();
void actCorrStop(void);
void actCorrTurnBack(void);
void actRushStart();
unsigned int actRushStop();
void actRush(float v1, float v2);
void actXRush(float v1, float v2);
void actL45i();
void actL45o();
void actR45i();
void actR45o();
void actR45o();
void actRL90();
void actRR90();
void actXL90();
void actXR90();
void actL135i();
void actL135o();
void actR135i();
void actR135o();
void actL180();
void actR180();
inline void ExcuteAction(Act::ActType x){
	OSTaskQPost(&ActionTaskTCB,(Act::ActType*)(x),4,OS_OPT_POST_FIFO,&ActionExErr);
	OSTaskQPend(0,OS_OPT_PEND_BLOCKING,&actionEndMsgSize,NULL,&ActionExErr);	
}
}
#endif

