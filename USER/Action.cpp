#include "Action.h"
extern InfraredControl::Infrared *IR;
extern MotorControl::MotorCtl *Motor;
extern OS_SEM MotorTick;
//extern Pid *yawPid;
volatile MouseAction::WallStatus cur_wall;


namespace MouseAction{
	
const float BackCorrPosPidP = 5.f;
const float BackCorrDirPosPidP = 5.f;
const float BackCorrYawPidP = 12.f;
const float StopCorrYawPidP = 18.f;
const float CtpdCorrGain = .08f;
const float StopCorrPosPidP = 6.f;
	
const float sqrt2 = 1.41421f;
//	
//void Init_Action(){
//    yawPid = new Pid(YawCorrPidP, YawCorrPidI, YawCorrPidD,YawCorrPidN, PP::Ts, -3142.0f, 3142.0f);
//}
//#define CORR_FWDEND_DIST_W2NW (cur_wall.WallStatusStruct.left? CORR_LFWDEND_DIST_W2NW : CORR_RFWDEND_DIST_W2NW)
//#define CORR_BACKANGLE_LRDIFF (wall->WallStatusStruct.left? CORR_LBACKANGLE_LRDIFF : CORR_RBACKANGLE_LRDIFF)
//#define CORR_BACKCENTER_ADJ (wall->WallStatusStruct.left? CORR_LBACKCENTER_ADJ : CORR_RBACKCENTER_ADJ)
float distZero;

	
	float MaxB(float a,float b){
	
	return  (a>=b)? a: b;
	}
	#define Max(a,b) (a>=b)? a: b;
void GetWallInfo()
{	
		if(MaxB(IR->IRInts.dFL,IR->IRInts.dFR) >= PP::FWD_WALL_TH)
			cur_wall.WallStatusStruct.fwd = 0;
		else if(MaxB(IR->IRInts.dFL,IR->IRInts.dFR) <= PP::FWD_WALL_TL)
			cur_wall.WallStatusStruct.fwd = 1;
		
		if(IR->IRInts.dSL >= PP::LEFT_WALL_TH)
			cur_wall.WallStatusStruct.left = 0;
		else if(IR->IRInts.dSL <= PP::LEFT_WALL_TL)
			cur_wall.WallStatusStruct.left = 1;
		
		if(IR->IRInts.dSR >= PP::RIGHT_WALL_TH)
			cur_wall.WallStatusStruct.right = 0;
		else if(IR->IRInts.dSR <= PP::RIGHT_WALL_TL)
			cur_wall.WallStatusStruct.right = 1;
}

void GetWallInfoRun()
{
		cur_wall.WallStatusStruct.fwd = (IR->IrDistFwd() < PP::FWD_WALL_TH_RUN);
		cur_wall.WallStatusStruct.left = (IR->IRInts.dSL < PP::LEFT_WALL_TH_RUN);
		cur_wall.WallStatusStruct.right = (IR->IRInts.dSR < PP::RIGHT_WALL_TH_RUN);
//						printf("DSr:%f\r\n",IR->IRInts.dSR);
//					printf("DSl:%f\r\n",IR->IRInts.dSL);
}

void GetWallInfoHalfWall()
{
		cur_wall.WallStatusStruct.fwd = (IR->IrDistFwd() < PP::FWD_WALL_TH);
		cur_wall.WallStatusStruct.left = (IR->IRInts.dSL < 0.09f);
		cur_wall.WallStatusStruct.right = (IR->IRInts.dSR < 0.09f);
}

int MotionCalcFwd(float v0, float v1, float s, float *vs)
{
    s *= 2.0f;
    float t = s / (v0 + v1);
    float a = (v1*v1-v0*v0)/s;
		int i, imax = t / PP::Ts;
    for(i = 1; i <= imax; i++)
    {
        vs[i - 1] = v0 + i * PP::Ts * a;
    }
    vs[i - 1] = v1;
		if(s < 0)return 0;
		else return imax + 1;
}

int MotionCalcRotate(float ang, float mu, float *omgs)
{
    float beta = PP::g * mu / PP::W;
    float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
    float omg = 0.0f, tht = 0.0f;
    int n = 0, i;
    do
    {
        omg += beta * PP::Ts;
        omgs[n++] = ang > 0.0f ? omg : -omg;
        tht += omg * PP::Ts;
    }while(tht < ha);
    for(i = n - 2; i >= 0; i--)
    {
        omgs[2 * n - 2 - i] = omgs[i];
    }
    omgs[2 * n - 1] = 0.0f;
    return 2 * n;
}

int MotionCalcTurn(float v, float ang, float mu, float *omgs,float *requ)
{

    if(v < 0.001f && v > -0.001f)
        return MotionCalcRotate(ang, mu, omgs);

        float ha = ang > 0.0f ? ang / 2.0f : ang / -2.0f;
        float omg = 0.0f, tht = 0.0f, x = 0.0f, y = 0.0f;
        float k = sqrt(PP::W*PP::W - mu*mu * PP::H*PP::H);
        float tv = PP::W*PP::W/k/v * acosf(mu*PP::H/PP::W);
        float c = mu*PP::g*PP::W/v/k;
        float t, l;
        int n = 0, i, imax = tv/PP::Ts;
        for(i = 1; i <= imax; i++)
        {
            t = PP::Ts * i;
            l = t*v*k/PP::W/PP::W;
            omg = mu*c*PP::H/k*(cosf(l)-1.0f)+c*sinf(l);
            omgs[n++] = ang > 0.0f ? omg : -omg;
            tht += omg * PP::Ts;
            x += v * PP::Ts * cosf(tht);
            y += v * PP::Ts * sinf(tht);
            if(tht >= ha)
                break;
        }
        if(i > imax)   // large turn
        {
            do
            {
                omgs[n++] = ang > 0.0f ? omg : -omg;
                tht += omg * PP::Ts;
                x += v * PP::Ts * cosf(tht);
                y += v * PP::Ts * sinf(tht);
            } while(tht < ha);
        }
        for(i = n - 2; i >= 0; i--)
        {
            omgs[2 * n - 2 - i] = omgs[i];
        }
        omgs[2 * n - 1] = 0.0f;
        /*lastRequ =*/ *requ = x / tanf(tht) + y;
        //lastCnt = 2 * n;
        //memcpy(lastOmgs, omgs, lastCnt * sizeof(float));
        return 2 * n;
}

float actHeadingDirCorrBySideIrSide(volatile WallStatus *wall)
{
   float yaw;
	 uint8_t flag = 0;
   if(wall->WallStatusStruct.left)
   {
       yaw = IR->IRInts.YawL;
       if(wall->WallStatusStruct.right)
       {
           yaw += IR->IRInts.YawR;
           yaw *= 0.5f;
						flag = 1;
       }
   }
   else if(wall->WallStatusStruct.right)
       yaw = IR->IRInts.YawR;
   else
       yaw = 0.0f;
	
		if(yaw > .9f)
			yaw = .9f;
		else if(yaw < -.9f)
			yaw = -.9f;
//		else if((yaw > -.005f)&&(yaw < .005f))
//			yaw = 0.f;
//	 if(flag){
//		if(yaw > .5f)
//			yaw = .5f;
//		else if(yaw < -.5f)
//			yaw = -.5f;
//		else if((yaw > -.03f)&&(yaw < .03f))
//			yaw = 0.f;
//	 }

	 return (flag)? (-yaw * 2.35f):(-yaw * 3.05f);
}

float actBackDirCorrBySideIrSide(volatile WallStatus *wall)
{
   float yaw;

   if(wall->WallStatusStruct.left)
   {
       yaw = IR->IRInts.YawL;
       if(wall->WallStatusStruct.right)
       {
           yaw += IR->IRInts.YawR;
           yaw *= 0.5f;
       }
   }
   else if(wall->WallStatusStruct.right)
       yaw = IR->IRInts.YawR;
   else
       yaw = 0.0f;
	
//		if(yaw > .5f)
//			yaw = .5f;
//		else if(yaw < -.5f)
//			yaw = -.5f;
//		else if((yaw > -.5f)&&(yaw < .5f))
//			yaw = 0.f;

		return -yaw * 2.75f;
}

void actPowerupDirCorr(){
		OS_ERR err;
		float yaw;
		do{
			yaw = IR->IRInts.YawL;
			yaw += IR->IRInts.YawR;
			yaw *= 0.5f;
			Motor->dOmgAdj =-6.f * yaw;
			OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err);
		}while(yaw > 0.05f || yaw < -0.05f);
		Motor->dOmgAdj = .0f;
}
u32 reportWall()
{
//	u32 msg;
//	WallStatus w;
//	w.mask = 0;
//    w.fwd = TskIr::IrBins.Fwd;
//    w.left = TskIr::IrBins.LS;
//    w.right = TskIr::IrBins.RS;

//    msg = TskSolve::ActMsg::ReportWall | (w.mask << 20);
//    msgsnd(TskSolve::MsgFromActId, (void *)msg, 4, IPC_NOWAIT);

//    dbgPutWall(&w);
//    return w.mask;
}
//float actFwdDisCorrByFwdIr(volatile WallStatus *wall)
//{
//		float dist;

//		dist = 0.5f * (IR->IRInts.dFL + IR->IRInts.dFR) - (pp.GridSize - pp.WallThick)/2.f;

//		if(dist > 0.007f)dist = 0.007f;
//		else if(dist < -0.007f)dist = -0.007f;
//	
//		return (dist * 12.f);
//}



CCMRAM float v_s[1024], o_s[1024];


#define WALL_STATUS_MASK_L 1
#define WALL_STATUS_MASK_F 2
#define WALL_STATUS_MASK_R 4

inline void actWaitSeqEnd()
{
		OS_ERR err;
    while(Motor->MotorDesireSpeed->Length() > 1)
    {
    	OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
    }
}
//																										&cur_wall, PP::SearchSpeed, PP::SearchSpeed, 0.74f, .083f, wallsts

int actFwdEndCorrBySideWallDisappear(volatile WallStatus *wall, float v0, float v1,float d, uint16_t wallsts)
{
		int len, i;
		MotorControl::MotorSpeedDef s;
	
		if((wall->WallStatusStruct.left == 0)&&(wallsts&WALL_STATUS_MASK_L))
		{
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdL, v_s);
				
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Left WallDisappear\r\n");
				return 1;
		}else if((wall->WallStatusStruct.right == 0)&&(wallsts&WALL_STATUS_MASK_R)){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdR, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Right WallDisappear\r\n");
				return 1;
		}
		return 0;
}

int actFwdEndCorrBySideWallDisappearAndShow(volatile WallStatus *wall, float v0, float v1,float d, uint16_t wallsts)
{
		int len, i;
		MotorControl::MotorSpeedDef s;
	
		if((wall->WallStatusStruct.left == 0)&&(wallsts&WALL_STATUS_MASK_L))
		{
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdL, v_s);
				
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Left Disappear\r\n");
				return 1;
		}else if((wall->WallStatusStruct.right == 0)&&(wallsts&WALL_STATUS_MASK_R)){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdR, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Right Disappear\r\n");
				return 1;
		}
		else if((wall->WallStatusStruct.right == 1)&&(!(wallsts&WALL_STATUS_MASK_R))){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdR_Show, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Right show\r\n");
				return 1;
		}
		else if((wall->WallStatusStruct.left == 1)&&(!(wallsts&WALL_STATUS_MASK_L))){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdL_Show, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Left show\r\n");
				return 1;
		}
		return 0;
}

int actFwdEndCorrBySideWallDisappearAndShowNowall(volatile WallStatus *wall, float v0, float v1,float d, uint16_t wallsts)
{
		int len, i;
		MotorControl::MotorSpeedDef s;
		if((wall->WallStatusStruct.right == 1)&&(!(wallsts&WALL_STATUS_MASK_R))){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdR_Show_Nowall, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Right show\r\n");
				return 1;
		}
		else if((wall->WallStatusStruct.left == 1)&&(!(wallsts&WALL_STATUS_MASK_L))){
				// clear motion seq
				Motor->MotorDesireSpeed->Clear();
				// add
				len = MotionCalcFwd(v0, v1, SCP::FwdL_Show_Nowall, v_s);
			
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
//				printf("Left show\r\n");
				return 1;
		}
		return 0;
}

void actStart(bool corr = true)
{
		OS_ERR err;
		int i, len;
		Motor->QueueEmptyOp = 0;
		uint16_t wallsts=0;
		GetWallInfo();
		distZero = Motor->DistanceAcc;
	
		len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::StartAcclDist, v_s);
    MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
    for(i = 0; i < len; i++)
    {
        s.Lv = v_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }

    len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed,PP::StartTotalDist - PP::StartAcclDist, v_s);
		
    s.Av = 0.f;
    for(i = 0; i < len; i++)
    {
        s.Lv = v_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
    if(corr)
    {
		while(Motor->DistanceAcc - distZero < SCP::StartYawEnd)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
		}
		Motor->dOmgAdj = 0.0f;
//		yawPid->Reset();
		
		wallsts = (cur_wall.WallStatusStruct.left?WALL_STATUS_MASK_L:0)|(cur_wall.WallStatusStruct.right?WALL_STATUS_MASK_R:0);
		while(Motor->DistanceAcc - distZero < SCP::StartDisEnd)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			GetWallInfo();
			if(actFwdEndCorrBySideWallDisappear(&cur_wall,
					Motor->NowSpeed.Lv, PP::SearchSpeed,
					/*CORR_FWDEND_DIST_W2NW,*/
					Motor->DistanceAcc - distZero,wallsts))
				break;
		}
    }
	while(Motor->DistanceAcc - distZero < SCP::StartReport)
	{
		OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	}
    reportWall();

    actWaitSeqEnd();
	

}

void actFwd(bool corr = false)
{
//    float vs[SEQARRAY_LEN];
    int i, len;
		OS_ERR err;
		Motor->QueueEmptyOp = 0;
		uint16_t wallsts=0;
		volatile MotorControl::MotorSpeedDef s;
    float distZero = Motor->DistanceAcc;
    WallStatus wall;
    bool fwdDistCorred = false;
    bool reported = false;
    float dist, lastDist = distZero;
		GetWallInfo();
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
//    INT8U err;

#if DBG_PRINT_ACT_INFO > 0
    sprintf(actionDbgString, "Fwd, %1d%1d%1d\n", wall.left, wall.fwd, wall.right);
    DbgPuts(actionDbgString);
#endif
		s.Av = 0.f;
		len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::GridSize + SCP::FwdLengthCorr, v_s);
    for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		//printf("des len:%d",len);
    if(corr)
    {
		if(cur_wall.WallStatusStruct.left || cur_wall.WallStatusStruct.right)  // side wall exist
		{
			while(Motor->MotorDesireSpeed->Length() > 1)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				dist = Motor->DistanceAcc - distZero;
				if(dist < SCP::FwdYawEnd)
				{
					Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
				}
				if(lastDist < SCP::FwdYawEnd && dist >= SCP::FwdYawEnd)
				{
					Motor->dOmgAdj = 0.0f;
				}
				if(SCP::FwdYawEnd < dist)
				{
					if(!fwdDistCorred)
					{
						wallsts = (cur_wall.WallStatusStruct.left?WALL_STATUS_MASK_L:0)|(cur_wall.WallStatusStruct.right?WALL_STATUS_MASK_R:0);
						GetWallInfo();
						//Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
						if(actFwdEndCorrBySideWallDisappearAndShow(&cur_wall, PP::SearchSpeed, PP::SearchSpeed, /*CORR_FWDEND_DIST_W2NW, */Motor->DistanceAcc - distZero,wallsts))
							fwdDistCorred = true;
					}
				}
				if(lastDist < SCP::FwdReport && dist >= SCP::FwdReport)
				{
					reportWall();
					reported = true;
				}
				lastDist = dist;
			}
		}
		else    // may be centipede
		{
//			dist = Motor->DistanceAcc - distZero;
//			while(dist < PP::GridSize + SCP::FwdLengthCorr - 0.02f)
//			{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				dist = Motor->DistanceAcc - distZero;
////				printf("fifolen:%d",Motor->MotorDesireSpeed->Length());
////				printf("Des Sp:%f",Motor->DesireSpeed.Lv);
////				if(dist < SCP::FwdYawEnd)
////				{
////					Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
////				}
////				if(lastDist < SCP::FwdYawEnd && dist >= SCP::FwdYawEnd)
////				{
////					Motor->dOmgAdj = 0.0f;
////				}
//					if(!fwdDistCorred)
//					{
//						wallsts = (cur_wall.WallStatusStruct.left?WALL_STATUS_MASK_L:0)|(cur_wall.WallStatusStruct.right?WALL_STATUS_MASK_R:0);
//						GetWallInfo();
//						//Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
//						if(actFwdEndCorrBySideWallDisappearAndShowNowall(&cur_wall, PP::SearchSpeed, PP::SearchSpeed, /*CORR_FWDEND_DIST_W2NW, */Motor->DistanceAcc - distZero,wallsts))
//						{
//							fwdDistCorred = true;
////							LED_B_Toggle;
////							printf("Wall Show");
//						}
//					}
//			}
//			actWaitSeqEnd();
			
			int lfInt = IR->IRInts.SL, rfInt = IR->IRInts.SR;
			int lfIntLast = 0, rfIntLast = 0, lFallCnt = 0, rFallCnt = 0;
			float lfMaxDist = -1.0f, rfMaxDist = -1.0f;
			float angErr, omgMax = 0.f;
			while(Motor->MotorDesireSpeed->Length() > 1)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				dist = Motor->DistanceAcc - distZero;
				if(SCP::CtpdStatBgn < dist && dist < SCP::CtpdYawBgn)
				{
					lfInt = IR->IRInts.SL;
					rfInt = IR->IRInts.SR;
					if(lfMaxDist < 0.0f)
					{
						if(lfInt < lfIntLast - 25)
						{
							if(++lFallCnt == 2)
								lfMaxDist = dist;
						}
						else lFallCnt = 0;
					}
					if(rfMaxDist < 0.0f)
					{
						if(rfInt < rfIntLast - 25)
						{
							if(++rFallCnt == 2)
								rfMaxDist = dist;
						}
						else rFallCnt = 0;
					}
					lfIntLast = lfInt;
					rfIntLast = rfInt;
				}
				if(lastDist < SCP::CtpdYawBgn && dist >= SCP::CtpdYawBgn)
				{
					if(lfMaxDist > 0.0f && rfMaxDist > 0.0f)
						angErr = (lfMaxDist - rfMaxDist) / PP::GridSize;
					else
						angErr = 0.0f;
					omgMax = angErr * (CtpdCorrGain
					/ ((SCP::CtpdYawEnd - SCP::CtpdYawBgn) / PP::SearchSpeed));
				}
				if(SCP::CtpdYawBgn < dist && dist < SCP::CtpdYawEnd)
				{
					Motor->dOmgAdj = -omgMax;
				}
				if(SCP::CtpdYawEnd < dist)
				{
					Motor->dOmgAdj = omgMax;
				}
				if(lastDist < SCP::FwdReport && dist >= SCP::FwdReport)
				{
					reportWall();
					reported = true;
				}
				lastDist = dist;
			}
//			Motor->dOmgAdj = 0.0f;
	//        actHDirPid->Reset();
	//        actWaitSeqEnd();
		}
    }
    else
    {
		while(Motor->MotorDesireSpeed->Length() > 1)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			dist = Motor->DistanceAcc - distZero;
			if(lastDist < SCP::FwdReport && dist >= SCP::FwdReport)
			{
				reportWall();
				reported = true;
			}
			lastDist = dist;
		}
    }
    if(!reported)
    	reportWall();
		
}

void actBack(WallStatus *wall, bool corr = false)
{
		OS_ERR err;
    int i, len;
		//GetWallInfo();
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
		Motor->QueueEmptyOp = 1;
		MotorControl::MotorSpeedDef s;
		s.Lv = 0.f;
#if DBG_PRINT_ACT_INFO > 0
    sprintf(actionDbgString, "Back, %1d%1d%1d\n", wall->left, wall->fwd, wall->right);
    DbgPuts(actionDbgString);
#endif
		float angleaccZero = 0.f;
		float tht = wall->WallStatusStruct.left? PP::Pi / 2.f : -PP::Pi / 2.f;
	
    len = MotionCalcRotate(tht, 0.7f * PP::Mu, o_s);
//		while(Motor->)
	  angleaccZero = Motor->AngleAcc;
    for(i = 0; i < len; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		
    actWaitSeqEnd();
		//printf("Back1;%f\r\n",Motor->AngleAcc - angleaccZero);
		if(corr && (wall->WallStatusStruct.left || wall->WallStatusStruct.right))
		{
			//printf("Back Corr\r\n");
			float pos;
			float yaw;
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
//		sleep(50);
			//printf("pos:%f\r\n",PP::CenterToWall - IR->IrDistFwd());
			while(
				fabsf(PP::CenterToWall - IR->IrDistFwd() - SCP::Pos) > 0.003f )
			{
				pos = PP::CenterToWall - IR->IrDistFwd() - SCP::Pos;
				//printf("pos:%f\r\n",PP::CenterToWall - IR->IrDistFwd());
				Motor->LvAdj = BackCorrPosPidP * MotorControl::saturate( - pos, .04f, -.04f);

			}
			Motor->LvAdj = 0.f;
			while(
				fabsf(IR->IRInts.YawFLR) > 0.6f * PP::Pi/180.f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				yaw = IR->IRInts.YawFLR;
				//printf("YawFLR:%f\r\n",yaw);
				Motor->dOmgAdj = BackCorrYawPidP * MotorControl::saturate(SCP::Yaw - yaw, .035f, -.035f);
			}
			Motor->dOmgAdj = 0.f;
		}
		len = MotionCalcRotate(tht,0.7f * PP::Mu, o_s);
		Motor->MMode = MotorControl::Cal;
//		while((fabs(Motor->NowSpeed.Av) > 0.002f) || (fabs(Motor->NowSpeed.Lv) > 0.005f)){}
//		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		while(Motor->MMode != MotorControl::Run){};
		angleaccZero = Motor->AngleAcc;
    for(i = 0; i < len; i++) {s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		actWaitSeqEnd();
		float AngleErr = tht - (Motor->AngleAcc - angleaccZero);
		if(AngleErr >= 0)
		s.Av = 0.5f;
		else s.Av = -0.5f;
		len = AngleErr /PP::Ts/s.Av;
		for(i = 0; i < len; i++) {Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		actWaitSeqEnd();
		//printf("Back2;%f\r\n",Motor->AngleAcc - angleaccZero);
//		if(IR->IRInts.dSL < 0.09f && IR->IRInts.dSR < 0.09f ){
//			GetWallInfoHalfWall();
//			while(fabsf(IR->IRInts.YawL + IR->IRInts.YawR) > .08f)
//			{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				Motor->dOmgAdj = 2.f * actBackDirCorrBySideIrSide(&cur_wall);
//				printf("2\r\n");
//			}
//			
//		}
//		else if(IR->IRInts.dSL < 0.09f || IR->IRInts.dSR < 0.09f){
////				cur_wall.WallStatusStruct.left = IR->IRInts.dSL < 0.09f;
////				cur_wall.WallStatusStruct.right = IR->IRInts.dSL < 0.09f;
//				GetWallInfoHalfWall();
//			while(cur_wall.WallStatusStruct.left? fabsf(IR->IRInts.YawL) : fabsf(IR->IRInts.YawR) > .08f)
//				{
//					OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//					Motor->dOmgAdj = 2.f * actBackDirCorrBySideIrSide(&cur_wall);
//	//				printf("2\r\n");
//				}
//		}
//		else if(IR->IRInts.dSL < 0.09f || IR->IRInts.dSR < 0.09f ){
//			GetWallInfoHalfWall();
//			if(wall->WallStatusStruct.left )printf("left\r\n");
//			else printf("Right\r\n");
//			while(wall->WallStatusStruct.left ? fabsf(IR->IRInts.YawL) : fabsf(IR->IRInts.YawR)   > .5f)
//			{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				Motor->dOmgAdj = actBackDirCorrBySideIrSide(&cur_wall);
//				printf("YawL:%f\r\n",IR->IRInts.YawL);
//				printf("YawR:%f\r\n",IR->IRInts.YawR);
//			}
//			
//		}
//		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
//		GetWallInfo();		
//		if(corr && (cur_wall.WallStatusStruct.left && cur_wall.WallStatusStruct.right)){
//			float pos;
//			float yaw;
//			while(fabsf(IR->IRInts.YawL + IR->IRInts.YawR) > .15f)
//			{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				Motor->dOmgAdj = 0.2f * actBackDirCorrBySideIrSide(&cur_wall);
//			}
//		}
//		else if(corr && (cur_wall.WallStatusStruct.left || cur_wall.WallStatusStruct.right)){
//			float pos;
//			float yaw;
//			while(fabsf(cur_wall.WallStatusStruct.left?IR->IRInts.YawL : IR->IRInts.YawR) > .15f)
//			{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				Motor->dOmgAdj = 0.2f * actBackDirCorrBySideIrSide(&cur_wall);
//			}
//		}
//		Motor->dOmgAdj = 0.f;
		Motor->dOmgAdj = 0.f;
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    reportWall();

#if DBG_PRINT_ACT_INFO > 0
    DbgPuts("\t##\n");
#endif
}

unsigned int actStop(bool corr = false)
{
//    float vs[SEQARRAY_LEN];
    int i, len;
		OS_ERR err;
		Motor->QueueEmptyOp = 0;
    float distZero = Motor->DistanceAcc;
		GetWallInfo();
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		len = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, PP::StopTotalDist - PP::StopAccDist, v_s);
    for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}

    if(corr && cur_wall.WallStatusStruct.fwd)
    {
			while((Motor->DistanceAcc - distZero) <= SCP::StopYawEnd && (IR->IrDistFwd() > PP::CenterToWall + PP::StopAccDist - SCP::Stop))
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
				if(Motor->MotorDesireSpeed->Length() == 0)
				{
					Motor->LvAdj = BackCorrDirPosPidP * (SCP::StopYawEnd - (Motor->DistanceAcc - distZero));
				}
			}
			Motor->LvAdj = 0.f;
			Motor->dOmgAdj = 0.0f;
//		yawPid->Reset();
    }
		if(corr && cur_wall.WallStatusStruct.fwd)
		{
			float pos;
			float yaw;
			while(IR->IrDistFwd() > PP::CenterToWall + PP::StopAccDist - SCP::Stop)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				if(Motor->MotorDesireSpeed->Length() == 0)
				{
					Motor->LvAdj = BackCorrDirPosPidP * (IR->IrDistFwd() - (PP::CenterToWall + PP::StopAccDist - SCP::Stop));
				}
			}
			Motor->MotorDesireSpeed->Clear();
			len = MotionCalcFwd(PP::SearchSpeed, 0.0f, PP::StopAccDist, v_s);
			for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
			Motor->QueueEmptyOp = 1;
			actWaitSeqEnd();
			
			while(fabs(Motor->NowSpeed.Lv) > 0.005f){
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			}
			
			while(fabsf(IR->IrDistFwd()-PP::CenterToWall + SCP::Stop) > 0.005f){
				Motor->LvAdj = BackCorrDirPosPidP * (IR->IrDistFwd()-PP::CenterToWall + SCP::Stop);
			}
			Motor->LvAdj = 0.f;
		
			while(fabsf(IR->IRInts.YawFLR - SCP::Yaw) > 0.6f * PP::Pi/180.f /*||
//				fabsf(pp.CenterToWall - TskIr::IrDistFwd() - scp.BackCorrAdj.Pos) > 0.001f*/)
//			while(fabsf(IR->IRInts.YawL + IR->IRInts.YawR) > .005f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				yaw = IR->IRInts.YawFLR;
//			pos = pp.CenterToWall - TskIr::IrDistFwd();
//			TskMotor::AdjVO.Velocity = BackCorrPidP * saturate(scp.BackCorrAdj.Pos - pos, -.005f, .005f);
				Motor->dOmgAdj = StopCorrYawPidP * MotorControl::saturate(SCP::Yaw - yaw, .04f, -.04f);
			}
			Motor->LvAdj = 0.f;
			Motor->dOmgAdj = 0.f;
		}	
		else
		{
			len = MotionCalcFwd(PP::SearchSpeed, 0.0f, PP::StopAccDist, v_s);
			for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
			Motor->QueueEmptyOp = 1;
		}
    actWaitSeqEnd();
		if(corr && (!cur_wall.WallStatusStruct.fwd)){
			s.Av = 0.0f;
			for(i = 0; i < 50; i++) {s.Lv = 0.0f; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
			while(fabs(Motor->NowSpeed.Lv) > 0.01f){
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			}
			Motor->MotorDesireSpeed->Clear();
		}
    reportWall();

    // stay still for 700ms, refine imu zeros
//    sleep(750);
		Motor->MotorDesireSpeed->Clear();
		Motor->MMode = MotorControl::Cal;
//		OSTimeDlyHMSM(0, 0, 0, 750, OS_OPT_TIME_HMSM_STRICT, &err);

#if DBG_PRINT_ACT_INFO > 0
    DbgPuts("\t##\n");
#endif

    return cur_wall.msk;
}


void actLR90(Act::ActType act, bool corr = false)
{
//    float vs[SEQARRAY_LEN], os[SEQARRAY_LEN], requ;
		OS_ERR err;
		float requ;
		volatile Act::ActType actTemp = act;
		int i, vslen, oslen;
    float distZero = Motor->DistanceAcc,angleaccZero,AngleErr = 0.f;
		Motor->QueueEmptyOp = 0;
		GetWallInfo();
//		if(cur_wall.WallStatusStruct.fwd)printf("fwd wall\r\n");
		MouseAction::WallStatus AfterTrunWall;
	
    Motor->dOmgAdj = 0.f; //actHDirPid->Reset();
//    INT8U err;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		if(act == Act::L90)
			oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ? (PP::Pi * .5f): -PP::Pi * .5f,3.5f * PP::Mu, o_s, &requ);
		else 
			oslen = MotionCalcTurn(PP::SearchSpeed, act == Act::L90 ? PP::Pi * .5f: -PP::Pi * .5f,3.5f * PP::Mu, o_s, &requ);
		
//		printf("req:%f\r\n",requ);
//		printf("\r\n");
		float straightPre = (PP::GridSize * 0.5f - requ)
				+ (act == Act::L90 ? BCP::LPre : BCP::RPre);
		float straightPost = (PP::GridSize * 0.5f - requ)
				+ (act == Act::L90 ? BCP::LPost : BCP::RPost);
    vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPre, v_s);
    for(i = 0; i < vslen; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}

//    while(TskMotor::DistAcc - distZero < GEO_FWD)
//    {
//        OSSemPend(ActionTick, 0, &err);
//        OSSemSet(ActionTick, 0, &err);
//        TskMotor::AdjVO.Omega = actHeadingDirCorrBySideIrSide(&wall);
//    }
//    TskMotor::AdjVO.Omega = 0.0f;
    ////// comment @2015-09-16 16:37
    //actHDirPid->Reset();

    if(corr)
    {
    	if(cur_wall.WallStatusStruct.fwd)
    	{
				//printf("Fwd Wall\r\n");
			while(
					(actTemp == Act::L90 ? IR->IRInts.dFL : IR->IRInts.dFR)>(requ + (PP::CenterToWall - ((act == Act::L90) ? SCP::L : SCP::R))))
			{
//				printf("dr:%f\r\n",IR->IRInts.dFR);
//				printf("aim :%f\r\n",requ + (PP::CenterToWall - (act == Act::L90 ? SCP::L : SCP::R)));
//				printf("3\r\n");
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				if(Motor->DistanceAcc - distZero < SCP::TrunYawEnd)
//				{
////					Motor->dOmgAdj = StopCorrYawPidP * MotorControl::saturate(SCP::Yaw - IR->IRInts.YawFLR, .02f, -.02f);
//					Motor->dOmgAdj = 1.2f * actHeadingDirCorrBySideIrSide(&cur_wall);
//				}
//				else
//					Motor->dOmgAdj = 0.f;
//				if(Motor->MotorDesireSpeed->Length() == 0){
//					Motor->LvAdj = 1.5f * (requ + (PP::CenterToWall - (act == Act::L90 ? SCP::L : SCP::R)));
//				}
			}
//				printf("dr:%f\r\n",IR->IRInts.dFR);
//				printf("aim :%f\r\n",requ + (PP::CenterToWall - (act == Act::L90 ? SCP::L : SCP::R)));
			Motor->MotorDesireSpeed->Clear();
			s.Lv = PP::SearchSpeed;
			angleaccZero = Motor->AngleAcc;
			for(i = 0; i < oslen; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
			Motor->LvAdj = 0.f;
			Motor->dOmgAdj = 0.f;
    	}
    	else
    	{
				while((Motor->DistanceAcc - distZero) < ((PP::CenterToWall - requ) + (actTemp == Act::L90 ? (SCP::L + 0.0104f) : (SCP::R + 0.0117f))))//L:21
        {
						OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
						//Motor->dOmgAdj = 1.2f * actHeadingDirCorrBySideIrSide(&cur_wall);
        }
				Motor->MotorDesireSpeed->Clear();
				Motor->dOmgAdj = 0.f;
				angleaccZero = Motor->AngleAcc;
				for(i = 0; i < oslen; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
    	}
    }
    else
    {
        for(i = 0; i < oslen; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
    }
    actWaitSeqEnd();
		AngleErr = act == Act::L90 ? (PP::Pi * .5f): -PP::Pi * .5f - (Motor->AngleAcc - angleaccZero);
		reportWall();
		
		
		if(corr){
			u16 wallsts;
			u16 AngleCorrLen = 0;
//			GetWallInfo();
			if(act == Act::L90){
				wallsts = (cur_wall.WallStatusStruct.fwd?WALL_STATUS_MASK_R:0);
			}
			else {
				wallsts = (cur_wall.WallStatusStruct.fwd?WALL_STATUS_MASK_L:0);
			}
			vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s);
			if(AngleErr >= 0)s.Av = 0.15f;
			else s.Av = -0.15f;
			//GetWallInfo();
			AngleCorrLen = AngleErr / PP::Ts / s.Av; 
			
			for(i = 0; i < vslen; i++) {
				s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				if(i == (AngleCorrLen-1))s.Av = 0.0f;
			}
//			while(Motor->DistanceAcc - distZero < SCP::AfterTrunReadyDist){
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//			}
			while((Motor->DistanceAcc - distZero) < (straightPost) && (Motor->DistanceAcc - distZero) > 0.01f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				GetWallInfo();
				Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
//				if(actFwdEndCorrBySideWallDisappear(&cur_wall,
//						Motor->NowSpeed.Lv, PP::SearchSpeed,
//						/*CORR_FWDEND_DIST_W2NW,*/
//						Motor->DistanceAcc - distZero,wallsts))
//				break;
			}
		}
		else{
			vslen = MotionCalcFwd(PP::SearchSpeed, PP::SearchSpeed, straightPost, v_s);
			s.Av = 0.f;
			for(i = 0; i < vslen; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		}
//		if(corr){
//			while((Motor->DistanceAcc - distZero) < 0.005f){}
//			GetWallInfo();
////		if(corr){
//			distZero = Motor->DistanceAcc;
//			while((Motor->DistanceAcc - distZero) < SCP::TurnDirEnd){
//			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//			Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
//			}
//		}
		
		Motor->dOmgAdj = 0.f;
    actWaitSeqEnd();
		
}

void actRestart(bool corr = false)
{
//    float vs[SEQARRAY_LEN];
		OS_ERR err;
		uint16_t wallsts=0;
		Motor->QueueEmptyOp = 0;
    int i, len;
    float distZero = Motor->DistanceAcc;
		MotorControl::MotorSpeedDef s;
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
		GetWallInfo();
#if DBG_PRINT_ACT_INFO > 0
    DbgPuts((char *)"Restart\n");
#endif
//		len = MotionCalcFwd(0.0f, PP::SearchSpeed, PP::RestartAccDist, v_s);
//		for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		len = MotionCalcFwd(0, PP::SearchSpeed, PP::RestartDist, v_s);
		float omgadjtemp = 0.f; 
    for(i = 0; i < len; i++) {s.Lv = v_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		if(corr){
		while(Motor->DistanceAcc - distZero < 0.005f)
    {
    	OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//			omgadjtemp = 0.7f * actHeadingDirCorrBySideIrSide(&cur_wall);
////			if(omgadjtemp > 0 && omgadjtemp < 0.1f)omgadjtemp = 0.f;
//			Motor->dOmgAdj = omgadjtemp - 0.2f;
    }
		Motor->dOmgAdj = 0.0f;
		
		wallsts = (cur_wall.WallStatusStruct.left?WALL_STATUS_MASK_L:0)|(cur_wall.WallStatusStruct.right?WALL_STATUS_MASK_R:0);
		while(Motor->DistanceAcc - distZero < SCP::RestartDisEnd)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			GetWallInfo();
			//Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
			if(actFwdEndCorrBySideWallDisappear(&cur_wall,
					Motor->NowSpeed.Lv, PP::SearchSpeed,
					/*CORR_FWDEND_DIST_W2NW,*/
					Motor->DistanceAcc - distZero,wallsts)){

				break;}
		}
		}
		Motor->dOmgAdj = 0.0f;
		
//		while(Motor->DistanceAcc - distZero < SCP::RestartReport){
//			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//			Motor->dOmgAdj = actHeadingDirCorrBySideIrSide(&cur_wall);
//		}
//		Motor->dOmgAdj = 0.0f;
    reportWall();

    actWaitSeqEnd();

#if DBG_PRINT_ACT_INFO > 0
    DbgPuts("\t##\n");
#endif
}

void actRotateL90(bool corr = false){
		OS_ERR err;
		float requ;
    int i, len;
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
		MotorControl::MotorSpeedDef s;s.Lv = 0.f;
		float AngZero = Motor->AngleAcc;
		float tht = PP::Pi / 2.f ;
		Motor->QueueEmptyOp = 1;

    len = MotionCalcRotate(tht, PP::Mu, o_s);
//		while(Motor->)
    for(i = 0; i < len; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		if(corr){
			while((Motor->AngleAcc - AngZero) < PP::Pi / 2.f - 0.1){
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				if(Motor->MotorDesireSpeed->Length() == 0){
					Motor->dOmgAdj = ((PP::Pi / 2.f - 0.1) - (Motor->AngleAcc - AngZero));
				}
			}
		}
		Motor->dOmgAdj = 0.f;
		actWaitSeqEnd();
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		GetWallInfo();
		if(corr && (cur_wall.WallStatusStruct.left && cur_wall.WallStatusStruct.right)){
			float pos;
			float yaw;
			while(fabsf(IR->IRInts.YawL + IR->IRInts.YawR) > .005f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				Motor->dOmgAdj = 2.0f * actBackDirCorrBySideIrSide(&cur_wall);
			}
		}
		else if(corr && (cur_wall.WallStatusStruct.left || cur_wall.WallStatusStruct.right)){
			float pos;
			float yaw;
			while(fabsf(cur_wall.WallStatusStruct.left?IR->IRInts.YawL : IR->IRInts.YawR) > .0025f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				Motor->dOmgAdj = 2.0f * actBackDirCorrBySideIrSide(&cur_wall);
			}
		}
		Motor->dOmgAdj = 0.f;
}
void actRotateR90(bool corr = false){
		OS_ERR err;
		float requ;
    int i, len;
		float AngZero = Motor->AngleAcc;
		Motor->QueueEmptyOp = 1;
    Motor->dOmgAdj = 0.0f; //actHDirPid->Reset();
		MotorControl::MotorSpeedDef s;s.Lv = 0.f;
		
		float tht = -PP::Pi / 2.f ;

    len = MotionCalcRotate(tht, PP::Mu, o_s);
//		while(Motor->)
    for(i = 0; i < len; i++){s.Av = o_s[i]; Motor->MotorDesireSpeed->Push((uint8_t *)&s);}
		if(corr){
			while((Motor->AngleAcc - AngZero) > -PP::Pi / 2.f + 0.1){
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				if(Motor->MotorDesireSpeed->Length() == 0){
					Motor->dOmgAdj = ((-PP::Pi / 2.f + 0.1) - (Motor->AngleAcc - AngZero));
				}
			}
		}
		Motor->dOmgAdj = 0.f;
		
		actWaitSeqEnd();
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		GetWallInfo();
		if(corr && (cur_wall.WallStatusStruct.left && cur_wall.WallStatusStruct.right)){
			float pos;
			float yaw;
			while(fabsf(IR->IRInts.YawL + IR->IRInts.YawR) > .005f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				Motor->dOmgAdj = 2.0f * actBackDirCorrBySideIrSide(&cur_wall);
			}
		}
		else if(corr && (cur_wall.WallStatusStruct.left || cur_wall.WallStatusStruct.right)){
			float pos;
			float yaw;
			while(fabsf(cur_wall.WallStatusStruct.left?IR->IRInts.YawL : IR->IRInts.YawR) > .0025f)
			{
				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
				Motor->dOmgAdj = 2.0f * actBackDirCorrBySideIrSide(&cur_wall);
			}
		}
		Motor->dOmgAdj = 0.f;
}

void actRushStart()
{
		OS_ERR err;
		uint32_t i, len;
		len = MotionCalcFwd(0.0f, PP::RushSpeedLow, SCP::RushStartDist, v_s);
		Motor->QueueEmptyOp = 0;
//	distZero = DistanceAcc;
	
    MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
    for(i = 0; i < len; i++)
    {
        s.Lv = v_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
}

void actRush(float v1, float v2)
{
		OS_ERR err;
		float distZero = Motor->DistanceAcc;
		Motor->dOmgAdj = 0.f;
//		GetWallInfoRun();
		GetWallInfo();
		uint32_t i, len;
		Motor->QueueEmptyOp = 0;
		if(v2< v1)
			len = MotionCalcFwd(v1, v2, PP::GridSize + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2 - 0.01f, v_s);
		else 
			len = MotionCalcFwd(v1, v2, PP::GridSize + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2, v_s);
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		MouseAction::WallStatus LastWallSts;
		for(i = 0; i < len; i++)
		{
			s.Lv = v_s[i];
			Motor->MotorDesireSpeed->Push((uint8_t *)&s);
		}
		float yaw = 0.f;
		uint8_t flag = 0;
		uint8_t centipedeState = 0;
		uint8_t LLCnt = 0,LRCnt = 0,HLCnt = 0,HRCnt = 0;
		uint8_t CorrCnt = 0;
		uint8_t cnt = 0;
		float LastDistL = 0.f,LastDistR = 0.f;
		float DetDistL = 0.f,DetDistR = 0.f;
		uint8_t DetDistLCnt= 0,DetDistRCnt= 0;
		uint8_t IsCtpd = 0;
		while((Motor->DistanceAcc - distZero) < .09f){
//			LastDistL = IR->IRInts.dSL;
//			LastDistR = IR->IRInts.dSR;
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				LastWallSts.WallStatusStruct.left = cur_wall.WallStatusStruct.left;
//				LastWallSts.WallStatusStruct.right = cur_wall.WallStatusStruct.right;
//				GetWallInfoRun();
//				switch(centipedeState){
//					case 0: //wait two wall disappear	
//						if(IR->IRInts.dSL - LastDistL  > 0.012f){
//							DetDistLCnt +=1;
//						}
//						else if(IR->IRInts.dSL - LastDistL  > 0.025f)DetDistLCnt +=2;
//						else DetDistLCnt = 0;
//						
//						if(IR->IRInts.dSR - LastDistR  > 0.012f){
//							DetDistRCnt +=1;
//						}
//						else if(IR->IRInts.dSR - LastDistR  > 0.025f)DetDistRCnt +=2;
//						else DetDistRCnt = 0;
//						//if(LastWallSts.WallStatusStruct.left && (!cur_wall.WallStatusStruct.left)){
//						if(DetDistLCnt > 1){
//							centipedeState = 1;
//							HLCnt = cnt;
////							printf("left Wall Disappear1\r\n");
//							IsCtpd = 1;
//						}
//						//if(LastWallSts.WallStatusStruct.right && (!cur_wall.WallStatusStruct.right)){
//						if(DetDistRCnt > 1){
//							if(centipedeState == 1){
//								centipedeState = 3;
//								printf("time span:%d\r\n",(HLCnt - HRCnt));
//								IsCtpd = 1;
//							}
//							else {
//								centipedeState = 2;
////								printf("Right Wall Disappear1\r\n");
//							}
//							HRCnt = cnt;
//						}
////						printf("DetDistLCnt:%d\r\n",DetDistLCnt);
////						printf("DetDistRCnt:%d\r\n",DetDistRCnt);
//						break;
//					case 1://wait Right Ir High
////						printf("wait Right Ir High\r\n");
////						LED_R(1);LED_G(1);LED_B(1);
//						//if(LastWallSts.WallStatusStruct.right && (!cur_wall.WallStatusStruct.right)){
//						if(IR->IRInts.dSR - LastDistR  > 0.012f){
//							DetDistRCnt +=1;
//						}
//						else if(IR->IRInts.dSR - LastDistR  > 0.025f)DetDistRCnt +=2;
//						else DetDistRCnt = 0;
//						if(DetDistRCnt > 1){
//							centipedeState = 4;
////							printf("both high with time span\r\n");
//							HRCnt = cnt;
//							printf("time span:%d\r\n",(HLCnt - HRCnt));
//							IsCtpd = 1;
//						}
//						else if(cnt > 20){
//							centipedeState = 5;
//						}
//						break;
//					case 2://wait Left Ir High
////						printf("wait Left Ir High\r\n");
////						LED_R(0);LED_G(1);LED_B(1);
//						//if(LastWallSts.WallStatusStruct.left && (!cur_wall.WallStatusStruct.left)){
//						if(IR->IRInts.dSL - LastDistL  > 0.012f){
//							DetDistLCnt +=1;
//						}
//						else if(IR->IRInts.dSL - LastDistL  > 0.025f)DetDistLCnt +=2;
//						else DetDistLCnt = 0;
//						if(DetDistLCnt > 1){
//							centipedeState = 4;
////							printf("both high with time span\r\n");
//							HLCnt = cnt;
//							printf("time span:%d\r\n",(HLCnt - HRCnt));
//							IsCtpd = 1;
//						}
//						else if(cnt > 20){
//							centipedeState = 6;
//						}
//						break;
//					case 3://both high with no time span
////						printf("both high with no time span\r\n");
////						LED_R(0);LED_G(0);LED_B(0);
//						Motor->dOmgAdj = 0.f;
//						break;
//					case 4://both high with time span-
//						
////						LED_R(1);LED_G(0);LED_B(0);
//						if(CorrCnt < 6)
//							Motor->dOmgAdj = -(HLCnt - HRCnt) * 0.02f;
//						else if(CorrCnt < 11)
//							Motor->dOmgAdj = +(HLCnt - HRCnt) * 0.008f;
//						else Motor->dOmgAdj = 0.f;
//						CorrCnt++;
//						break;
//					case 5://right Wall No Disapper
//						break;
//				}
//				cnt ++;
			}
		Motor->dOmgAdj = 0.f;
//		LED_R(0);LED_G(1);LED_B(0);
		float LFallDistErr,RFallDistErr;
		bool LErrValid,RErrValid;
		LFallDistErr = Motor->DistanceAcc - IR->IRInts.LFallStp;
		RFallDistErr = Motor->DistanceAcc - IR->IRInts.RFallStp;
		LErrValid = (LFallDistErr > 0.f)&&(LFallDistErr < .09f);
		RErrValid = (RFallDistErr > 0.f)&&(RFallDistErr < .09f);
		GetWallInfo();
		if(LErrValid&&RErrValid)
		{
				Motor->MotorDesireSpeed->Clear();
				len = MotionCalcFwd(Motor->DesireSpeed.Lv, v2, (PP::GridSize - (LFallDistErr + RFallDistErr)/2.f - (SCP::LEFT_WALLDIS_POS + SCP::LEFT_WALLDIS_POS)/2.f), v_s);
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
		}
		else if(LErrValid)
		{
				Motor->MotorDesireSpeed->Clear();
				len = MotionCalcFwd(Motor->DesireSpeed.Lv, v2, (PP::GridSize - LFallDistErr - SCP::LEFT_WALLDIS_POS)/*/(1.f + DIST_SPEED_CORR*(desire.lv + v2)/2)*/, v_s);
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
		}
		else if(RErrValid){
				Motor->MotorDesireSpeed->Clear();
				len = MotionCalcFwd(Motor->DesireSpeed.Lv, v2, (PP::GridSize - RFallDistErr - SCP::RIGHT_WALLDIS_POS)/*/(1.f + DIST_SPEED_CORR*(desire.lv + v2)/2)*/, v_s);
				s.Av = 0.f;
				for(i = 0; i < len; i++)
				{
						s.Lv = v_s[i];
						Motor->MotorDesireSpeed->Push((uint8_t *)&s);
				}
		}
		

		while(Motor->MotorDesireSpeed->Length() > 2){
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			if( !IsCtpd){
			if((cur_wall.WallStatusStruct.left) || (cur_wall.WallStatusStruct.right)){
				if(cur_wall.WallStatusStruct.left)
				{
				yaw = IR->IRInts.YawL;
				if(cur_wall.WallStatusStruct.right)
					{
						yaw += IR->IRInts.YawR;
						yaw *= 0.5f;
						flag = 1;
					}
				}
				else if(cur_wall.WallStatusStruct.right)
					yaw = IR->IRInts.YawR;
				else
					yaw = 0.0f;
	
				if(yaw > .9f)
					yaw = .9f;
				else if(yaw < -.9f)
					yaw = -.9f;
				Motor->dOmgAdj = (flag)? (-yaw * 1.2f):(-yaw * 1.75f);
			}
		}
		else {
			printf("cptd\r\n");
//			if(CorrCnt < 6)
//				Motor->dOmgAdj = -(HLCnt - HRCnt) * 0.02f;
//			else if(CorrCnt < 11)
//				Motor->dOmgAdj = +(HLCnt - HRCnt) * 0.008f;
//			else Motor->dOmgAdj = 0.f;
//			CorrCnt++;
		}
	}
		Motor->dOmgAdj = 0.0f;
}

unsigned int actRushStop()
{
		OS_ERR err;
		uint32_t i, len;
		len = MotionCalcFwd(PP::RushSpeedLow, 0.0f, SCP::RUSH_STOP_DIST, v_s);
		Motor->QueueEmptyOp = 1;
		Motor->dOmgAdj = 0.f;
		distZero = Motor->DistanceAcc;
		GetWallInfo();
    MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
    for(i = 0; i < len; i++)
    {
        s.Lv = v_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);

		OSTimeDlyHMSM(0, 0, 0, 80, OS_OPT_TIME_HMSM_STRICT, &err);
		return cur_wall.msk;
}

void actXRush(float v1, float v2)
{
		OS_ERR err;
		uint32_t i, len;
		Motor->QueueEmptyOp = 0;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		while(((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)&&((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f))OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//LED_R(1);LED_G(1);LED_B(0);
		if((Motor->DistanceAcc - IR->IRInts.LFallStp) <= .08f){
			if(v2 < v1)
				len = MotionCalcFwd(v1, v2, -0.01f + PP::GridSize*sqrt2/2.f + SCP::XLPOSTADJ - (Motor->DistanceAcc - IR->IRInts.LFallStp) + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2, v_s);
			else
				len = MotionCalcFwd(v1, v2, PP::GridSize*sqrt2/2.f + SCP::XLPOSTADJ - (Motor->DistanceAcc - IR->IRInts.LFallStp) + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2, v_s);
			LED_R(1);LED_G(1);LED_B(0);
		}
		else 
		{
			if(v2 < v1)
				len = MotionCalcFwd(v1, v2, -0.01f + PP::GridSize*sqrt2/2.f + SCP::XLPOSTADJ - (Motor->DistanceAcc - IR->IRInts.LFallStp) + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2, v_s);
			else
				len = MotionCalcFwd(v1, v2, PP::GridSize*sqrt2/2.f + SCP::XRPOSTADJ - (Motor->DistanceAcc - IR->IRInts.RFallStp) + SCP::DIST_SPEED_CORR_1*v1 + SCP::DIST_SPEED_CORR_2*v2, v_s);
			LED_R(0);LED_G(1);LED_B(0);
		}
//		printf("DisAcc:%f\r\n",Motor->DistanceAcc);
//		printf("RFallStp:%f\r\n",IR->IRInts.RFallStp);
//		printf("Vslen:%d\r\n",len);
		distZero = Motor->DistanceAcc;
	
		for(i = 0; i < len; i++)
		{
        s.Lv = v_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
		}
		
//		while((Motor->DistanceAcc - distZero) < .06f)
//		{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				if(IR->IRInts.dFL < .205f)
//				{
//						Motor->dOmgAdj = -.5f;
//						
//				}
//				else if(IR->IRInts.dFR < .245f)
//				{
//						Motor->dOmgAdj = .5f;
//						
//				}
//				else Motor->dOmgAdj = 0.f;
//		}
//		while((Motor->DistanceAcc - distZero) < .10f)
//		{
//				OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//				if(IR->IRInts.dFL < .20f)
//				{
//						Motor->dOmgAdj = -.5f;
//						
//				}
//				else if(IR->IRInts.dFR < .24f)
//				{
//						Motor->dOmgAdj = .5f;
//						
//				}
//				else Motor->dOmgAdj = 0.f;
//		}
		static uint8_t XCorrCnt = 0;
		static uint8_t XCorrState = 0;
		while(Motor->MotorDesireSpeed->Length() > 1)
		{
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			switch(XCorrState){
				case 0:
					if(IR->IRInts.dSL < 0.043f){//0.067
						Motor->dOmgAdj = -.6f;
						XCorrState = 1;
//						printf("X Right Corr\r\n");
//						printf("MotorQueueLen:%d\r\n",Motor->MotorDesireSpeed->Length());
//						printf("XCorrState:%d\r\n",XCorrState);
					}
					if(IR->IRInts.dSR < 0.04f){	//0.0485
						Motor->dOmgAdj = .6f;	//0.06f
						XCorrState = 2;
//						printf("X Left Corr\r\n");
//						printf("MotorQueueLen:%d\r\n",Motor->MotorDesireSpeed->Length());
					}
					break;
				case 1: 
					if(XCorrCnt < 16){
						XCorrCnt++;
//						printf("XcorrCnt:%d\r\n",XCorrCnt);
					}
					else if(XCorrCnt < 21){
						Motor->dOmgAdj = .6f;
						XCorrCnt++;
//						printf("XcorrCnt:%d\r\n",XCorrCnt);
					}
					else {
						Motor->dOmgAdj = 0.f;
						XCorrState = 0;
					}
					break;
				case 2: 
					if(XCorrCnt < 16){
						XCorrCnt++;
//						printf("XcorrCnt:%d\r\n",XCorrCnt);
					}
					else if(XCorrCnt < 21){
						Motor->dOmgAdj = -.6f;
						XCorrCnt++;
//						printf("XcorrCnt:%d\r\n",XCorrCnt);
					}
					else {
						Motor->dOmgAdj = 0.f;
						XCorrState = 0;
					}
					break;
			
			}
			
			
//			if((XCorrState == 0) && (IR->IRInts.dSL < 0.075f)){
//				Motor->dOmgAdj = -1.f;
//				XCorrState = 1;
//				printf("X Right Corr\r\n");
//				printf("MotorQueueLen:%d\r\n",Motor->MotorDesireSpeed->Length());
//				printf("XCorrState:%d\r\n",XCorrState);
//			}
//			if((XCorrState == 0) && (IR->IRInts.dSR < 0.065f)){
//				Motor->dOmgAdj = 1.f;
//				XCorrState = 2;
//				printf("X Left Corr\r\n");
//				printf("MotorQueueLen:%d\r\n",Motor->MotorDesireSpeed->Length());
//			}
//			
//			if((XCorrState > 0) && (XCorrCnt < 7))
//			{
//				XCorrCnt++;
//				printf("XcorrCnt:%d",XCorrCnt);
//			}
//			else if((XCorrState > 0) && (XCorrCnt < 10)){
//				if(XCorrState == 1){
//					Motor->dOmgAdj = .7f;
//				}
//				else if(XCorrState == 2)Motor->dOmgAdj = -.7f;
//				XCorrCnt++;
//				printf("XcorrCnt:%d",XCorrCnt);
//			}
//			else {
//				Motor->dOmgAdj = 0.f;
//				XCorrState = 0;
//			}
		}
		Motor->dOmgAdj = 0.f;
		printf("\r\n");
}

void actR45i()
{
		OS_ERR err;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		float requ;
		int i, vslen, oslen;
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (PP::Pi/4.f), 6.f*PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .06f)
		{
//		printf("DisAcc:%f\r\n",Motor->DistanceAcc);
//		printf("RFallStp:%f\r\n",IR->IRInts.RFallStp);
			OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		}
		LED_R(0);LED_G(0);LED_B(1);
//		LED_R(1);LED_G(1);LED_B(0);
//		printf("DisAcc:%f\r\n",Motor->DistanceAcc);
//		printf("RFallStp:%f\r\n",IR->IRInts.RFallStp);
//		printf("req:%f\r\n",requ);
		vslen = (SCP::R45IPRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;//(Motor->DistanceAcc - IR->IRInts.RFallStp)
//		printf("DisAcc:%f\r\n",Motor->DistanceAcc);
//		printf("RFallStp:%f\r\n",IR->IRInts.RFallStp);
//		printf("Vslen:%d\r\n",vslen);
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::R45IPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}

void actL45o()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .06f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LED_R(0);LED_G(0);LED_B(1);
		vslen = (SCP::L45OPRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
       Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, (PP::Pi/4.f), 6.f*PP::Mu, o_s, &requ);
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::L45OPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}


void actR45o()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (PP::Pi/4.f),6.f*PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .06f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::R45oPRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::R45oPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actL45i()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, (PP::Pi/4.f), 6.f*PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .06f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::L45iPRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::L45iPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}

void actRL90()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::RL90PRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, (PP::Pi/2.f), 9.f * PP::Mu, o_s, &requ);
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::RL90POST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actRR90()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::RR90PRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (PP::Pi/2.f), 9.f *PP::Mu, o_s, &requ);
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::RR90POST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actR180()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (PP::Pi), 8.f *PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::R180PRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = (SCP::R180POST)/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actL180()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, (PP::Pi), 8.f *PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::L180PRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::L180POST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actR135i()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::R135iPRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (.75f*PP::Pi), 9.f *PP::Mu, o_s, &requ);
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::R135iPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}

void actL135o()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, (.75f*PP::Pi+ 0.05f), 9.f * PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::L135oPRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::L135oPOST/PP::Ts/PP::RushSpeedLow;
		
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actL135i()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::L135iPRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		oslen = MotionCalcTurn(PP::RushSpeedLow, (.75f*PP::Pi), 9.f * PP::Mu, o_s, &requ);
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::L135iPOST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}

void actR135o()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (.75f*PP::Pi),  9.f * PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::R135oPRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::R135oPOST/PP::Ts/PP::RushSpeedLow;
		
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(false);
}

void actXL90()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, (.5f*PP::Pi), 9.f * PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.LFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::XL90PRE - (Motor->DistanceAcc - IR->IRInts.LFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::XL90POST/PP::Ts/PP::RushSpeedLow;
		
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}

void actXR90()
{
		OS_ERR err;
		float requ;
		int i, vslen, oslen;
		Motor->QueueEmptyOp = 0;
		Motor->dOmgAdj = 0.f;
		oslen = MotionCalcTurn(PP::RushSpeedLow, - (.5f*PP::Pi), 9.f * PP::Mu, o_s, &requ);
		while((Motor->DistanceAcc - IR->IRInts.RFallStp) > .08f)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	
		vslen = (SCP::XR90PRE - (Motor->DistanceAcc - IR->IRInts.RFallStp))/PP::Ts/PP::RushSpeedLow;
		MotorControl::MotorSpeedDef s;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
	
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < oslen; i++)
    {
        s.Av = o_s[i];
       Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		vslen = SCP::XR90POST/PP::Ts/PP::RushSpeedLow;
		s.Av = 0.f;
		s.Lv = PP::RushSpeedLow;
    for(i = 0; i < vslen; i++)
    {
        Motor->MotorDesireSpeed->Push((uint8_t *)&s);
    }
		
		while(Motor->MotorDesireSpeed->Length() > 2)OSSemPend(&MotorTick,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		//DirectionCorr(true);
}


}
