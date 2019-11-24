/*
 * solve.cpp
 *
 *  Created on: Sep 6, 2014
 *      Author: loywong
 */

#include "solve.h"
#include "includes.h"
//#include <stdio.h>
#include "Action.h"
#include "Infrared.h"
#include "motor.h"
//#include <stdlib.h>
//#include "physparams.h"
#include "mmaze.h"
#include "mouse.h"
#include "stmflash.h"
#include <limits.h>

#include "usart.h"
//#include "top.h"

using namespace Micromouse;
using namespace MouseAction;
using namespace MotorControl;
using namespace InfraredControl;

extern OS_TCB   MotorTaskTCB;
extern OS_TCB   ActionTaskTCB;
extern volatile MouseAction::WallStatus cur_wall;
extern MotorCtl *Motor;
extern Infrared *IR;
//int waitActionFinish()
//{
//    void *msg;
//	OS_MSG_SIZE msg_size;
//    OS_ERR err;
//    do
//    {
//		msg = OSTaskQPend (0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
//    }while(((CPU_INT32U)msg & 0xFF000000) != ACT2SLV_MSG_ACT_FINISH);
//    return (CPU_INT32U)msg & 0x00000007;
//}

//void breakBeforeRun()
//{
//	OS_ERR err;
//    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

//	OSTaskQPost (&TskMotorTCB,(void *)DisablePID,1,OS_OPT_POST_FIFO,&err);
//	OSTaskQPost (&TskMotorTCB,(void *)DisableGyro,1,OS_OPT_POST_FIFO,&err);
//	OSTaskQPost (&TskMotorTCB,(void *)DisableAccl,1,OS_OPT_POST_FIFO,&err);
//	OSTaskQPost (&TskMotorTCB,(void *)DisableAcqZeros,1,OS_OPT_POST_FIFO,&err);
//	DistanceAcc = 0.f;

//    while(DistanceAcc < 0.02f)
//    {
//        OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &err);		
//    }
//    // indicate ready to go
//	OSTaskQPost (&TskIndicTCB,(void *)INDIC_MSG_READYTOGO,1,OS_OPT_POST_FIFO,&err);
//    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

//    // wait for ir touch
//    WaitIrTouch(IRCH_SL_MSK, 10000, 7000);

//	OSTaskQPost (&TskMotorTCB,(void *)EnableAccl,1,OS_OPT_POST_FIFO,&err);
//	OSTaskQPost (&TskMotorTCB,(void *)EnableGyro,1,OS_OPT_POST_FIFO,&err);
//	OSTaskQPost (&TskMotorTCB,(void *)EnableAcqZeros,1,OS_OPT_POST_FIFO,&err);
//    OSTaskQPost (&TskMotorTCB,(void *)EnablePID,1,OS_OPT_POST_FIFO,&err);
//    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
//}

uint16_t mskGenerator(Turning::Type *ptr)
{
		uint8_t i;
		uint16_t msk = 0;
	
		for(i=0;i<3;i++)
		{
				msk <<= 2;
				switch(*(ptr + i))
				{
						case Turning::Forward:
								msk += 0;
								break;
						case Turning::Left:
								msk += 1;
								break;
						case Turning::Right:
								msk += 2;
								break;
						case Turning::Backward:
								msk += 3;
								break;
						default:
								break;
				}
		}
		
		return msk;
}

void ActionConvert(Turning::Type *turnings,Act::ActType *actions)
{
#define mskF	0
#define mskL	1
#define mskR	2
#define mskB	3
	
		uint16_t aptr = 1, actmsk;
		bool convertFinish = false;
		
		*actions = Act::RushStart;
		
		while(!convertFinish)
		{
				actmsk = mskGenerator(turnings);
				switch(actmsk)
				{
						case (mskF * 16 + mskF * 4 + mskF):
								*(actions + aptr) = Act::RushLow;
								*(actions + aptr + 1) = Act::RushLow;
								aptr += 2; turnings += 2;
								break;
						case (mskF * 16 + mskF * 4 + mskL):
								*(actions + aptr) = Act::RushLow;
								aptr += 1; turnings += 1;
								break;
						case (mskF * 16 + mskF * 4 + mskR):
								*(actions + aptr) = Act::RushLow;
								aptr += 1; turnings += 1;
								break;
						case (mskF * 16 + mskL * 4 + mskF):
								*(actions + aptr) = Act::RL90;
								aptr += 1; turnings += 2;
								break;
						case (mskF * 16 + mskL * 4 + mskL):
								switch(*(turnings + 3))
								{
										case Turning::Forward:
												*(actions + aptr) = Act::L180;
												aptr += 1; turnings += 3;
												break;
										case Turning::Right:
												*(actions + aptr) = Act::L135i;
												aptr += 1; turnings += 2;
												break;
										default:
												break;
								}
								break;
						case (mskF * 16 + mskL * 4 + mskR):
								*(actions + aptr) = Act::L45i;
								aptr += 1; turnings += 1;
								break;
						case (mskF * 16 + mskR * 4 + mskF):
								*(actions + aptr) = Act::RR90;
								aptr += 1; turnings += 2;
								break;
						case (mskF * 16 + mskR * 4 + mskL):
								*(actions + aptr) = Act::R45i;
								aptr += 1; turnings += 1;
								break;
						case (mskF * 16 + mskR * 4 + mskR):
								switch(*(turnings + 3))
								{
										case Turning::Forward:
												*(actions + aptr) = Act::R180;
												aptr += 1; turnings += 3;
												break;
										case Turning::Left:
												*(actions + aptr) = Act::R135i;
												aptr += 1; turnings += 2;
												break;
										default:
												break;
								}
								break;
						case (mskL * 16 + mskR * 4 + mskF):
								*(actions + aptr) = Act::R45o;
								aptr += 1; turnings += 2;
								break;
						case (mskL * 16 + mskR * 4 + mskL):
								*(actions + aptr) = Act::XLow;
								aptr += 1; turnings += 1;
								break;
						case (mskL * 16 + mskR * 4 + mskR):
								switch(*(turnings + 3))
								{
										case Turning::Forward:
												*(actions + aptr) = Act::R135o;
												aptr += 1; turnings += 3;
												break;
										case Turning::Left:
												*(actions + aptr) = Act::XR90;
												aptr += 1; turnings += 2;
												break;
										default:
												break;
								}
								break;
						case (mskR * 16 + mskL * 4 + mskF):
								*(actions + aptr) = Act::L45o;
								aptr += 1; turnings += 2;
								break;
						case (mskR * 16 + mskL * 4 + mskL):
								switch(*(turnings + 3))
								{
										case Turning::Forward:
												*(actions + aptr) = Act::L135o;
												aptr += 1; turnings += 3;
												break;
										case Turning::Right:
												*(actions + aptr) = Act::XL90;
												aptr += 1; turnings += 2;
												break;
										default:
												break;
								}
								break;
						case (mskR * 16 + mskL * 4 + mskR):
								*(actions + aptr) = Act::XLow;
								aptr += 1; turnings += 1;
								break;
						case (mskF * 16 + mskB * 4 + mskF):
						case (mskF * 16 + mskB * 4 + mskL):
						case (mskF * 16 + mskB * 4 + mskR):
						case (mskF * 16 + mskB * 4 + mskB):
								*(actions + aptr) = Act::RushStop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						case (mskF * 16 + mskF * 4 + mskB):
								*(actions + aptr) = Act::RushLow;
								aptr += 1;
								*(actions + aptr) = Act::RushStop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						case (mskL * 16 + mskR * 4 + mskB):
								*(actions + aptr) = Act::R45o;
								aptr += 1;
								*(actions + aptr) = Act::Stop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						case (mskR * 16 + mskL * 4 + mskB):
								*(actions + aptr) = Act::L45o;
								aptr += 1;
								*(actions + aptr) = Act::Stop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						case (mskF * 16 + mskL * 4 + mskB):
								*(actions + aptr) = Act::RL90;
								aptr += 1;
								*(actions + aptr) = Act::Stop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						case (mskF * 16 + mskR * 4 + mskB):
								*(actions + aptr) = Act::RR90;
								aptr += 1;
								*(actions + aptr) = Act::Stop;
								aptr += 1;
								*(actions + aptr) = Act::Null;
								aptr += 1;
								convertFinish = true;
								break;
						default:
//								UartPrint((char*)"action convert error");
								printf("action convert error");
								break;
				}
		}
		
		while(*actions != Act::Null)
		{
				aptr = 0;
				if(*actions == Act::RushLow)
				{
						while(*(actions + aptr) == Act::RushLow)aptr++;
					
						if(aptr == 1)*(actions++) = Act::RushLow;
						else if(aptr == 2)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 3)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::RushMidLow;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 4)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::MidDown;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 5)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::RushMid;
								*(actions++) = Act::MidDown;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 6)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::HighUp;
								*(actions++) = Act::HighDown1;
								*(actions++) = Act::MidDown;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 7)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::HighUp;
								*(actions++) = Act::RushHighLow;
								*(actions++) = Act::HighDown1;
								*(actions++) = Act::MidDown;
								*(actions++) = Act::MidDown1;
						}
						else if(aptr == 8)
						{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::HighUp;
								*(actions++) = Act::HighUp1;
								*(actions++) = Act::HighDown;
								*(actions++) = Act::HighDown1;
								*(actions++) = Act::MidDown;
								*(actions++) = Act::MidDown1;
						}
						else{
								*(actions++) = Act::MidUp;
								*(actions++) = Act::MidUp1;
								*(actions++) = Act::HighUp;
								*(actions++) = Act::HighUp1;
								while(*actions == Act::RushLow)
								{
										*actions = Act::RushHigh;
										actions++;
								}
								*(actions-4) = Act::HighDown;
								*(actions-3) = Act::HighDown1;
								*(actions-2) = Act::MidDown;
								*(actions-1) = Act::MidDown1;
						}
				}
				else if(*actions == Act::XLow)
				{
						while(*(actions + aptr) == Act::XLow)aptr++;
					
						if(aptr == 1)*(actions++) = Act::XLow;
						else if(aptr == 2){
							*(actions++) = Act::XMidUp;
							*(actions++) = Act::XMidDown1;
						}
						else if(aptr == 3){
							*(actions++) = Act::XMidUp;
							*(actions++) = Act::XMid1;
							*(actions++) = Act::XMidDown1;
						}
						else if(aptr == 4){
							*(actions++) = Act::XMidUp;
							*(actions++) = Act::XMidUp1;
							*(actions++) = Act::XMidDown;
							*(actions++) = Act::XMidDown1;
						}
						else{
								*(actions++) = Act::XMidUp;
								*(actions++) = Act::XMidUp1;
								while(*actions == Act::XLow)
								{
										*actions = Act::XMid;
										actions++;
								}
								*(actions - 2) = Act::XMidDown;
								*(actions - 1) = Act::XMidDown1;
						}
				}
				else actions++;
		}
}
void ActionConvertDebug(Turning::Type *turnings,Act::ActType *actions){
	*actions++ = Act::Restart;
	while(*turnings++ != Turning::Backward){
	switch(*turnings){
		case Turning::Forward:
				 *(actions++) = Act::Fwd;
				 break;
		case Turning::Left:
				 *(actions++) = Act::L90;
				 break;
		case Turning::Right:
				 *(actions++) = Act::R90;
				 break;
		case Turning::Backward:
				 *(actions++) = Act::Stop;
//				 *(actions++) = Act::Back;
				 break;
		default:
				 break;
	}
//	turnings++;
	}
//	*(actions++) = Act::Stop;
//	*(actions++) = Act::Back;
	*(actions++) = Act::Null;

}

void Mouse::FindPath(Turning::Type *path, unsigned short X, unsigned short Y)
{
		Direction::Type dir;
		GridCoor adjGrid;
		unsigned short priority, mostPriority = USHRT_MAX,stepcnt = 0;
		Turning::Type bestTurn = Turning::Forward;
		bool findfinish = false;
	
		this->currCoor.Set(0, 0);
		this->targetCoor.Set(X, Y);
		//this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Open, true);
		this->maze->Flood(this->height, this->targetCoor, this->currCoor, WallType::Open, true);
	
		currHeading = Direction::North;
	
		while((!findfinish)&&(this->height[0] != 0))
		{
				for (int i = 0; i < 4; i++)
				{
						dir = currHeading + Mouse::turnPriorities[i];
						adjGrid = currCoor + dir;
						if (this->maze->GetWall(currCoor, dir) >= WallType::Open &&
							(priority = this->height[adjGrid.X + adjGrid.Y * this->colNum]) != 0)
						{
								if(priority < mostPriority)
								{
										mostPriority = priority;
										bestTurn = Mouse::turnPriorities[i];
								}
						}
				}

				if((this->targetCoor.X == this->currCoor.X && this->targetCoor.Y == this->currCoor.Y))
				{
						bestTurn = Turning::Backward;
						findfinish = true;
				}
				
        this->currHeading += bestTurn;
        this->currCoor += this->currHeading;
				
				path[stepcnt] = bestTurn;
				stepcnt++;
		}
}
void printfPath(Turning::Type *path){
	
		while(*(path) != Turning::Backward){
				switch(*(path))
				{
						case Turning::Forward:
								printf("^\r\n");
								break;
						case Turning::Left:
								printf("<\r\n");
								break;
						case Turning::Right:
								printf(">\r\n");
								break;
						case Turning::Backward:
								printf("B\r\n");
								break;
						default:
								break;
				}
				path++;
	}

}

void printfAction(Act::ActType *actions){
	
		while(*(actions) != Act::Null){
				switch(*(actions))
				{
						case Act::RushStart:
								printf("RushStart\r\n");
								break;
						case Act::MidUp:
								printf("MidUp\r\n");
								break;
						case Act::MidUp1:
								printf("MidUp1\r\n");
								break;
						case Act::HighUp:
									printf("HighUp\r\n");
								break;
						case Act::HighUp1:
								printf("HighUp1\r\n");
								break;
						case Act::RushHigh:
								printf("RushHigh\r\n");
								break;
						case Act::RushMid:
								printf("RushMid\r\n");
								break;
						case Act::RushMidLow:
								printf("RushMidLow\r\n");
								break;
						case Act::RushHighLow:
								printf("RushHighLow\r\n");
								break;
						case Act::RushLow:
								printf("RushLow\r\n");
								break;
						case Act::HighDown:
								printf("HighDown\r\n");
								break;
						case Act::HighDown1:
								printf("HighDown1\r\n");
								break;
						case Act::MidDown:
								printf("MidDown\r\n");
								break;
						case Act::MidDown1:
								printf("MidDown1\r\n");
								break;
						case Act::RushStop:
								printf("RushStop\r\n");
								break;
				case Act::XLow:
						printf("XLow\r\n");
						break;
				case Act::XMid:
						printf("XMid\r\n");
						break;
				case Act::XMid1:
						printf("XMid1\r\n");
						break;
				case Act::XMidUp:
						printf("XMidUp\r\n");
						break;
				case Act::XMidDown:
						printf("XMidDown\r\n");
						break;
				case Act::XMidUp1:
						printf("XMidUp1\r\n");
						break;
				case Act::XMidDown1:
						printf("XMidDown1\r\n");
						break;
				case Act::L45i:
						printf("L45i\r\n");
						break;
				case Act::L45o:
						printf("L45o\r\n");
						break;
				case Act::R45i:
						printf("R45i\r\n");
						break;
				case Act::R45o:
						printf("R45o\r\n");
						break;
				case Act::RL90:
						printf("RL90\r\n");
						break;
				case Act::RR90:
						printf("RR90\r\n");
						break;
				case Act::XL90:
						printf("XL90\r\n");
						break;
				case Act::XR90:
						printf("XR90\r\n");
						break;
				case Act::L135i:
						printf("L135i\r\n");
						break;
				case Act::L135o:
						printf("L135o\r\n");
						break;
				case Act::R135i:
						printf("R135i\r\n");
						break;
				case Act::R135o:
						printf("R135o\r\n");
						break;
				case Act::L180:
						printf("L180\r\n");
						break;
				case Act::R180:
						printf("R180\r\n");
						break;
				default:
						break; 
				}
				actions++;
	}

}

void Mouse::Rush(void)
{
		Turning::Type path[256];
		Act::ActType actions[256];
		unsigned short stepcnt = 0;
		unsigned int startpointheight,testcnt=0;
		
		this->currCoor.Set(7, 7);
//		this->currCoor.Set(6, 6);
		if(((this->maze->GetWall(currCoor, Direction::North)) == WallType::Blocked)
			||((this->maze->GetWall(currCoor, Direction::East)) == WallType::Blocked))
		{	
				FindPath(path, 7, 7);
//				FindPath(path, 6, 6);
		}
		else{
				FindPath(path, 7, 7);
//				FindPath(path, 6, 6);
				startpointheight = this->height[0];
				if(startpointheight == 0)startpointheight = UINT_MAX;
				FindPath(path, 7, 8);
//				FindPath(path, 6, 7);
				if((this->height[0] < startpointheight)&&(this->height[0] != 0))
				{
						startpointheight = this->height[0];
						testcnt = 1;
				}
				FindPath(path, 8, 7);
//				FindPath(path, 7, 6);
				if((this->height[0] < startpointheight)&&(this->height[0] != 0))
				{
						startpointheight = this->height[0];
						testcnt = 2;
				}
				FindPath(path, 8, 8);
//				FindPath(path, 7, 7);
				if((this->height[0] < startpointheight)&&(this->height[0] != 0))
				{
						startpointheight = this->height[0];
						testcnt = 3;
				}
				
				switch(testcnt)
				{
						case 1:
								FindPath(path, 7, 8);
//								FindPath(path, 6,7);
								this->targetCoor += (this->currHeading + Turning::Backward);
								break;
						case 2:
								FindPath(path, 8, 7);
//								FindPath(path, 7,6);
								this->targetCoor += (this->currHeading + Turning::Backward);
								break;
						case 3:
								FindPath(path, 8, 8);
//								FindPath(path, 7,7);
								this->targetCoor += (this->currHeading + Turning::Backward);
								break;
						default:
								FindPath(path, 7, 7);
//								FindPath(path, 6,6);
								this->targetCoor += (this->currHeading + Turning::Backward);
								break;
				}
				
				while(path[stepcnt] != Turning::Backward)stepcnt++;
				path[stepcnt] = Turning::Forward;
				path[stepcnt + 1] = Turning::Backward;
		}
//		printfPath(path);
//		OS_ERR  err;
//		OS_MSG_SIZE actionEndMsgSize;
		
		stepcnt = 0;
		ActionConvert(path,actions);
//		ActionConvertDebug(path,actions);
//		printfAction(actions);
		
//		MotorCal(800);
//		MMode = TskMotor::Run;
		Motor->MMode = Cal;
		
		while(1)
		{
				if(actions[stepcnt] == Act::Null)break;
				
				ExcuteAction(actions[stepcnt]);
				stepcnt++;
				LED_B_Toggle;
				
//			sprintf(str, "%d %d\r\n",(int)(accmax*1000.f),(int)(accmin*1000.f));
//										UartPrint(str);
		}
		printf("Rush Finish");
		
//		ExcuteAction(Act::Turn);
		ExcuteAction(Act::Back);
}
#define FlashAddr 0X080D0000
void FlashTest(){
		OS_ERR  err;
		OS_MSG_SIZE actionEndMsgSize;
		CPU_SR_ALLOC();
		OS_CRITICAL_ENTER();
    GridCoor Target(7, 7);
    Mouse m(16, 16, Target);
		m.searchedTimes[0] = 1;
		m.searchedTimes[1] = 2;
		m.searchedTimes[200] = 3;
	  //m.maze->grids[0].South = WallType::Open;
		UploadMazeInfo1(FlashAddr,(uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1),
					(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum,
					(uint32_t *)m.height,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum);
		OS_CRITICAL_EXIT();
		OS_CRITICAL_ENTER();
		STMFLASH_Read(FlashAddr, (uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1)/4);
		STMFLASH_Read(FlashAddr + 0x00010000,(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum/4);
		OS_CRITICAL_EXIT();
				int x,y;
				for(y=m.maze->rowNum;y>=0;y--)
				{
						for(x=0;x<m.maze->colNum+1;x++)
						{
								if(m.maze->grids[x+y*(m.maze->colNum+1)].West==WallType::Blocked)
//									UartPrint((char*)"|");
									printf("|");
//								else UartPrint((char*)" ");
								else printf(" ");
								if(x<m.maze->colNum && y<m.maze->rowNum)
								{
//										sprintf(str, "%3d",m.height[x+y*m.maze->colNum]);
//										UartPrint(str);
										printf("%3d",m.height[x+y*m.maze->colNum]);
								}
						}
//						UartPrint((char*)"\r\n");
						printf("\r\n");
						for(x=0;x<m.maze->colNum+1;x++)
						{
//								UartPrint((char*)"*");
								printf("*");
								if(m.maze->grids[x+y*(m.maze->colNum+1)].South==WallType::Blocked)
//									UartPrint((char*)"---");
									printf("---");
//								else UartPrint((char*)"   ");
								else printf("   ");
						}
//						UartPrint((char*)"\r\n");
						printf("\r\n");
				}
//				UartPrint((char*)"draw map finished\r\n");
				printf("draw map finished\r\n");

}

void GameBegin(bool readFlash)
{
		OS_ERR  err;
		OS_MSG_SIZE actionEndMsgSize;
		CPU_SR_ALLOC();
		OS_CRITICAL_ENTER();
    GridCoor Target(7, 7);
    Mouse m(16, 16, Target);
//    GridCoor Target(6, 6);
//    Mouse m(8, 8, Target);
		OS_CRITICAL_EXIT();
    Turning::Type turning;
    WallType::Type wFwd = WallType::Open, wLeft = WallType::Blocked, wRight = WallType::Blocked;
    bool searchFinish = false;
		
		if(readFlash)
		{
				OS_CRITICAL_ENTER();
				STMFLASH_Read(FlashAddr, (uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1)/4);
				STMFLASH_Read(FlashAddr + 0x00010000,(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum/4);
				OS_CRITICAL_EXIT();
				//draw map
				int x,y;
				for(y=m.maze->rowNum;y>=0;y--)
				{
						for(x=0;x<m.maze->colNum+1;x++)
						{
								if(m.maze->grids[x+y*(m.maze->colNum+1)].West==WallType::Blocked)
//									UartPrint((char*)"|");
									printf("|");
//								else UartPrint((char*)" ");
								else printf(" ");
								if(x<m.maze->colNum && y<m.maze->rowNum)
								{
//										sprintf(str, "%3d",m.height[x+y*m.maze->colNum]);
//										UartPrint(str);
										printf("%3d",m.height[x+y*m.maze->colNum]);
								}
						}
//						UartPrint((char*)"\r\n");
						printf("\r\n");
						for(x=0;x<m.maze->colNum+1;x++)
						{
//								UartPrint((char*)"*");
								printf("*");
								if(m.maze->grids[x+y*(m.maze->colNum+1)].South==WallType::Blocked)
//									UartPrint((char*)"---");
									printf("---");
//								else UartPrint((char*)"   ");
								else printf("   ");
						}
//						UartPrint((char*)"\r\n");
						printf("\r\n");
				}
//				UartPrint((char*)"draw map finished\r\n");
				printf("draw map finished\r\n");
				
		}
		else
		{
				turning = m.Step(wFwd, wLeft, wRight, true, &searchFinish);
			
//				MotorCal(800);
//				MMode = TskMotor::Run;
				Motor->MMode = Cal;
			
				ExcuteAction(Act::Start);
	
				while (!searchFinish)
				{
						GetWallInfo();
						wFwd = (cur_wall.WallStatusStruct.fwd) ? WallType::Blocked : WallType::Open;
						wLeft = (cur_wall.WallStatusStruct.left) ? WallType::Blocked : WallType::Open;
						wRight = (cur_wall.WallStatusStruct.right) ? WallType::Blocked : WallType::Open;
				
						turning = m.Step(wFwd, wLeft, wRight, true, &searchFinish);

						if(searchFinish)
						{
								printf("Search1 Finish Back\r\n");
								ExcuteAction(Act::Stop);
								ExcuteAction(Act::Back);
						}
						else
						{
								switch(turning)
								{
										case Turning::Forward:
												ExcuteAction(Act::Fwd);
												break;
										case Turning::Left:
												ExcuteAction(Act::L90);
												break;
										case Turning::Right:
												ExcuteAction(Act::R90);
												break;
										case Turning::Backward:
												LED_R_Toggle;
												ExcuteAction(Act::Stop);	
												ExcuteAction(Act::Back);											
												ExcuteAction(Act::Restart);
												break;
										default:
												break;
								}
//								LED_B_TOG;
//								LED_B_Toggle;
						}
				}
				printf("Search 1 Finish\r\n");
				Motor->MMode = Idle;
				LED_R(0);LED_G(1);LED_B(0);
				OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
				LED_G_Toggle;
				OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
				LED_G_Toggle;
				OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
				
				searchFinish = false;
//				CPU_SR_ALLOC();
//				CPU_CRITICAL_ENTER();
//				UploadMazeInfo1(FlashAddr,(uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1),
//					(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum,
//					(uint32_t *)m.height,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum);
//				CPU_CRITICAL_EXIT();
				Motor->MMode = Cal;
				while(Motor->MMode == Run){}
				ExcuteAction(Act::Restart);
			
				while (!searchFinish)
				{
						GetWallInfo();
						wFwd = (cur_wall.WallStatusStruct.fwd) ? WallType::Blocked : WallType::Open;
						wLeft = (cur_wall.WallStatusStruct.left) ? WallType::Blocked : WallType::Open;
						wRight = (cur_wall.WallStatusStruct.right) ? WallType::Blocked : WallType::Open;
						printf("Cur_gridx:%d\r\n",m.currCoor.X);
						printf("Cur_gridy:%d\r\n",m.currCoor.Y);
						turning = m.Step(wFwd, wLeft, wRight, false, &searchFinish);
						printf("Turnung Type:%d\r\n",turning);
						if(wFwd == WallType::Blocked)
						printf("Fwd Wall\r\n");
						if(wLeft == WallType::Blocked)
						printf("Left Wall\r\n");
						if(wRight == WallType::Blocked)
						printf("Right Wall\r\n");
						printf("Cur_gridx:%d\r\n",m.currCoor.X);
						printf("Cur_gridy:%d\r\n",m.currCoor.Y);
						if(searchFinish)
						{
								printf("Search2 Finish Back\r\n");
								ExcuteAction(Act::Stop);
								ExcuteAction(Act::Back);
						}
						else
						{
								switch(turning)
								{
										case Turning::Forward:
												ExcuteAction(Act::Fwd);
												break;
										case Turning::Left:
												ExcuteAction(Act::L90);
												break;
										case Turning::Right:
												ExcuteAction(Act::R90);
												break;
										case Turning::Backward:
												ExcuteAction(Act::Stop);
												ExcuteAction(Act::Back);
												ExcuteAction(Act::Restart);
												break;
										default:
												break;
								}
						}
				}
				
				searchFinish = false;
				m.searchedTimes[0]--;
				CPU_CRITICAL_ENTER();
				UploadMazeInfo1(FlashAddr,(uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1),
					(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum,
					(uint32_t *)m.height,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum);
				CPU_CRITICAL_EXIT();
		}
		printf("Search 2 Finish\r\n");
			LED_R(0);LED_G(1);LED_B(0);
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
			LED_G_Toggle;
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
			LED_G_Toggle;
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
		int IrTestState = 0;
		while(true)
		{

			Motor->MMode= Idle;
			if(IR->IRInts.FR > 35000)IrTestState = 1;
			else if(IR->IRInts.FR > 35000 && IrTestState == 1)IrTestState = 1;
			else if(IrTestState == 1 && IR->IRInts.FR < 35000)IrTestState = 2;
			else if(IrTestState == 2) IrTestState = IrTestState;
			else IrTestState = 0;		
			if(IrTestState == 2){

				printf("Rush Start\r\n");
				m.Rush();
				Motor->MMode= Idle;
				
				m.currCoor.Set(m.targetCoor.X, m.targetCoor.Y);//m.targetCoor.Y
				wFwd=m.maze->GetWall(m.currCoor, m.currHeading);
				wLeft=m.maze->GetWall(m.currCoor, m.currHeading + Turning::Left);
				wRight=m.maze->GetWall(m.currCoor, m.currHeading + Turning::Right);
				printf("Cur_gridx:%d\r\n",m.currCoor.X);
				printf("Cur_gridy:%d\r\n",m.currCoor.Y);
//				printf("currheading:%d\r\n",m.currHeading);
//				if(wFwd == WallType::Blocked)printf("Fwd Wall\r\n");
//				if(wLeft == WallType::Blocked)printf("Left Wall\r\n");
//				if(wRight == WallType::Blocked)printf("Right Wall\r\n");
				Motor->MMode = Cal;
				while(Motor->MMode == Run){}
				ExcuteAction(Act::Restart);
				//turning = m.Step(wFwd, wLeft, wRight, false, &searchFinish);
				m.currHeading += Turning::Forward;
				m.currCoor += m.currHeading;
//				printf("Cur_gridx:%d\r\n",m.currCoor.X);
//				printf("Cur_gridy:%d\r\n",m.currCoor.Y);
//				printf("currheading:%d\r\n",m.currHeading);
//				ExcuteAction(Act::Restart);
			
				while (!searchFinish)
				{
						GetWallInfo();
						wFwd = (cur_wall.WallStatusStruct.fwd) ? WallType::Blocked : WallType::Open;
						wLeft = (cur_wall.WallStatusStruct.left) ? WallType::Blocked : WallType::Open;
						wRight = (cur_wall.WallStatusStruct.right) ? WallType::Blocked : WallType::Open;
//						if(cur_wall.WallStatusStruct.right)printf("Right wall \r\n");
//						if(cur_wall.WallStatusStruct.fwd)printf("Fwd wall \r\n");
						turning = m.Step(wFwd, wLeft, wRight, false, &searchFinish);
	
						if(searchFinish)
						{
								printf("Search3 Finish Back\r\n");
								ExcuteAction(Act::Stop);
								ExcuteAction(Act::Back);
						}
						else
						{
								switch(turning)
								{
										case Turning::Forward:
												ExcuteAction(Act::Fwd);
												break;
										case Turning::Left:
												ExcuteAction(Act::L90);
												break;
										case Turning::Right:
												ExcuteAction(Act::R90);
												break;
										case Turning::Backward:
												ExcuteAction(Act::Stop);
												ExcuteAction(Act::Back);
												ExcuteAction(Act::Restart);
												break;
										default:
												break;
								}
								LED_B_Toggle;
						}
//						printf("Cur_gridx:%d\r\n",m.currCoor.X);
//						printf("Cur_gridy:%d\r\n",m.currCoor.Y);
				}
				
				searchFinish = false;
				m.searchedTimes[0]--;
				CPU_CRITICAL_ENTER();
				UploadMazeInfo1(FlashAddr,(uint32_t *)m.maze->grids,sizeof(GridInfo)*(m.maze->colNum+1)*(m.maze->rowNum+1),
					(uint32_t *)m.searchedTimes,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum,
					(uint32_t *)m.height,sizeof(unsigned int)*m.maze->colNum*m.maze->rowNum);
				CPU_CRITICAL_EXIT();
					
//				ExcuteAction(Act::Restart);
				IrTestState = 0;
		}
	}
}
