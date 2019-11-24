//
//  mouse.h
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/10/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#ifndef __mmmaze__mouse__
#define __mmmaze__mouse__

#include "mmaze.h"
#include "includes.h"
//#include <limits.h>
	
namespace Micromouse {
    
    class MouseProcState
    {
				public : enum Type
        {
            Idle = 0,
            FirstSearching = 1,
            BackSearching = 2,
            Running = 3
        };
    };
    
//    class MousePrintOption
//    {
//				public: enum Type
//        {
//            ShowNone = 0,
//            ShowSearchedTimes = 1,
//            ShowFluidHeights = 2
//        };
//    };
    
    struct MouseState
    {
        MouseProcState::Type Proc;
        short RunTimes;
        short StepCount;
        MouseState() {
            Proc = MouseProcState::Idle;
            RunTimes = 0;
            StepCount = 0;
        }
    };
    
    class Mouse {
				private:
						//MMazeSW *maze;
						MMazeSW *mazeForStore;
						//unsigned short *searchedTimes;
						//unsigned short *height;
						int *lastStepHere;
						short colNum, rowNum;
//						GridCoor targetCoor;
						//GridCoor currCoor;
						//Direction::Type currHeading;
						MouseState state;
						Turning::Type turnPrioritiesForwardFirst[4];
						Turning::Type turnPrioritiesLeftFirst[4];
						Turning::Type turnPrioritiesRightFirst[4];
						Turning::Type *turnPriorities;
						int index(const GridCoor &coor);
						int index(short x, short y);
						short searchedTimesConv(short t);
				public:
						Mouse(short colNum, short rowNum, const GridCoor &target);
						~Mouse();
				Direction::Type currHeading;
				unsigned int *searchedTimes;
						unsigned int *height;
						MMazeSW *maze;
						GridCoor currCoor;
						GridCoor targetCoor;
						GridCoor CurrCoor();
						Direction::Type CurrHeading();
						Turning::Type Step(WallType::Type fwd, WallType::Type left, WallType::Type right, bool toTarget, bool *srchFinish);//, bool &goal);
						//void Print(stringstream &str, bool withTimes, bool withHeight, WallType wallCanGo);
						//void Print(stringstream &str, MousePrintOption::Type option, WallType::Type wallCanGo, GridCoor mouseHere = GridCoor(-1, -1));
						//const short *GetHeight();
						//const MMaze *GetMaze();
						//void *GetGridsBuf(int discardNum);
						void Rush(void);
						void FindPath(Turning::Type *path, unsigned short X, unsigned short Y);
    };
}

#endif /* defined(__mmmaze__mouse__) */
