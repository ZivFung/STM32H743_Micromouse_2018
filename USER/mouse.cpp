//
//  mouse.cpp
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/10/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#include "mouse.h"
#include "mmaze.h"
#include <limits.h>
//#include <stdio.h>
//#include <string.h>

namespace Micromouse {
    
    Mouse::Mouse(short colNum, short rowNum, const GridCoor &target)
    {
        this->maze = new MMazeSW(colNum, rowNum);
        this->mazeForStore = this->maze;

        this->searchedTimes = new unsigned int[colNum * rowNum];
        this->height = new unsigned int[colNum * rowNum];
        this->lastStepHere = new int[colNum * rowNum];

        for(int y = 0; y < rowNum; y++)
        {
            for(int x = 0; x < colNum; x++)
            {
                this->maze->SetWall(GridCoor(x, y), WallType::Unknown, WallType::Unknown, WallType::Unknown, WallType::Unknown);
                if(x == 0)
                    this->maze->SetWall(GridCoor(x, y), Direction::West, WallType::Blocked);
                if(y == 0)
                    this->maze->SetWall(GridCoor(x, y), Direction::South, WallType::Blocked);
                if(x == colNum - 1)
                    this->maze->SetWall(GridCoor(x, y), Direction::East, WallType::Blocked);
                if(y == rowNum - 1)
                    this->maze->SetWall(GridCoor(x, y), Direction::North, WallType::Blocked);

                // TODO set entry zone wall state
//                if(x == 5 && y == 8)
//                    this->maze->SetWall(GridCoor(x, y), WallType::Blocked, WallType::Blocked, WallType::Open, WallType::Open);
//                if(x == 6 && y == 7)
//                    this->maze->SetWall(GridCoor(x, y), WallType::Open, WallType::Open, WallType::Blocked, WallType::Blocked);
//                if(x == 6 && y == 8)
//                    this->maze->SetWall(GridCoor(x, y), WallType::Blocked, WallType::Open, WallType::Open, WallType::Blocked);
                this->searchedTimes[x + y * colNum] = 0;
            }
        }
				
        this->colNum = colNum;
        this->rowNum = rowNum;
        this->targetCoor = target;
        this->currCoor.Set(0, 0);
        this->currHeading = Direction::North;
        this->state.Proc = MouseProcState::Idle;
        this->state.RunTimes = 0;
        this->state.StepCount = 0;
				
//        this->turnPriorities[0] = Turning::Forward;
//        this->turnPriorities[1] = Turning::Right;
//        this->turnPriorities[2] = Turning::Left;
//        this->turnPriorities[3] = Turning::Backward;
				
				this->turnPrioritiesForwardFirst[0] = Turning::Forward;
				this->turnPrioritiesForwardFirst[1] = Turning::Right;
				this->turnPrioritiesForwardFirst[2] = Turning::Left;
				this->turnPrioritiesForwardFirst[3] = Turning::Backward;
					
				this->turnPrioritiesLeftFirst[0] = Turning::Left;
				this->turnPrioritiesLeftFirst[1] = Turning::Forward;
				this->turnPrioritiesLeftFirst[2] = Turning::Right;
				this->turnPrioritiesLeftFirst[3] = Turning::Backward;
					
				this->turnPrioritiesRightFirst[0] = Turning::Right;
				this->turnPrioritiesRightFirst[1] = Turning::Forward;
				this->turnPrioritiesRightFirst[2] = Turning::Left;
				this->turnPrioritiesRightFirst[3] = Turning::Backward;
				
        this->turnPriorities = this->turnPrioritiesRightFirst;
    }
    
    Mouse::~Mouse() {
        delete this->maze;
        delete[] this->height;
        delete[] this->searchedTimes;
        delete[] this->lastStepHere;
    }
    
    inline int Mouse::index(const GridCoor &coor){
        return coor.X + coor.Y * this->colNum;
    }
    
    inline int Mouse::index(short x, short y){
        return x + y * this->colNum;
    }
    
    GridCoor Mouse::CurrCoor()
    {
        return this->currCoor;
    }
    
    Direction::Type Mouse::CurrHeading()
    {
        return this->currHeading;
    }

    short Mouse::searchedTimesConv(short t)
    {
    	float f = (float)t;
    	// enlarge the exponent to advance mouse's desire to explore
        return (short)(0.5f + powf(f, 0.85f));   // 4 -> 3
//    	return (short)(0.5f + powf(f, 0.7f));	// 3 -> 2
//    	return (short)(0.5f + powf(f, 0.5f));	// 2 -> 1
    }

//    Turning::Type Mouse::Step(WallType::Type fwd, WallType::Type left, WallType::Type right, bool toTarget, bool *srchFinish)//, bool &goal)
//    {
//        if(this->currCoor.X == 0 && this->currCoor.Y == 0 && this->state.Proc == MouseProcState::Idle)
//						this->state.Proc = MouseProcState::FirstSearching;
//        
//        this->maze->SetWall(currCoor, currHeading + Turning::Forward, fwd);
//        this->maze->SetWall(currCoor, currHeading + Turning::Left, left);
//        this->maze->SetWall(currCoor, currHeading + Turning::Right, right);
//        //this->maze->IncrSearchedTime(currCoor);
//        this->searchedTimes[index(currCoor)]++;

//        switch (this->state.Proc)
//        {
//            case MouseProcState::FirstSearching:
//                this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Unknown, false);
//                break;
//            case MouseProcState::BackSearching:
//                this->maze->Fluid(this->height, GridCoor(0, 0), this->currCoor, WallType::Unknown, false);
//                break;
//            case MouseProcState::Running:
//                this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Open, false);
//                break;
//            case MouseProcState::Idle:
//            default:
//                break;
//        }

//        int priority, mostPriority = INT_MAX;
//        Turning::Type bestTurn = Turning::Forward;
//        Direction::Type dir;
//        GridCoor adjGrid;

//        if(this->targetCoor.X == this->currCoor.X && this->targetCoor.Y == this->currCoor.Y)
//        {
//            //goal = true;
//            bestTurn = Turning::Backward;
//        }
//        else
//        {
//        	//goal = false;
//						for (int i = 0; i < 4; i++)
//						{
//								dir = currHeading + Mouse::turnPriorities[i];
//								adjGrid = currCoor + dir;
//								if (this->maze->GetWall(currCoor, dir) >= WallType::Open &&
//									(priority = this->height[adjGrid.X + adjGrid.Y * this->colNum]) != 0)
//								{
//										if(this->state.Proc != MouseProcState::Running)
//										{
//												//priority += this->maze->GetSearchedTimes(adjGrid) << 16;
//												priority += searchedTimesConv(this->searchedTimes[index(adjGrid)]);
//										}
//										if(priority < mostPriority)
//										{
//												mostPriority = priority;
//												bestTurn = Mouse::turnPriorities[i];
//										}
//								}
//						}
//        }

//        if(bestTurn == Turning::Backward)
//        {
//            //this->maze->IncrSearchedTime(currCoor);
//            this->searchedTimes[index(currCoor)]++;
//        }

//        this->lastStepHere[index(currCoor)] = this->state.StepCount;
//        this->state.StepCount++;

//        this->currHeading += bestTurn;
//        this->currCoor += this->currHeading;

//        if(this->targetCoor.X == this->currCoor.X && this->targetCoor.Y == this->currCoor.Y)
//            this->state.Proc = MouseProcState::BackSearching;
//        else if(this->currCoor.X == 0 && this->currCoor.Y == 0)
//        {
//            this->state.Proc = MouseProcState::Running;
//            *srchFinish = true;
//        }
//				
////        not good
////        switch(bestTurn)
////        {
////        case Turning::Left:
////        	this->turnPriorities = this->turnPrioritiesRightFirst;
////        	break;
////        case Turning::Right:
////        	this->turnPriorities = this->turnPrioritiesLeftFirst;
////        	break;
////        default:
////        	this->turnPriorities = this->turnPrioritiesForwardFirst;
////        	break;
////        }
//				
//        return bestTurn;
//    }

		Turning::Type Mouse::Step(WallType::Type fwd, WallType::Type left, WallType::Type right, bool toTarget, bool *srchFinish)
    {
        this->maze->SetWall(currCoor, currHeading + Turning::Forward, fwd);
        this->maze->SetWall(currCoor, currHeading + Turning::Left, left);
        this->maze->SetWall(currCoor, currHeading + Turning::Right, right);
			
        this->searchedTimes[index(currCoor)]++;
				this->lastStepHere[index(currCoor)] = this->state.StepCount;
        this->state.StepCount++;
			
        if(toTarget)this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Unknown, false);
        else this->maze->Fluid(this->height, GridCoor(0, 0), this->currCoor, WallType::Unknown, false);

        volatile int priority, mostPriority = INT_MAX;
        Turning::Type bestTurn = Turning::Forward;
        Direction::Type dir;
        GridCoor adjGrid;

				for (int i = 0; i < 4; i++)
				{
						dir = currHeading + Mouse::turnPriorities[i];
						adjGrid = currCoor + dir;
						if(this->maze->GetWall(currCoor, dir) == WallType::Open){
//							printf(" i = %d\r\n",i);
//							printf("Wall Block!\r\n");
//							printf("gridx:%d",adjGrid.X);
//							printf("gridy:%d",adjGrid.Y);
//							printf("height:%d",this->height[adjGrid.X + adjGrid.Y * this->colNum]);
						}
						if (this->maze->GetWall(currCoor, dir) >= WallType::Open &&
							(priority = this->height[adjGrid.X + adjGrid.Y * this->colNum]) != 0)
						{
								priority += searchedTimesConv(this->searchedTimes[index(adjGrid)]);
//								printf("%d\r\n",priority);
//								printf("%d\r\n",mostPriority);
								if(priority < mostPriority)
								{
										mostPriority = priority;
										bestTurn = Mouse::turnPriorities[i];
								}
						}
				}

				if(((toTarget)&&(this->targetCoor.X == this->currCoor.X && this->targetCoor.Y == this->currCoor.Y)) ||
					((!toTarget)&&(this->currCoor.X == 0 && this->currCoor.Y == 0)) )        
				{
						bestTurn = Turning::Backward;
            *srchFinish = true;
				}
				
				if(bestTurn == Turning::Backward)this->searchedTimes[index(currCoor)]++;
				
        this->currHeading += bestTurn;
        this->currCoor += this->currHeading;
				
        return bestTurn;
    }

//    inline void Mouse::Print(stringstream &str, bool withTimes, bool withHeight, WallType wallCanGo)
//    {
//        Print(str, withTimes, withHeight, wallCanGo, this->currCoor);
//    }

//    void Mouse::Print(stringstream &str, MousePrintOption::Type option, WallType::Type wallCanGo, GridCoor mouseHere)
//    {
//        switch (option) {
//            case MousePrintOption::ShowNone :
//                this->maze->Print(str, wallCanGo, NULL, mouseHere);
//                break;
//            case MousePrintOption::ShowSearchedTimes:
//                this->maze->Print(str, wallCanGo, this->searchedTimes, mouseHere);
//                break;
//            case MousePrintOption::ShowFluidHeights:
//                this->maze->Print(str, wallCanGo, this->height, mouseHere);
//                break;
//            default:
//                break;
//        }
//    }
//    const short *Mouse::GetHeight()
//    {
//        return this->height;
//    }
//    
//    const MMaze *Mouse::GetMaze()
//    {
//        return this->maze;
//    }

//    void *Mouse::GetGridsBuf(int discardNum)
//    {
//        int x, y;
//        int row, col;
//        size_t gridsNum;
//        GridCoor coor;

//        row = this->maze->ColNum();
//        col = this->maze->RowNum();
//        gridsNum = (row + 1) * (col + 1);

//        memcpy((_PTR)this->mazeForStore->Grids(), (const _PTR)this->maze->Grids(), sizeof(GridInfo) * gridsNum);

//        for(y = 0; y < row; y++)
//        {
//            for(x = 0; x < col; x++)
//            {

////                if(this->lastStepHere[this->index(x, y)] >= this->state.StepCount - discardNum)
////                {
////                    this->mazeForStore->SetWall()
////                }
//            }
//        }
//        return (void *)(this->mazeForStore->Grids());
//    }
}
