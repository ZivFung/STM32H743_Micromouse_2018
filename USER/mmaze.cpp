//
//  mmmaze.cpp
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/09/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#include "mmaze.h"
//#include <string>
//#include <sstream>
//#include <math.h>
//#include <stdlib.h>

//#define SAFE

#define abs(x) ((x) < 0? -(x) : (x))
namespace Micromouse
{
    void operator++(Direction::Type& l)
    {
        signed char i = (signed char)l + 1;
        if(i > (signed char)Direction::West)
            i -= 4;
        l = (Direction::Type)i;
    }
    
    Direction::Type operator+(const Direction::Type & l, const Turning::Type & r)
    {
        signed char rtn = (signed char)l + (signed char)r;
        if(rtn > 1) return (Direction::Type)(rtn - 4);
        else if(rtn < -2) return (Direction::Type)(rtn + 4);
        else return (Direction::Type)rtn;
    }
    
    const Direction::Type &operator+=(Direction::Type & l, const Turning::Type & r)
    {
        signed char rtn = (signed char)l + (signed char)r;
        if(rtn > 1) l = (Direction::Type)(rtn - 4);
        else if(rtn < -2) l = (Direction::Type)(rtn + 4);
        else l = (Direction::Type)rtn;
        return l;
    }
    
    inline Turning::Type operator-(const Direction::Type & l, const Direction::Type & r)
    {
        signed char rtn = (signed char)l - (signed char)r;
        if(rtn > 1) return (Turning::Type)(rtn - 4);
        if(rtn < -2) return (Turning::Type)(rtn + 4);
        else return (Turning::Type)rtn;
    }
    
    // N N N
    // W N E
    // S S S
    inline Direction::Type operator-(const GridCoor & l, const GridCoor & r)
    {
        short x = l.X - r.X;
        short y = l.Y - r.Y;
        if(abs(x) > abs(y)) // east or west
        {
            if (x < 0)
                return Direction::West;
            else
                return Direction::East;
        }
        else // north or south
        {
            if(y < 0)
                return Direction::South;
            else
                return Direction::North;
        }
    }
    
    GridCoor operator+(const GridCoor & l, const Direction::Type & r)
    {
        switch (r) {
            case Direction::East:
                return GridCoor(l.X + 1, l.Y);
                //break;
            case Direction::West:
                return GridCoor(l.X - 1, l.Y);
                //break;
            case Direction::North:
                return GridCoor(l.X, l.Y + 1);
                //break;
            case Direction::South:
                return GridCoor(l.X, l.Y - 1);
                //break;
            default:
                break;
        }
    	return GridCoor(l.X, l.Y);
    }
    
    const GridCoor &operator+=(GridCoor & l, const Direction::Type & r)
    {
        switch (r) {
            case Direction::East:
                l.X++;
                break;
            case Direction::West:
                l.X--;
                break;
            case Direction::North:
                l.Y++;
                break;
            case Direction::South:
                l.Y--;
                break;
            default:
                break;
        }
        return l;
    }
    
    template <typename T>
    Queue<T>::Queue(int capacity)
    {
        this->array = new T[capacity + 1];
        this->head = 0;
        this->tail = 0;
        this->capacity = capacity;
    }
    
    template <typename T>
    Queue<T>::~Queue()
    {
        delete[] this->array;
    }
    
    template <typename T>
    void Queue<T>::Enqueue(const T & data)
    {
#ifdef SAFE
        if (Length() < capacity)
        {
#endif
            this->array[tail++] = data;
            if (tail > capacity) tail = 0;
#ifdef SAFE
        }
        else
            throw "Error: Enqueue when queue full.";
#endif
    }
    
    template <typename T>
    T Queue<T>::Dequeue()
    {
#ifdef SAFE
        if (Length() > 0)
        {
#endif
            T& rtn = this->array[head++];
            if(head > capacity) head = 0;
            return rtn;
#ifdef SAFE
        }
        else
            throw "Error: Dequeue when queue empty.";
#endif
    }
    
    template <typename T>
    T Queue<T>::Peek()
    {
#ifdef SAFE
        if (Length() > 0)
        {
#endif
            return this->array[head++];
#ifdef SAFE
        }
        else
            throw "Error: Peek when queue empty";
#endif
    }
    
    template <typename T>
    void Queue<T>::Reset()
    {
        this->head = 0;
        this->tail = 0;
    }
    
    template <typename T>
    int Queue<T>::Length()
    {
        int rtn = this->tail - this->head;
        if(rtn < 0)
            return rtn + capacity + 1;
        else
            return rtn;
    }
    
    template <typename T>
    int Queue<T>::Capacity()
    {
        return capacity;
    }
    
    MMazeSW::MMazeSW(int colNum, int rowNum)
    {
        this->colNum = colNum;
        this->rowNum = rowNum;
//				this->test = new int[512];
        this->grids = new GridInfo[(colNum + 1) * (rowNum + 1)];
        for (int y = 0; y <= rowNum; y++)
        {
            for (int x = 0; x <= colNum; x++)
            {
                this->grids[index(x, y)].South = WallType::Undefined;
                this->grids[index(x, y)].West = WallType::Undefined;
            }
        }
    }

		MMazeSW::MMazeSW()
    {
        this->colNum = 12;
        this->rowNum = 12;
        this->grids = new GridInfo[(colNum + 1) * (rowNum + 1)];
        for (int y = 0; y <= rowNum; y++)
        {
            for (int x = 0; x <= colNum; x++)
            {
                this->grids[index(x, y)].South = WallType::Undefined;
                this->grids[index(x, y)].West = WallType::Undefined;
            }
        }
    }
    
    MMazeSW::~MMazeSW()
    {
        delete[] this->grids;
    }
    
    inline int MMazeSW::index(const GridCoor &coor)
    {
        return coor.X + coor.Y * (this->colNum + 1);
    }
		
		inline int MMazeSW::indexheight(const GridCoor &coor)
    {
        return coor.X + coor.Y * this->colNum;
    }
    
    inline int MMazeSW::index(short x, short y)
    {
        return x + y * (this->colNum + 1);
    }

    short MMazeSW::ColNum()
    {
        return this->colNum;
    }
    
    short MMazeSW::RowNum()
    {
        return this->rowNum;
    }
    
    GridCoor MMazeSW::Target()
    {
        return this->target;
    }
    
    bool MMazeSW::SetWall(const GridCoor& coor, Direction::Type dir, WallType::Type wall)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#endif
        if(wall == WallType::Undefined) return true;
        switch (dir)
        {
						case Direction::South:
								this->grids[index(coor)].South = wall;
								break;
						case Direction::West:
								this->grids[index(coor)].West = wall;
								break;
						case Direction::North:
								this->grids[index(coor + Direction::North)].South = wall;
								break;
						case Direction::East:
								this->grids[index(coor + Direction::East)].West = wall;
								break;
						default:
								break;
        }
        return true;
    }
    
    bool MMazeSW::SetWall(const GridCoor& coor, WallType::Type north, WallType::Type west, WallType::Type south, WallType::Type east)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#endif
        if(north != WallType::Undefined)
        {
            this->grids[index(coor + Direction::North)].South = north;
        }
        if(east != WallType::Undefined)
        {
            this->grids[index(coor + Direction::East)].West = east;
        }
        if(south != WallType::Undefined)
        {
            this->grids[index(coor)].South = south;
        }
        if(west != WallType::Undefined)
        {
            this->grids[index(coor)].West = west;
        }
        return true;
    }
    
//    void MMaze::IncrSearchedTime(const GridCoor &coor) {
//        this->grids[index(coor)].SearchedTimes++;
//    }
//    
//    short MMaze::GetSearchedTimes(const GridCoor &coor) {
//        return this->grids[index(coor)].SearchedTimes;
//    }
    
    WallType::Type MMazeSW::GetWall(const GridCoor& coor, Direction::Type dir)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#else
        if(coor.X >= colNum || coor.Y >= rowNum)
            return WallType::Undefined;
#endif
        switch (dir) {
            case Direction::South:
                return this->grids[index(coor)].South;
                //break;
            case Direction::West:
                return this->grids[index(coor)].West;
                //break;
            case Direction::North:
                return this->grids[index(coor + Direction::North)].South;
                //break;
            case Direction::East:
                return this->grids[index(coor + Direction::East)].West;
                //break;
            default:
                return WallType::Undefined;
                //break;
        }
        //return WallType::Undefined;
    }
    
    void MMazeSW::Fluid(unsigned int *height, const GridCoor& start, const GridCoor& target, WallType::Type wallCanGo, bool fullFluid)
    {
        static Queue<GridCoor> q(this->colNum * this->rowNum / 2);
        q.Reset();
        
        short h;
        GridCoor grid(0, 0);
        GridCoor adjGrid(0, 0);
        for(int y = 0; y < this->rowNum; y++)
            for(int x = 0; x < this->colNum; x++) {
                height[x + y * this->colNum] = 0;
            }
        q.Enqueue(start);
        height[start.X + start.Y * this->colNum] = 1;
        while (q.Length() > 0)
        {
            grid = q.Dequeue();
            h = height[grid.X + grid.Y * this->colNum];
            for(signed char i = -2; i <= 1; i++)
            {
                adjGrid = grid + (Direction::Type)i;
                if(adjGrid.X >= this->colNum || adjGrid.Y >= this->rowNum)
                    continue;
                if(this->GetWall(grid, (Direction::Type)i) >= wallCanGo && height[adjGrid.X + adjGrid.Y * this->colNum] == 0)
                {
                    if(fullFluid || adjGrid.X != target.X || adjGrid.Y != target.Y) {
                        q.Enqueue(adjGrid);
                        height[adjGrid.X + adjGrid.Y * this->colNum] = h + 1;
                    }
                    else {
                        return;
                    }
                }
            }
        }
    }

		void MMazeSW::Flood(unsigned int *height, const GridCoor& start, const GridCoor& target, WallType::Type wallCanGo, bool fullFluid)
    {
        static Queue<FloodInfo> q(this->colNum * this->rowNum / 2);
        q.Reset();

        FloodInfo CurrInfo;

        for(int y = 0; y < this->rowNum; y++)
            for(int x = 0; x < this->colNum; x++)
                height[x + y * this->colNum] = 0;

        GridCoor adjGrid(0, 0);

        unsigned short CurrH, RunL, DeltaH;
        Turning::Type turn;

        height[indexheight(start)] = 1;

        { // 1st grid
            CurrH = height[indexheight(start)];
            for(signed char i = -2; i <= 1; i++)    // South -> East -> North -> West
            {
                adjGrid = start + (Direction::Type)i;
                if(adjGrid.X < 0 || adjGrid.X >= this->colNum || adjGrid.Y < 0 || adjGrid.Y >= this->rowNum)
                    continue;   // Out Border
                if(this->GetWall(start, (Direction::Type)i) >= wallCanGo)
                {
                    turn = Turning::Forward;
                    RunL = 1;
                    DeltaH = HeightTable[RunL];
                    if(height[indexheight(adjGrid)] == 0 || height[indexheight(adjGrid)] >= CurrH + DeltaH)
                    {
                        q.Enqueue(FloodInfo(adjGrid, RunL, (Direction::Type)i, turn));
                        height[indexheight(adjGrid)] = CurrH + DeltaH;
                    }
                    if((!fullFluid) && (adjGrid.X == target.X) && (adjGrid.Y == target.Y)) // Flood Finished
                        return;
                }
            }
        }

        while(q.Length() > 0)
        {
            CurrInfo = q.Dequeue();
            CurrH = height[indexheight(CurrInfo.Grid)];
            for(signed char i = -2; i <= 1; i++)    // South -> East -> North -> West
            {
                adjGrid = CurrInfo.Grid + (Direction::Type)i;
                if(adjGrid.X < 0 || adjGrid.X >= this->colNum || adjGrid.Y < 0 || adjGrid.Y >= this->rowNum)
                    continue;   // Out Border
                if(this->GetWall(CurrInfo.Grid, (Direction::Type)i) >= wallCanGo)
                {
                    turn = (Direction::Type)i - CurrInfo.Dir;
                    RunL = CurrInfo.runLength;

                    if((CurrInfo.Turn == Turning::Forward && turn == Turning::Forward)
                        || (CurrInfo.Turn == Turning::Left && turn == Turning::Right)
                        || (CurrInfo.Turn == Turning::Right && turn == Turning::Left))
                    {
                        RunL++;
                    }
                    else
                        RunL = 1;

                    DeltaH = HeightTable[RunL];
                    if(height[indexheight(adjGrid)] == 0 || height[indexheight(adjGrid)] >= CurrH + DeltaH)
                    {
                        height[indexheight(adjGrid)] = CurrH + DeltaH;
                        q.Enqueue(FloodInfo(adjGrid, RunL, (Direction::Type)i, turn));

                    }
                    if((!fullFluid) && (adjGrid.X == target.X) && (adjGrid.Y == target.Y)) // Flood Finished
                        return;
                }
            }
        }
    }
		
    GridInfo *MMazeSW::Grids()
    {
        return this->grids;
    }

//    void MMazeSW::Print(stringstream &str, WallType::Type canGo, unsigned short *info, GridCoor mouseHere)
//    {
//        str.clear();
//        short y = this->rowNum;
//        short x;
//        // top line "+---+---+"
//        str << "  ";
//        for(x = 0; x < this->colNum; x++)
//        {
//            str << "+";
//            if(this->grids[index(x, y)].South < canGo)
//                str << "---";
//            else
//                str << "   ";
//        }
//        str << "+" << '\n';
//        // last lines
//        for(--y; y >= 0; y--)
//        {
//            // row number
//            if(y < 10)
//                str << ' ' << y;
//            else
//                str << y;
//            // until last one
//            for(x = 0; x < this->colNum; x++)
//            {
//                if(this->grids[index(x, y)].West < canGo)
//                    str << "|";
//                else
//                    str << " ";
//                if(x == mouseHere.X && y == mouseHere.Y)
//                    str << " @ ";
//                else if(info != NULL) {
//                    if(info[x + y * this->colNum] == 0)
//                        str << "   ";
//                    else if(info[x + y * this->colNum] < 10)
//                        str << " " << info[x + y * this->colNum] << " ";
//                    else if(info[x + y * this->colNum] < 100)
//                        str << " " << info[x + y * this->colNum];
//                    else
//                        str << info[x + y * this->colNum];
//                }
//                else {
//                    str << "   ";
//                }
//            }

//            // last one
//            if(this->grids[index(x, y)].West < canGo)
//                str << "|" << '\n';
//            else
//                str << " " << '\n';

//            // until last one
//            str << "  ";
//            for(x = 0; x < this->colNum; x++)
//            {
//                if(this->grids[index(x, y)].South < canGo)
//                    str << "+---";
//                else
//                    str << "+   ";
//            }
//            // last one
//            str << "+" << '\n';
//        }
//        // col number
//        str << "  ";
//        for(x = 0; x < this->colNum; x++)
//        {
//            if(x < 10)
//                str << "  " << x << ' ';
//            else
//                str << ' ' << x << ' ';
//        }
//        str << '\n';
//    }
}
