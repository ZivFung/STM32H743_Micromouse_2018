#ifndef MMAZE_H_
#define MMAZE_H_

//#include <iostream>
//#include <sstream>
//#include <fstream>
#include "includes.h"

namespace Micromouse
{
    class WallType
    {
				public: enum Type
				{
						Undefined = 0,
						Blocked = 1,
						Unknown = 2,
						Open = 3
				};
    };

    class Direction
    {
				public : enum Type
				{
						North = 0,
						West = 1,
						South = -2,
						East = -1
				};
    };
    
    class Turning
    {
				public : enum Type
				{
						Forward = 0,
						Left = 1,
						Backward = -2,
						Right = -1
				};
    };

    struct GridCoor
    {
        // east x, north y
        unsigned short X;// : 5;
        unsigned short Y;// : 5;
        GridCoor() {
            X = 0; Y = 0;
        }
        GridCoor(unsigned short x, unsigned short y) {
            X = x; Y = y;
        }
        void Set(unsigned short x, unsigned short y) {
            X = x; Y = y;
        }
    };
    
    //void (operator++)((Direction::Type)&);
    // direction turning
    Direction::Type operator+(const Direction::Type &, const Turning::Type &);
    const Direction::Type &operator+=(Direction::Type &, const Turning::Type &);
    Turning::Type operator-(const Direction::Type &, const Direction::Type &);
    // grid direction
    GridCoor operator+(const GridCoor &, const Direction::Type &);
    const GridCoor &operator+=(GridCoor &, const Direction::Type &);
    Direction::Type operator-(const GridCoor &, const GridCoor &);
    
		struct GridInfo
    {
        WallType::Type South : 2;
        WallType::Type West : 2;
        GridInfo()
        {
            South = WallType::Undefined;
            West = WallType::Undefined;
        }
    };
    
    template <typename T>
    class Queue
    {
				private:
						T *array;
						int head;
						int tail;
						int capacity;
				public:
						Queue(int capacity);
						~Queue();
						void Enqueue(const T & data);
						T Dequeue();
						T Peek();
						void Reset();
						int Length();
						int Capacity();
    };
    
		const unsigned short HeightTable[] =
    {
         99, 22, 16, 12, 10, 9, 8, 7,
         6, 6, 5, 5, 5, 5, 4, 4, 3
    };

    struct FloodInfo
    {
        Direction::Type Dir;
        Turning::Type Turn;
        unsigned char runLength;
        GridCoor Grid;
        FloodInfo()
        {
            Dir = Direction::North;
            Turn = Turning::Forward;
            runLength = 0;
            Grid = GridCoor(0, 0);
        }
        FloodInfo(GridCoor Grid_t, unsigned char runLength_t, Direction::Type Dir_t, Turning::Type Turn_t)
        {
            Dir = Dir_t;
            Turn = Turn_t;
            runLength = runLength_t;
            Grid = Grid_t;
        }
    };
		
    class MMazeSW
    {
				private:
//						GridInfo *grids;
						//short colNum;
						//short rowNum;
						GridCoor target;
						int index(const GridCoor &coor);
						int index(short x, short y);
						int indexheight(const GridCoor &coor);
				public:
						MMazeSW(int colNum, int rowNum);
						MMazeSW();
						~MMazeSW();
						short colNum;
						short rowNum;
						GridInfo *grids;
//						int *test; 
//						GridInfo grids[256];
						bool SetWall(const GridCoor& coor, Direction::Type dir, WallType::Type wall);
						bool SetWall(const GridCoor& coor, WallType::Type north, WallType::Type west, WallType::Type south, WallType::Type east);
		//        void IncrSearchedTime(const GridCoor &coor);
						WallType::Type GetWall(const GridCoor& coor, Direction::Type dir);
		//        short GetSearchedTimes(const GridCoor &coor);
						void Fluid(unsigned int * height, const GridCoor &start, const GridCoor &target, WallType::Type wallCanGo, bool fullFluid);
						void Flood(unsigned int * height, const GridCoor& start, const GridCoor& target, WallType::Type wallCanGo, bool fullFluid);
						short ColNum();
						short RowNum();
						GridCoor Target();
		//        void Print(stringstream &str, bool withTimes, WallType wallCanGo, GridCoor mouseHere = GridCoor(-1, -1));
						//void Print(stringstream &str, WallType::Type wallCanGo, unsigned short *info = NULL, GridCoor mouserHere = GridCoor(-1, -1));
						GridInfo *Grids();
    };
}
#endif /* defined(__mmmaze__mmmaze__) */
