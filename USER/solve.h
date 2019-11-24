/*
 * solve.h
 *
 *  Created on: Sep 6, 2014
 *      Author: loywong
 */

#ifndef SOLVE_H_
#define SOLVE_H_

#include <includes.h>
#include "app.h"
//#define ACT2SLV_MSG_ACT_FINISH  0xFF000000
//#define Solve_FirstStart		0x01000000
//#define Solve_SecondStart		0x02000000

//#define SOLVE_MSG_CLEANSTART    0x01000000
//#define SOLVE_MSG_START         0x02000000
//#define SOLVE_MSG_END           0x03000000
//#define SOLVE_MSG_ACT_DEBUG     0x04000000
//#define SOLVE_MSG_RANDOM_TEST   0x05000000
//#define SOLVE_MSG_TEST_ALGO     0x06000000

//void TskSolve(void *);
void GameBegin(bool readFlash);
void FlashTest();
#endif /* SOLVE_H_ */
