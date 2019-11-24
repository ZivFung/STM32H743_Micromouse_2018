/*
 * Kalman1Var.cpp
 *
 *  Created on: Aug 4, 2016
 *      Author: loywong
 */

#include <Kalman1Var.h>

Kalman1Var::Kalman1Var(float A, float B, float H, float Q, float R, float x0, float P0)
{
    this->A = A;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->xpos = x0;
    this->Ppos = P0;
}

Kalman1Var::~Kalman1Var()
{
    // TODO Auto-generated destructor stub
}

void Kalman1Var::Predict(float u)
{
//    xˆ -k = A xˆ k – 1 + B u k – 1
    xpri = A * xpos + B * u;
//    P -k = A P k – 1 A T + Q
    Ppri = A * Ppos * A + Q;
}

float Kalman1Var::Correct(float z)
{
//    K k = P -k H T ( H P -k H T + R ) – 1
    K = Ppri * H / (H * Ppri * H + R);
//    xˆ k = xˆ -k + K k ( z k – H xˆ -k )
    xpos = xpri + K * (z - H * xpri);
//    P k = ( I – K k H ) P -k
    Ppos = (1 - K * H) * Ppri;
    return xpos;
}
