/*
 * Kalman1Var.h
 *
 *  Created on: Aug 4, 2016
 *      Author: loywong
 */

#ifndef KALMAN_KALMAN1VAR_H_
#define KALMAN_KALMAN1VAR_H_

class Kalman1Var
{
private:
    float A, B, H, Q, R;
    float xpri, Ppri;
    float K, xpos, Ppos;
public:
    Kalman1Var(float A, float B, float H, float Q, float R, float x0, float P0);
    void Predict(float u);
    float Correct(float z);
    virtual ~Kalman1Var();
};

#endif /* KALMAN_KALMAN1VAR_H_ */
