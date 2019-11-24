#ifndef __PID_H__
#define __PID_H__

class Pid
{
private:

public:
    volatile float p, i, d, n, ts;
    volatile float accI, accD, accIMax, accIMin, accDMax, accDMin;
    Pid(float p, float i, float d, float n, float ts, float outMin, float outMax);
    float Tick(volatile float diff);
    void Reset();
};

#endif
