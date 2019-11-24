#include "Pid.h"

#define USEI
#define USED

Pid::Pid(float p, float i, float d, float n, float ts, float outMin, float outMax)
{
    this->p = p;
    this->i = i;
    this->d = d;
    this->n = n;
    this->ts = ts;
    this->accI = .0f;
    this->accD = .0f;
    this->accIMin = outMin;
    this->accIMax = outMax;
    this->accDMin = -outMax / n;
    this->accDMax = -outMin / n;
}

void Pid::Reset()
{
    this->accI = .0f;
    this->accD = .0f;
}

float Pid::Tick(volatile float diff)
{
    float pout;
#ifdef USED
    float dout;
#endif
    
    pout = diff * p;

#ifdef USEI    
    accI += diff * i * ts;
    if(accI > accIMax)
        accI = accIMax;
    else if(accI < accIMin)
        accI = accIMin;
#endif

#ifdef USED
    dout = (diff * d - accD) * n;    
    accD += dout * ts;
    if(accD > accDMax)
        accD = accDMax;
    else if(accD < accDMin)
        accD = accDMin;
#endif
    
#ifdef USEI
    pout += accI;
#endif
#ifdef USED
    pout += dout;
#endif

    if(pout > accIMax)
        pout = accIMax;
    else if(pout < accIMin)
        pout = accIMin;
    
    return pout;
}


