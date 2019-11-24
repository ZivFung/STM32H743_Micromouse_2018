#ifndef MOTORENCIMU_MOTORPWM_H_
#define MOTORENCIMU_MOTORPWM_H_

#include "includes.h"
#include "usart.h"
#include "physparams.h"
namespace MotorControl{

typedef struct
{
    short r,l;
}MotorDuty;

class MotorPwm{
private:
    void MotorPwmInit();
protected:
public:
    MotorDuty MotorDutyNow;
    MotorPwm();
void MotorPwmCoast();
// r,l -> [-999, 999]
void MotorPwmSetDuty(MotorDuty duty);


};

}


#endif /* MOTORENCIMU_MOTORPWM_H_ */
