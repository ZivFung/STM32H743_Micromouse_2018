#ifndef _WHEELENC_H_
#define _WHEELENC_H_
#include "physparams.h"
#include "usart.h"
#include "includes.h"
namespace MotorControl{
//extern Semaphore_Handle SemMotTick;
//extern Semaphore_Handle SemIrTick;

const float EncUnit = PP::RWheel * PP::Pi / PP::EncRes / PP::Ts/2.f;
	
struct WheelEncValueDef{
	volatile int32_t old_encoder,encoder,count;
};
class WheelEnc{
private:
		void Encoder_TIM_Init(u16 arr,u16 psc);
public:
    WheelEncValueDef WheelEncValueL, WheelEncValueR;
		float EncVal;
    WheelEnc();
    void WheelEncGetVel();
};
}
#endif /* MOTORENCIMU_WHEELENC_H_ */
