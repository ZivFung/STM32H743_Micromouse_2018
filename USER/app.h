#ifndef _APP_H_
#define _APP_H_


#include "includes.h"
#include "usart.h"
#include "physparams.h"
#include "Motor.h"
#include "Queue.h"
#include "Infrared.h"
#include "IrCorr.h"
#include "os_app_hooks.h"
#include "Action.h"
#include "solve.h"

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
void start_task(void *p_arg);

#define LED_R(n)		(n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET))
#define LED_R_Toggle (HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10)) 
#define LED_G(n)		(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET))
#define LED_G_Toggle (HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15)) 
#define LED_B(n)		(n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET))
#define LED_B_Toggle (HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11)) 

class MouseMode
{
		public:
				enum ModeType
				{
						Idle = 0,
						IrValueTest = 1,
						IrDistTest = 2,
						ActionCorrection = 3,
						RandomRun = 4,
						FirstStart = 5,
						SecondStart = 6,
						IrCorrection = 7,
						RushTest = 8,
						FlashWriteTest = 9,
						EncTest = 10,
						EncImuMonitor,
						ActionWithOutIrCorrectTest,
						PIDTest,
						ListenUartCmd,
						Gaming2,
						Gaming1,
						Gaming0
				};
};
void IRFallDistStampDet(uint16_t LIRint,float *LDistStamp,uint16_t RIRint,float *RDistStamp);
void ModeEntry(MouseMode::ModeType mode);
void IrDistTest();
void RamdomRun();
namespace APP{



}



#endif
