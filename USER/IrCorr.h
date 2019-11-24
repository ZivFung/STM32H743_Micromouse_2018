#ifndef _IRCORR_H_
#define _IRCORR_H_
#include "includes.h"
#include "usart.h"
#include "physparams.h"
#include "Infrared.h"
#include "stmflash.h"

#define TEXT_LENTH sizeof(IrApproxCoef)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)
#define FLASH_SAVE_ADDR  0X080D0000 	//设置FLASH 保存地址(必须为4的倍数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define FLASH_SAVE_ADDR1  0X080E0000
namespace InfraredControl{


	
//#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
//#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)


typedef union {
	float k[4][3];
	struct
	{
		float FL[3];
		float FR[3];
		float LS[3];
		float RS[3];
	}meb;
}IrApproxCoef;
const int IrLookupTableLen = 32;
typedef struct {
    unsigned short Dists[IrLookupTableLen];   // distance unit: 0.1mm, from ir diodes
    union
    {
        unsigned short Ints[4][IrLookupTableLen];
        struct
        {
            unsigned short FLnsInts[IrLookupTableLen];
//            unsigned short FLwsInts[IrLookupTableLen];
            unsigned short FRnsInts[IrLookupTableLen];
//            unsigned short FRwsInts[IrLookupTableLen];
            unsigned short LSInts[IrLookupTableLen];
//            unsigned short LFInts[IrLookupTableLen];
            unsigned short RSInts[IrLookupTableLen];
//            unsigned short RFInts[IrLookupTableLen];
        }meb2;
    }uni;
}IrLookupTable;
	
class IRCorrection{
	private:
		void ReadFlash(unsigned int addr, unsigned char *data, int byteLen);
		void linEquSolve(float *mat, int n);
		void irApprox2nd(float *x, float *y, int n, float *coef);
		void irApprox(float *x, float *y, int n, float *a, float *b);
	protected:
	public:
		IrApproxCoef IrACs;
//		IrLookupTable IrLTs;
		
		IRCorrection();
		void doIrCorrection();
		void doIrCorrection1();
};

class IrCh
{
		public:
				enum Type
				{
						FL = 0x1,
						FR = 0x2,
						SL = 0x4,
						SR = 0x8
				};
};

void WaitIrTouch(Infrared *IR,unsigned char chMask, int hTh, int lTh);
}

#endif

