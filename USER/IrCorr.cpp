#include "IrCorr.h"
extern InfraredControl::Infrared *IR;
namespace InfraredControl{
//IrApproxCoef TestIrACs = {
//2.48148f, -0.64591f, 0.01312f,
//2.9494f, -0.72441f, 0.01690f,
//-7.94628f, 1.47859f, -0.09174f,
//1.53259f, -0.38549f, -0.00135f
//};

IrApproxCoef TestIrACsBack1 = {
11.4267998f, -2.3915780f, 0.1007368f,
1.4024494f, -0.3276576f, -0.0050595f,
10.1065483f, -2.1331048f, 0.0857769f,
14.7684755, -3.0116220f, 0.1269022f
};
IrApproxCoef TestIrACsBack2 = {
6.9334383f, -1.4373534f, 0.0499271f,
4.2992907f, -0.9392898f, 0.0273701f,
2.8491101f, -0.7667754f, 0.0200059f,
15.5483932f, -3.3280506f, 0.1495192f
};

IrApproxCoef TestIrACsBack3 = {
5.9535723f, -1.1770098f, 0.0339242, 
-0.8144493f, 0.2465135f, -0.0392111, 
-5.2683897f, 1.0691702f, -0.0820851, 
25.5024929f, -5.2438645f, 0.2419643
};

IrApproxCoef TestIrACs = {
5.9535723f, -1.1770098f, 0.0339242, 
-0.8144493f, 0.2465135f, -0.0392111, 
-5.2683897f, 1.0691702f, -0.0820851, 
25.5024929f, -5.2438645f, 0.2419643
};
	IRCorrection::IRCorrection()
	{
//		uint8_t temp1 = sizeof(IrApproxCoef)/4;
//		uint8_t temp = ((int)(sizeof(IrApproxCoef)/4.f) + sizeof(IrApproxCoef)%4?1:0);
//		STMFLASH_Read(FLASH_SAVE_ADDR,(u32 *)&IrLTs.Dists[0],(sizeof(IrLookupTable)/4 + sizeof(IrLookupTable)%4?1:0));
//		STMFLASH_Read(FLASH_SAVE_ADDR,(u32 *)&IrACs.k[0],6);
//		STMFLASH_Read(FLASH_SAVE_ADDR1,(u32 *)&IrACs.k[2],6);
		IrACs = TestIrACs;
	}
	void IRCorrection::linEquSolve(float *mat, int n)
	{
    int k, j, i;
    // to up triangular mat
    for(k = 0; k < n - 1; k++)
    {
        // find major row
        int major = k;
        for(j = k + 1; j < n; j++)
        {
            if(mat[j * (n + 1) + k] > mat[major * (n + 1) + k])
                major = j;
        }
        // swap major row
        if(major != k)
        {
            float t;
            for(i = 0; i < n + 1; i++)
            {
                t = mat[k * (n + 1) + i];
                mat[k * (n + 1) + i] = mat[major * (n + 1) + i];
                mat[major * (n + 1) + i] = t;
            }
        }
        // eliminating column k, form row k + 1 to n - 1
        for(j = k + 1; j < n; j++)
        {
            float c = mat[j * (n + 1) + k] / mat[k * (n + 1) + k];
            for(i = k; i < n + 1; i++)
            {
                mat[j * (n + 1) + i] -= mat[k * (n + 1) + i] * c;
            }
        }
    }
    // to 1
    for(k = 0; k < n; k++)
    {
        float c = mat[k * (n + 1) + k];
        for(i = k; i < n + 1; i++)
        {
            mat[k * (n + 1) + i] /= c;
        }
    }
    //
    for(k = n - 1; k >= 1; k--)
    {
        for(j = k - 1; j >= 0; j--)
        {
            float c = mat[j * (n + 1) + k];
            for(i = k; i < n + 1; i++)
            {
                mat[j * (n + 1) + i] -= mat[k * (n + 1) + i] * c;
            }
        }
    }
	}

	void IRCorrection::irApprox2nd(float *x, float *y, int n, float *coef)
	{
    int i, j, k;
    float equ[3][4] = {0.f};
    for(i = 0; i < n; i++)
    {
        y[i] = logf(y[i]);
        x[i] = logf(x[i]);
    }
    for(j = 0; j < 3; j++)
    {
        for(i = 0; i < 3; i++)
        {
            for(k = 0; k < n; k++)
                equ[j][i] += powf(x[k], (float)(i + j));
        }
        for(k = 0; k < n; k++)
            equ[j][i] += powf(x[k], (float)(j)) * y[k];
    }
    //sleep(1);
    linEquSolve(&equ[0][0], 3);
    //sleep(1);
    coef[0] = equ[0][3];
    coef[1] = equ[1][3];
    coef[2] = equ[2][3];
	}
	
	void IRCorrection::irApprox(float *x, float *y, int n, float *a, float *b)
	{
    int i;
    float sumx = 0.0f, sumy = 0.0f, sumx2 = 0.0f, sumxy = 0.0f;
    float equ[2][3];
    for(i = 0; i < n; i++)
    {
        y[i] = logf(y[i]);
        x[i] = logf(x[i]);
    }
    for(i = 0; i < n; i++)
    {
        sumx += x[i];
        sumy += y[i];
        sumx2 += x[i] * x[i];
        sumxy += x[i] * y[i];
    }
    equ[0][0] = n;
    equ[0][1] = sumx;
    equ[0][2] = sumy;
    equ[1][0] = sumx;
    equ[1][1] = sumx2;
    equ[1][2] = sumxy;
    //sleep(1);
    linEquSolve(&equ[0][0], 2);
    //sleep(1);
    *a = equ[0][2];
    *b = equ[1][2];
	}
	
	void WaitIrTouch(Infrared *IR,unsigned char chMask, int hTh, int lTh)
	{
		OS_ERR  err;
    while(true)
    {
        OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
        if(     ((IR->IRInts.FL < lTh) || !(chMask & IrCh::FL)) &&
                ((IR->IRInts.FR < lTh) || !(chMask & IrCh::FR)) &&
                ((IR->IRInts.SL < lTh) || !(chMask & IrCh::SL)) &&
                ((IR->IRInts.SR < lTh) || !(chMask & IrCh::SR)) )
            break;
//				printf("Wait for ir touch low1\r\n");
    }
    while(true)
    {
        OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
        if(     ((IR->IRInts.FL > hTh) && (chMask & IrCh::FL)) ||
                ((IR->IRInts.FR > hTh) && (chMask & IrCh::FR)) ||
                ((IR->IRInts.SL > hTh) && (chMask & IrCh::SL)) ||
                ((IR->IRInts.SR > hTh) && (chMask & IrCh::SR))
                )
            break;
//				printf("Wait for ir touch high1\r\n");
    }
    while(true)
    {
        OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
        if(     ((IR->IRInts.FL < lTh) || !(chMask & IrCh::FL)) &&
                ((IR->IRInts.FR < lTh) || !(chMask & IrCh::FR)) &&
                ((IR->IRInts.SL < lTh) || !(chMask & IrCh::SL)) &&
                ((IR->IRInts.SR < lTh) || !(chMask & IrCh::SR))
                )
            break;
//				printf("Wait for ir touch low2\r\n");
    }
	}
	
	void IRCorrection::doIrCorrection()
	{
//		float flnsDist[3]={.084f,.134f,.214f};float flnsInts[3]; int flnsIdx = 0;
//    float frnsDist[3]={.084f,.134f,.214f};float frnsInts[3]; int frnsIdx = 0;
//    float lsDist[3]={.059f,.084f,.134f}; float lsInts[3]; int lsIdx = 0;
//    float rsDist[3]={.059f,.084f,.134f}; float rsInts[3]; int rsIdx = 0;
		float flnsDist[3]={.084f,.134f,.204f};float flnsInts[3]; int flnsIdx = 0;
    float frnsDist[3]={.084f,.134f,.204f};float frnsInts[3]; int frnsIdx = 0;
    float lsDist[3]={.059f,.084f,.109f}; float lsInts[3]; int lsIdx = 0;
    float rsDist[3]={.059f,.084f,.109f}; float rsInts[3]; int rsIdx = 0;
		
		float aim;
		
		OS_ERR  err;
		
		printf("Arrange walls like this:\r\n");
    printf("\t+---+---+\r\n");
    printf("\t|       |\r\n");
    printf("\t+       +\r\n");
    printf("\t|   @   |\r\n");
    printf("\t+---+---+\r\n");
		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
		printf("Push mouse forward gentlely.\r\n");
		
		for(aim = 0; aim < 3; aim += 1)//for(aim = 0.03f; aim < .111f; aim += 0.01f)   // 9 times
    {
			WaitIrTouch(IR, IrCh::SR, 45000, 40000);
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
			flnsInts[flnsIdx] = IR->IRInts.FL;
      //frnsInts[frnsIdx] = IR->IRInts.FR;
//			sprintf(str, "\tFLns[%2d] = %4d @%3dmm\r\n", flnsIdx, (int)flnsInts[flnsIdx], (int)(1000.0f * flnsDist[flnsIdx]));
			printf("\tFLns[%2d] = %4d @%3dmm\r\n",flnsIdx, (int)flnsInts[flnsIdx], (int)(1000.0f * flnsDist[flnsIdx]));
//      UartPrint(str);
			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
			WaitIrTouch(IR, IrCh::SL, 45000, 40000);
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
			frnsInts[frnsIdx] = IR->IRInts.FR;
//      sprintf(str, "\tFRns[%2d] = %4d @%3dmm\r\n", frnsIdx, (int)frnsInts[frnsIdx], (int)(1000.0f * frnsDist[frnsIdx]));
//      UartPrint(str);
			printf("\tFRns[%2d] = %4d @%3dmm\r\n", frnsIdx, (int)frnsInts[frnsIdx], (int)(1000.0f * frnsDist[frnsIdx]) );
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
      flnsIdx++;frnsIdx++;
		}
		
		irApprox2nd(flnsInts, flnsDist, flnsIdx, IrACs.k[0]);
		irApprox2nd(frnsInts, frnsDist, frnsIdx, IrACs.k[1]);
//			sprintf(str, ": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[0][0], IrACs.k[0][1], IrACs.k[0][2]);
//      UartPrint(str);
		printf(": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[0][0], IrACs.k[0][1], IrACs.k[0][2]);
//			sprintf(str, ": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[1][0], IrACs.k[1][1], IrACs.k[1][2]);
//      UartPrint(str);
		printf(": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[1][0], IrACs.k[1][1], IrACs.k[1][2]);
			
		printf("Arrange walls like this:\r\n");
		printf("\t+---+\n");
		printf("\t|   |\n");
		printf("\t+   +\n");
		printf("\t|  @|\n");
		printf("\t+---+\n");
		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
		printf("Push mouse forward gentlely.\r\n");
		for(aim = 0; aim < 3; aim += 1)//for(aim = 0.03f; aim < .111f; aim += 0.01f)   // 9 times
		{
			WaitIrTouch(IR,IrCh::FL, 35000, 30000);
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
      lsInts[lsIdx] = IR->IRInts.SL;
//      sprintf(str, "\tSL[%2d] = %4d @%3dmm\r\n", lsIdx, (int)lsInts[lsIdx], (int)(1000.0f * lsDist[lsIdx]));
//      UartPrint(str); OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
			printf("\tSL[%2d] = %4d @%3dmm\r\n", lsIdx, (int)lsInts[lsIdx], (int)(1000.0f * lsDist[lsIdx]));
			lsIdx++;
		}
		
		irApprox2nd(lsInts, lsDist, lsIdx, IrACs.k[2]);
//		sprintf(str, ": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[2][0], IrACs.k[2][1], IrACs.k[2][2]);
//    UartPrint(str);
		printf(": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[2][0], IrACs.k[2][1], IrACs.k[2][2]);
		
		printf("Arrange walls like this:\r\n");
    printf("\t+---+\n");
    printf("\t|   |\n");
    printf("\t+   +\n");
    printf("\t|@  |\n");
    printf("\t+---+\n");
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    printf("Push mouse forward gentlely.\r\n");
    for(aim = 0; aim < 3; aim += 1)//for(aim = 0.03f; aim < .111f; aim += 0.01f)   // 9 times
    {
			WaitIrTouch(IR, IrCh::FR, 35000, 30000);
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
      rsInts[rsIdx] = IR->IRInts.SR;
//      sprintf(str, "\tSR[%2d] = %4d @%3dmm\r\n", rsIdx, (int)rsInts[rsIdx], (int)(1000.0f * rsDist[rsIdx]));
//      UartPrint(str);
			printf("\tSR[%2d] = %4d @%3dmm\r\n", rsIdx, (int)rsInts[rsIdx], (int)(1000.0f * rsDist[rsIdx]));
			OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
      rsIdx++;
    }
		irApprox2nd(rsInts, rsDist, rsIdx, IrACs.k[3]);
//		sprintf(str, ": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[3][0], IrACs.k[3][1], IrACs.k[3][2]);
//        UartPrint(str);
		printf(": d = e^(%8.5f + %8.5f*log(i) + %8.5f*log^2(i))\r\n", IrACs.k[3][0], IrACs.k[3][1], IrACs.k[3][2]);
		
//		STMFLASH_Write(FLASH_SAVE_ADDR,(u32 *)&IrLTs.Dists[0],(sizeof(IrLookupTable)/4 + sizeof(IrLookupTable)%4?1:0));
//		STMFLASH_Write(FLASH_SAVE_ADDR + 320,(u32 *)&IrACs.k[0],16);
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32 *)&IrACs,6);
		STMFLASH_Write(FLASH_SAVE_ADDR1,(u32 *)&IrACs.k[2],6);
		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    printf("\n+--------------------------+\r\n");
    printf("|  !!! Congratulation !!!  |\r\n");
    printf("|  Ir correction finished. |\r\n");
    printf("+--------------------------+\r\n");
    OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
	}
	
	void IRCorrection::doIrCorrection1(){
		OS_ERR  err;
		float flnsDist[3]={.084f,.134f,.204f};float flnsInts[3]; int flnsIdx = 0;
    float frnsDist[3]={.084f,.134f,.204f};float frnsInts[3]; int frnsIdx = 0;
    float lsDist[4]={.059f,.084f,.114f,.18f}; float lsInts[4]; int lsIdx = 0;
    float rsDist[4]={.059f,.084f,.114f,.18f}; float rsInts[4]; int rsIdx = 0;
//		flnsInts[0] = 26212.f;
//		frnsInts[0] = 27592.f;
//		flnsInts[1] = 9168.f;
//		frnsInts[1] = 9208.f;
//		flnsInts[2] = 4688.f;
//		frnsInts[2] = 3344.f;
//		
//		lsInts[0] = 35864.f;
//		lsInts[1] = 15120.f;
//		lsInts[2] = 8496.f;	//14864
//		
//		rsInts[0] = 33144.f;
//		rsInts[1] = 15568.f;//35297.f;
//		rsInts[2] = 9520.f;
		
//		flnsInts[0] = 23590.f;
//		frnsInts[0] = 30160.f;
//		flnsInts[1] = 8907.f;
//		frnsInts[1] = 9542.f;
//		flnsInts[2] = 4249.f;
//		frnsInts[2] = 3850.f;
//		
//		lsInts[0] = 22770.f;
//		lsInts[1] = 9069.f;
//		lsInts[2] = 4359.f;	//14864
//		
//		rsInts[0] = 25849.f;
//		rsInts[1] = 11056.f;//35297.f;
//		rsInts[2] = 6752.f;

		flnsInts[0] = 24522.f;
		frnsInts[0] = 32011.f;
		flnsInts[1] = 9999.f;
		frnsInts[1] = 13322.f;
		flnsInts[2] = 4834.f;
		frnsInts[2] = 5370.f;
		
		lsInts[0] = 23835.f;
		lsInts[1] = 12200.f;
		lsInts[2] = 5880.f;	//14864
		lsInts[3] = 8.f;
		
		rsInts[0] = 29149.f;
		rsInts[1] = 13101.f;//35297.f;
		rsInts[2] = 8964.f;
		rsInts[3] = 8.f;
		
		irApprox2nd(flnsInts, flnsDist, 3, IrACs.k[0]);
		irApprox2nd(frnsInts, frnsDist, 3, IrACs.k[1]);
		irApprox2nd(lsInts, lsDist, 4, IrACs.k[2]);
		irApprox2nd(rsInts, rsDist, 4, IrACs.k[3]);
		
		printf(": d = e^(%8.7f + %8.7f*log(i) + %8.7f*log^2(i))\r\n", IrACs.k[0][0], IrACs.k[0][1], IrACs.k[0][2]);
		printf(": d = e^(%8.7f + %8.7f*log(i) + %8.7f*log^2(i))\r\n", IrACs.k[1][0], IrACs.k[1][1], IrACs.k[1][2]);
		printf(": d = e^(%8.7f + %8.7f*log(i) + %8.7f*log^2(i))\r\n", IrACs.k[2][0], IrACs.k[2][1], IrACs.k[2][2]);
		printf(": d = e^(%8.7f + %8.7f*log(i) + %8.7f*log^2(i))\r\n", IrACs.k[3][0], IrACs.k[3][1], IrACs.k[3][2]);
		printf("\r\n");
		printf("%8.7ff, %8.7ff, %8.7f, \r\n", IrACs.k[0][0], IrACs.k[0][1], IrACs.k[0][2]);
		printf("%8.7ff, %8.7ff, %8.7f, \r\n", IrACs.k[1][0], IrACs.k[1][1], IrACs.k[1][2]);
		printf("%8.7ff, %8.7ff, %8.7f, \r\n", IrACs.k[2][0], IrACs.k[2][1], IrACs.k[2][2]);
		printf("%8.7ff, %8.7ff, %8.7f \r\n", IrACs.k[3][0], IrACs.k[3][1], IrACs.k[3][2]);
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32 *)&IrACs,6);
		STMFLASH_Write(FLASH_SAVE_ADDR1,(u32 *)&IrACs.k[2],6);
		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
    printf("\n+--------------------------+\r\n");
    printf("|  !!! Congratulation !!!  |\r\n");
    printf("|  Ir correction finished. |\r\n");
    printf("+--------------------------+\r\n");
    OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
	}

}
