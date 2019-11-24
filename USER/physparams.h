/*
 * phys_params.h
 *
 *  Created on: Aug 7, 2014
 *      Author: loywong
 */

#ifndef PHYSPARAMS_H_
#define PHYSPARAMS_H_

#ifdef __cplusplus
extern "C"{
#endif
#define ENABLE_CORRECTION   1

#define RAD2DEG(x)  ((x) * (180.0f / 3.1415926f))
#define ABSF(x)         ((x) < 0.0f ? -(x) : (x))
#define ABSI(x)         ((x) < 0 ? -(x) : (x))
#define MAX(x, y)   ((x) > (y) ? (x) : (y))

namespace PP
{
	const float Pi = 3.1415926536f;
	//================  physics parameters
	// friction coefficient
	const float Mu     = 0.25f; // 0.72f comment @2015-09-16 17:17
	// gravity constant
	const float g      = 9.80665f;
	// gravity center height
	const float H      = 0.01f;
	// half of distance between 2 tires
	const float W      = 0.05802f / 2.f;
	// wheel
	const float RWheel = 0.01124f;
	// system sampling period, DO NOT change this independently
	// if Ts changed, consider:
	//  * Encoder sampling period
	//  * PID controller
	//  * Imu auto adj zero
	//  * Motion seq calc
	const float Ts     = 0.001f;
	const float EncRes = 9.f;
	const float EncPosUnit = Pi * RWheel / EncRes / 2;
	const float EncAngUnit = EncPosUnit / W;
	const float EncVelUnit = EncPosUnit / Ts;
	const float EncOmgUnit = EncAngUnit / Ts;
	//================ end of physics
	//================ geometry informations
	// gravity center to the center of 4 wheels
	const float GvCenterFwd = 0.000f;
	// all geometry of mouse related to GRAVITY CENTER
	const float BodyLength  = 0.0753f;
	const float BodyWidth   = 0.0676f;
	const float HeadFwd     = 0.04585 - GvCenterFwd;
	// geometry center forward
	//const float GeoFwd      = 0.000f - GvCenterFwd;
	// !!! straight segment must be no less then 2x GeoFwd !!!
	// tail
	const float TailBack    = 0.02966f + GvCenterFwd;
	// ir position related to gravity center
	// forward ir forward dist to gravity center, approx
	const float IrFFwd      = 0.01618f - GvCenterFwd;
	// forward ir side dist to gravity center, approx
	const float IrFSide     = 0.02695f;
	// side ir forward dist to gravity center, approx
	const float IrSFwd      = 0.02942f - GvCenterFwd;
	// side ir side dist to gravity center, approx
	const float IrSSide     = 0.0103f;
	const float IrFLRDist   = 2.f * IrFSide;
	// angle between side ir emit dir and fwd
	const float IrSAngle    = 52.5f / 180.0f * 3.1415926536f;
	// grids
	const float GridSize       	= 0.180f;
	const float WallThick      	= 0.012f;
	const float CenterToWall   	= (GridSize - WallThick) / 2.0f;
	const float StartLengthCorr = 0.019836f;//0.19836
	const float StartTotalDist 	= GridSize - WallThick / 2.0f - TailBack + StartLengthCorr;
	const float StartAcclDist  	= 0.030f;
	const float StopLengthCorr = -0.027851f;
	const float StopTotalDist  	= GridSize / 2.0f + StopLengthCorr;
	const float StopAccDist    	= 0.0450f;
	const float RestartLengthCorr = 0.021051f;
	const float RestartDist    	= GridSize / 2.0f + RestartLengthCorr;
	const float RestartAccDist  = 0.030f;
	
	//================ end geometry infos
	//================ imu & encoder
	const float GyroVariance	= (.00123f * .00123f * 1.f);
	const float AcclVariance	= (.055f * .055f * 10000.f);
	const float EncVelVariance  = EncVelUnit * EncVelUnit / 12.f / 2.f;
	const float EncOmgVariance  = EncOmgUnit * EncOmgUnit / 12.f / 2.f;
	const float EncPosVariance	= EncPosUnit * EncPosUnit / 12.f / 2.f;
	const float EncAngVariance  = EncAngUnit * EncAngUnit / 12.f / 2.f;
	//================ action speeds & etc.
	const int SeqArrayLen		= 1500;
	const float SearchSpeed    	= 0.55f;	//0.36f comment @20150916 17:18
	
	const float RushSpeedLow    	= 1.0f;	//0.36f comment @20150916 17:18
	const float RushSpeedMidLow    	= 1.4f;
	const float RushSpeedMid      = 1.7f;
	const float RushSpeedHighLow    	= 2.0f;
	const float RushSpeedHigh			= 2.5f;
	const float XSpeedMid    	= 1.7f;
	const float XSpeedMidLow    	= 1.35f;
	//================
	
	const float FWD_WALL_TL = .225f;
	const float LEFT_WALL_TL = .10f;
	const float RIGHT_WALL_TL = .12f;
	
	const float FWD_WALL_TH = .265f;
	const float LEFT_WALL_TH = .125f;
	const float RIGHT_WALL_TH = .155f;
	
	const float FWD_WALL_TH_RUN = .255f;
	const float LEFT_WALL_TH_RUN = .101f;
	const float RIGHT_WALL_TH_RUN = .101f;
	
	const float AcclUnit = 4.f * g /32768.f;
	const float GyroUnit = 2000.f / 32768.f/180.f * Pi / 2.f * 0.99f; 
}


namespace BCP
{
//================ basic correction coefficients

	const	float Encoder = 1.000f;		// reduce to run farther
	const	float Gyro    = 1.000f;		// reduce to turn/rotate more
	const	float Accl    = 1.000f;		// reduce to run farther


	const	float RPre 	=  0.012f;
	const	float RPost	=  0.009f;
	const	float LPre	=  0.012f;
	const	float LPost	=  0.008f;

}


namespace SCP
{
		const float StartYawBgn = 0.000f;
		const float StartYawEnd = 0.03f;
		const float StartDisEnd = PP::StartTotalDist - 0.01f;
		const float RestartDisEnd = PP::RestartDist - 0.01f;
		const float StartReport = PP::StartTotalDist - 0.01f;
		const float RestartReport = PP::RestartDist - 0.01f;
		const float StopYawBgn  = 0.000f;
		const float StopYawEnd  = 0.015f;
		const float FwdLengthCorr = +0.00207f;
		const float FwdYawBgn   = 0.000f;
		const float FwdYawEnd   = 0.04f;
		const float FwdReport   = PP::GridSize - 0.01f + FwdLengthCorr;
		const float CtpdStatBgn = 0.040f;
		const float CtpdYawBgn  = 0.140f;
		const float CtpdYawEnd  = CtpdYawBgn + 0.667f * (PP::GridSize - CtpdYawBgn);
		const float TrunYawEnd  = 0.01f;
		const float TurnDirEnd  = 0.005f;
		const float AfterTrunReadyDist = 0.012f;
	
		
		const float RushStartDist =	.088f;
		const float DIST_SPEED_CORR_1 = (-.0225f);
		const float DIST_SPEED_CORR_2 = (.0225f);	

		const float LEFT_WALLDIS_POS = .006f;
		const float RIGHT_WALLDIS_POS = .008f;
		
		const int LDISTSTP_HTH = 6700;
		const int LDISTSTP_LTH = 6600;
		const int RDISTSTP_HTH = 7700;
		const int RDISTSTP_LTH = 7600;
		
		const float RUSH_STOP_DIST = .142f;
		
		const float XLPOSTADJ	= -.00f;
		const float XRPOSTADJ = -.00f;
		
		const float R45IPRE			=.010f;
		const float R45IPOST		=.0895f;
		const float L45OPRE			=.0535f;
		const float L45OPOST		=.0567f;

		const float L45iPRE			=.029f;
		const float L45iPOST		=.096f;
		const float R45oPRE			=.030f;
		const float R45oPOST		=.061f;

		const float RL90PRE			=.066f;
		const float RL90POST		=.089f;
		const float RR90PRE			=.063f;
		const float RR90POST		=.0895f;

		const float R180_MU_ADJ	=1.16f;
		const float R180PRE			=.037f;
		const float R180POST		=.089f;

		const float L180_MU_ADJ	=1.16f;
		const float L180PRE			=.038f;
		const float L180POST		=.068f;

		const float LR135_MU_ADJ=1.34f;
		const float R135iPRE		=.076f;
		const float R135iPOST		=.061f;
		const float L135oPRE		=.019f;
		const float L135oPOST		=.081f;
		const float L135iPRE		=.052f;
		const float L135iPOST		=.067f;
		const float R135oPRE		=.013f;
		const float R135oPOST		=.0835f;

		const float XL90PRE			=.008f;
		const float XL90POST		=.041f;
		const float XR90PRE			=.004f;
		const float XR90POST		=.045f;
		
	
		const float FwdL	=  0.091f;//0.068
		const float FwdR	=  0.066f;//0.081f
//		const float FwdL	=  0.045f;
//		const float FwdR	=  0.047f;
		const float FwdL_Show	=  0.107f;
		const float FwdR_Show	=  0.09f;
		const float FwdL_Show_Nowall	=  0.116f;
		const float FwdR_Show_Nowall	=  0.114f;
		const float Stop	=  -0.01f;

		// turn wait dist adjustment
		const float L    =  -0.018f;//-0.049f;    // more positive to turn later
		const float R    =  -0.0254f; //-0.0154f;   // more positive to turn later

		const float Yaw = 0.00f;
		const float Pos = -0.004f;

}



#ifdef __cplusplus
}
#endif
#endif /* PHYS_PARAMS_H_ */
