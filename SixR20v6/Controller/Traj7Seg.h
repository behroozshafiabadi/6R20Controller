#pragma once
#include <TcMath.h>
#include <RtlR0.h>	//abs
#include "TrajectoryPoint.h"
#include <iostream.h>
#include <vector>
#include <string.h>
//#include <numeric>
//#include <iostream>
#include <string>
#include <map> 
#include <algorithm>    // std::min
#define M_PI  3.14159265358979323846  /* pi */

class Traj7Seg {
public:
	int LastIkSolutionBranchNumber = 0;
	double ValidJointSpace[6] = { 165, 100, 95, 180, 100, 180 };	
	const double L[6] = { 389.5, 0 , 600, 200, 685.5, 135 };
	static double QbaseGlobal[8];// = { 1,0,0,0,0,0,0,0 };
	static double toolParamGlobal[8];// = { 1,0,0,0,0,0,0,0 };
	static double point2Circ[8];
	static double point3Circ[4];
	static int feed;

	double QEndEffector[8] = { 1, 0,0,0,0 , L[5], 0 ,0 };//QT
	double DriveEncoderRes = 524287;

	void jogCartesian(double ActualPos[], double out[]);
	//double PulsToDegFactor1[6] = { 360.0 / (DriveEncoderRes * 162.0), 360.0 / (DriveEncoderRes * 161.0), -1.0 * 360.0 / (DriveEncoderRes * 161.0), 360.0 / (DriveEncoderRes * 102.0), 360.0 / (DriveEncoderRes * 100.0), (-1.0 * 360.0) / (DriveEncoderRes * 102.0) };
	double PulsToDegFactor1[6] = { 360.0 / (DriveEncoderRes * 162.0), 360.0 / (DriveEncoderRes * 161.0), -1.0 * 360.0 / (DriveEncoderRes * 161.0), 360.0 / (DriveEncoderRes * 102.0), 360.0 / (DriveEncoderRes * 100.0), (-1.0 * 360.0) / (DriveEncoderRes * 102.0) };
	//double PulsToDegFactor1[6] = { 360.0 / DriveEncoderRes, 360.0 / DriveEncoderRes, (-1.0 * 360.0) / DriveEncoderRes, 360.0 / DriveEncoderRes, 360.0 / DriveEncoderRes , (-1.0 * 360.0) / DriveEncoderRes };
	void Traj7Seg::addArray(int a[], int b[], int out[]);
	int Traj7Seg::add(int a, int b);
	void DQmultiply(double Q1[], double Q2[], double out[]);
	void DQinv(double Q1[] , double out[]);
	void GetCartPos(double theta[6], double ToolParams[], double out[]);//mnr
	void Inversekinematic(double MT[], double QBase[], double QTool[], double CurPos[], double Q[6]);// (double QG[], double ToolParams[], double QBase[], double out[8][6]);//mnr
	TrajectoryPointList<double> Traj7Seg::SingleAxisTraj(TrajectoryPoint p0, TrajectoryPoint p1, double vmax, double amax, double jmax, double TS, double landa);//, TrajectoryPointList<double> out);
	void Traj7Seg::MultiAxisTraj(TrajectoryPoint p0[], TrajectoryPoint p1[], double vmax[], double amax[], double jmax[], double TS, double landa, TrajectoryPointList<double> out[]);
	void Traj7Seg::PTPList(double ActualPos[], double vals[], TrajectoryPointList<double> out[]);
	void Traj7Seg::PTPCartesian(double ActualPos[], double vals[], TrajectoryPointList<double> out[]);
	void Traj7Seg::CIRC(double zero[], double one[], double second[], TrajectoryPointList<double> outputs[]);
	void Traj7Seg::sub(double a[], double b[], double out[], int len);
	void Traj7Seg::cross(double a[], double b[], double out[], int len);
	double Traj7Seg::dot(double a[], double b[], int len);
	double Traj7Seg::normA(double a[], int len);
	void Approximation(TrajectoryPointList<double> da1[], TrajectoryPointList<double> da2[], double radius, TrajectoryPointList<double> out[], int &IndPre, int &IndNext);
	//TrajectoryPointList<double>* PTPList(double ActualPos[], std::map <char*, double> dict);
	void LIN(double ActualPos[], double vals[], double toolParams[], bool is_first, bool is_end, TrajectoryPointList<double> outputs[]);
	void OTG();
	void toEulerianAngle(double quar[], double out[]);
	void toQuaternion(double roll, double pitch, double yaw, double out[]);

	double doubleTolerance = 1e-15;

	double Atan3(double  y, double  x)
	{
		double absX = x >= 0 ? x : -x;
		double absY = x >= 0 ? y : -y;
		if (absX < doubleTolerance && absY < doubleTolerance)
			return 0;
		if (absY < doubleTolerance)
		{
			return x > 0 ? 0 : (double)(M_PI);
		}
		if (absX < doubleTolerance)
		{
			return y > 0 ? (double)(M_PI / 2) : -(double)(M_PI / 2);
		}
		return  (double)(atan2_((double)y, (double)x));
	}

	double MatlabMod(double y, double x)
	{
		//return 1;
		double result = -fmod_(y, x);
		if ((y < 0 && x < 0) || (y >= 0 && x >= 0))
		{
			result = fmod_(y, x);
		}
		return result;
	}
};

class BeckhoffContext {
public:
	static double abs(double x) {
		return x >= 0 ? x : -x;
	}
	//static double doubleTolerance;// = 1e-15;
	//BeckhoffContext() {
	//	doubleTolerance = 1e-15;
	//}
	static double Atan3(double  y, double  x)
	{
		double doubleTolerance = 1e-15;
		double absX = x >= 0 ? x : -x;
		double absY = x >= 0 ? y : -y;
		if (absX < doubleTolerance && absY < doubleTolerance)
			return 0;
		if (absY < doubleTolerance)
		{
			return x > 0 ? 0 : (double)(M_PI);
		}
		if (absX < doubleTolerance)
		{
			return y > 0 ? (double)(M_PI / 2) : -(double)(M_PI / 2);
		}
		return  (double)(atan2_((double)y, (double)x));
	}

	static double MatlabMod(double x, double y)
	{
		double result = fmod_(x, y);
		return result >= 0 ? result : result + y;
	}
};
//const double BeckhoffContext::doubleTolerance = 1e-15;