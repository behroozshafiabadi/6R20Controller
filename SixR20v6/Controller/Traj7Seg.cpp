///////////////////////////////////////////////////////////////////////////////
// Module1.cpp
#include "TcPch.h"
#pragma hdrstop
#include "TrajectoryPoint.h"
#include "Traj7Seg.h"
#include "slerp.h"
using namespace std;

ULONG TrajectoryPoints::dooo(ULONG a) {
	return a * 2;
}
int Traj7Seg::add(int a, int b) {
	int c;
	c = a + b;
	return c;
}
void Traj7Seg::jogCartesian(double ActualPos[], double out[]) {
	double currentPos[8];
	GetCartPos(ActualPos, toolParamGlobal, currentPos);
	double rpy[3];
	toEulerianAngle(currentPos, rpy);
}
void Traj7Seg::addArray(int a[], int b[], int out[]) {
	for (int i = 0; i < 6; i++) {
		out[i] = a[i] + b[i];
	}
}
void Traj7Seg::GetCartPos(double theta[6], double ToolParams[], double out[])//mnr
{
	double temp[] = { theta[0] , theta[1], theta[2],theta[3],theta[4],theta[5],theta[6] };
	double Q1[] = { (double)(cos_((double)theta[0] / 2.0)), 0, 0,(double)(sin_((double)theta[0] / 2.0)), 0, 0, 0, L[0] };
	double Q2[] = { (double)(cos_((double)theta[1] / 2.0)), 0,  (double)(sin_((double)theta[1] / 2.0)), 0, 0, 0, 0, 0 };
	double Q3[] = { (double)(cos_((double)theta[2] / 2.0)), 0, (double)(sin_((double)theta[2] / 2.0)), 0, 0, 0, 0, L[2] };
	double Q4[] = { (double)(cos_((double)theta[3] / 2.0)), (double)(sin_((double)theta[3] / 2.0)), 0, 0, 0, L[4], 0, L[3] };
	double Q5[] = { (double)(cos_((double)theta[4] / 2.0)), 0, (double)(sin_((double)theta[4] / 2.0)), 0, 0, 0, 0, 0 };
	double Q6[] = { (double)(cos_((double)theta[5] / 2.0)), (double)(sin_((double)theta[5] / 2.0)), 0, 0, 0, 0, 0, 0 };

	double M7[8];
	DQmultiply(QEndEffector, ToolParams, M7);
	double M6[8];
	DQmultiply(Q6, M7, M6);
	double M5[8];
	DQmultiply(Q5, M6, M5);
	double M4[8];
	DQmultiply(Q4, M5, M4);
	double M3[8];
	DQmultiply(Q3, M4, M3);
	double M2[8];
	DQmultiply(Q2, M3, M2);
	DQmultiply(Q1, M2, out);
}
void Traj7Seg::DQinv(double Q1[], double Q[])
{
	//double Q[8];
	Q[0] = Q1[0];
	Q[1] = -Q1[1];
	Q[2] = -Q1[2];
	Q[3] = -Q1[3];
	Q[4] = 0;
	Q[5] = -Q1[5] - Q1[2] * (Q1[1] * Q1[6] - Q1[2] * Q1[5]) * 2 + Q1[0] * (Q1[2] * Q1[7] - Q1[3] * Q1[6]) * 2 - Q1[3] * (Q1[1] * Q1[7] - Q1[3] * Q1[5]) * 2;
	Q[6] = -Q1[6] + Q1[1] * (Q1[1] * Q1[6] - Q1[2] * Q1[5]) * 2 - Q1[0] * (Q1[1] * Q1[7] - Q1[3] * Q1[5]) * 2 - Q1[3] * (Q1[2] * Q1[7] - Q1[3] * Q1[6]) * 2;
	Q[7] = -Q1[7] + Q1[0] * (Q1[1] * Q1[6] - Q1[2] * Q1[5]) * 2 + Q1[1] * (Q1[1] * Q1[7] - Q1[3] * Q1[5]) * 2 + Q1[2] * (Q1[2] * Q1[7] - Q1[3] * Q1[6]) * 2;
	//return Q;
}
void Traj7Seg::DQmultiply(double Q1[], double Q2[], double out[])
{
	out[0] = Q1[0] * Q2[0] - Q1[1] * Q2[1] - Q1[2] * Q2[2] - Q1[3] * Q2[3];
	out[1] = Q1[0] * Q2[1] + Q1[1] * Q2[0] + Q1[2] * Q2[3] - Q1[3] * Q2[2];
	out[2] = Q1[0] * Q2[2] + Q1[2] * Q2[0] - Q1[1] * Q2[3] + Q1[3] * Q2[1];
	out[3] = Q1[0] * Q2[3] + Q1[1] * Q2[2] - Q1[2] * Q2[1] + Q1[3] * Q2[0];
	out[4] = 0;
	out[5] = Q1[5] + Q2[5] + Q1[2] * (Q1[1] * Q2[6] - Q1[2] * Q2[5]) * 2 + Q1[0] * (Q1[2] * Q2[7] - Q1[3] * Q2[6]) * 2 + Q1[3] * (Q1[1] * Q2[7] - Q1[3] * Q2[5]) * 2;
	out[6] = Q1[6] + Q2[6] - Q1[1] * (Q1[1] * Q2[6] - Q1[2] * Q2[5]) * 2 - Q1[0] * (Q1[1] * Q2[7] - Q1[3] * Q2[5]) * 2 + Q1[3] * (Q1[2] * Q2[7] - Q1[3] * Q2[6]) * 2;
	out[7] = Q1[7] + Q2[7] + Q1[0] * (Q1[1] * Q2[6] - Q1[2] * Q2[5]) * 2 - Q1[1] * (Q1[1] * Q2[7] - Q1[3] * Q2[5]) * 2 - Q1[2] * (Q1[2] * Q2[7] - Q1[3] * Q2[6]) * 2;

	//return Q;
}
void Traj7Seg::Inversekinematic(double MT[], double QBase[], double QTool[], double CurPos[], double Q[6])//mnr
{
	double InK[8][6];// = double[8][6];
	//Ink[1][1] = 1;
	double M8[8], M7[8], M6[8], M5[8], M4[8], M3[8], M2[8];
	double res[8];
	DQinv(QBase, res);
	DQmultiply(res, MT, M8);

	DQinv(QTool, res);
	DQmultiply(M8, res, M7);

	DQinv(QEndEffector, res);
	DQmultiply(M7, res, M6);

	double Qg1 = round_digits_(M6[0],4), Qg2 = round_digits_(M6[1],4), Qg3 = round_digits_(M6[2],4), 
		Qg4 = round_digits_(M6[3],4), Qg5 = round_digits_(M6[4],4), Qg6 = round_digits_(M6[5],4), 
		Qg7 = round_digits_(M6[6],4), Qg8 = round_digits_(M6[7],4);
	double Teta1[2], Teta2[2][2], Teta3[2][2], Teta4[2][2][2], Teta5[2][2][2], Teta6[2][2][2];
	double N4[4];
	double t1, t2, t3, t4, t5, t6;
	double H1;
	for (int i = 1; i <= 2; i++) {
		//%Teta1
		double a = Qg6;
		double b = Qg7;
		double c = 0;
		Teta1[i - 1] = atan2_(b, a) - atan2_(c, (-2 * i + 3)*sqrt_(a*a + b*b - c*c));
		InK[(4 * i  - 3) - 1][0] = Teta1[i - 1];// *180 / M_PI;
		InK[(4 * i  - 2) - 1][0] = Teta1[i - 1];// *180 / M_PI;
		InK[(4 * i  - 1) - 1][0] = Teta1[i - 1];// *180 / M_PI;
		InK[(4 * i  - 0) - 1][0] = Teta1[i - 1];// *180 / M_PI;
		t1 = round_digits_(Teta1[i - 1],4);

		for (int j = 1; j <= 2; j++) {
			//%Teta2
			a = (-2 * L[2] * Qg6*cos_(t1) - 2 * L[2] * Qg7*sin_(t1) + 2 * L[1] * L[2]);
			b = (2 * L[2] * Qg8 - 2 * L[0] * L[2]);
			c = Qg6 * Qg6 * cos_(t1) *cos_(t1) - Qg7 *Qg7 * cos_(t1) *cos_(t1) + L[0] * L[0] + L[1] * L[1] + L[2] * L[2] + Qg7 * Qg7 + Qg8 * Qg8 - 2 * L[0] * Qg8 + Qg6*Qg7*sin_(2 * t1) - 2 * L[1] * Qg6*cos_(t1) - 2 * L[1] * Qg7*sin_(t1) - L[3] * L[3] - L[4] * L[4];
			Teta2[i-1][j-1] = atan2_(b, a) - atan2_(c, (2 * j - 3)*sqrt_(BeckhoffContext::abs(a *a + b *b - c *c)));
			InK[(4 * i + 2 * j  - 5) - 1][1] = Teta2[i-1][j-1];// *180 / M_PI;
			InK[(4 * i  + 2 * j  - 4) -1][1] = Teta2[i-1][j-1];// *180 / M_PI;
			t2 = round_digits_(Teta2[i-1][j-1],4);
			H1 = a *a + b*b - c *c;
			// % Verification of Secend Branch of Teta1
			if ((a *a + b*b - c *c) < 0)
				H1 = 1;

			//%Teta3
			a = 2 * L[2] * L[4];
			b = 2 * L[2] * L[3];
			c = (Qg6*cos_(t1) + Qg7*sin_(t1) - L[1]) * (Qg6*cos_(t1) + Qg7*sin_(t1) - L[1]) + (Qg8 - L[0]) *(Qg8 - L[0]) - L[2] * L[2] - L[3] * L[3] - L[4] * L[4];

			Teta3[i-1][j-1] = atan2_(b, a) - atan2_(c, (-2 * j + 3)*sqrt_(BeckhoffContext::abs(a *a + b*b - c *c)));
			InK[(4 * i  + 2 * j  - 5) - 1][2] = Teta3[i-1][j-1];// *180 / pi;
			InK[(4 * i  + 2 * j  - 4) - 1][2] = Teta3[i-1][j-1];// *180 / pi;
			t3 = round_digits_(Teta3[i-1][j-1],4);
			for (int k = 1; k <= 2; k++) {

				N4[0] = cos_(t3 / 2)*(cos_(t2 / 2)*(Qg1*cos_(t1 / 2) + Qg4*sin_(t1 / 2)) + sin_(t2 / 2)*(Qg3*cos_(t1 / 2) - Qg2*sin_(t1 / 2))) + sin_(t3 / 2)*(cos_(t2 / 2)*(Qg3*cos_(t1 / 2) - Qg2*sin_(t1 / 2)) - sin_(t2 / 2)*(Qg1*cos_(t1 / 2) + Qg4*sin_(t1 / 2)));
				N4[1] = cos_(t3 / 2)*(cos_(t2 / 2)*(Qg2*cos_(t1 / 2) + Qg3*sin_(t1 / 2)) - sin_(t2 / 2)*(Qg4*cos_(t1 / 2) - Qg1*sin_(t1 / 2))) - sin_(t3 / 2)*(cos_(t2 / 2)*(Qg4*cos_(t1 / 2) - Qg1*sin_(t1 / 2)) + sin_(t2 / 2)*(Qg2*cos_(t1 / 2) + Qg3*sin_(t1 / 2)));
				N4[2] = cos_(t3 / 2)*(cos_(t2 / 2)*(Qg3*cos_(t1 / 2) - Qg2*sin_(t1 / 2)) - sin_(t2 / 2)*(Qg1*cos_(t1 / 2) + Qg4*sin_(t1 / 2))) - sin_(t3 / 2)*(cos_(t2 / 2)*(Qg1*cos_(t1 / 2) + Qg4*sin_(t1 / 2)) + sin_(t2 / 2)*(Qg3*cos_(t1 / 2) - Qg2*sin_(t1 / 2)));
				N4[3] = cos_(t3 / 2)*(cos_(t2 / 2)*(Qg4*cos_(t1 / 2) - Qg1*sin_(t1 / 2)) + sin_(t2 / 2)*(Qg2*cos_(t1 / 2) + Qg3*sin_(t1 / 2))) + sin_(t3 / 2)*(cos_(t2 / 2)*(Qg2*cos_(t1 / 2) + Qg3*sin_(t1 / 2)) - sin_(t2 / 2)*(Qg4*cos_(t1 / 2) - Qg1*sin_(t1 / 2)));
				a = 2 * atan2_(N4[1], N4[0]);
				b = 2 * atan2_(N4[3], N4[2]);
				//%Teta4
				Teta4[i-1][j-1][k-1] = (a + b) / 2 - (k - 1)*M_PI;
				InK[(4 * i  + 2 * j  + k  - 6) - 1][3] = Teta4[i-1][j-1][k-1];// *180 / M_PI;
				t4 = round_digits_(Teta4[i-1][j-1][k-1],4);
				//%Teta5
				Teta5[i-1][j-1][k-1] = (2 * atan2_(sqrt_((N4[2]) *N4[2] + (N4[3]) *N4[3]), sqrt_((N4[0]) *(N4[0]) + (N4[1]) *(N4[1]))))*(-2 * k + 3);
				InK[(4 * i  + 2 * j  + k  - 6) - 1][4] = Teta5[i-1][j-1][k-1];// *180 / M_PI;
				t5 = round_digits_(Teta5[i-1][j-1][k-1],4);
				//%Teta6
				Teta6[i-1][j-1][k-1] = (a - b) / 2 - (k - 1)*M_PI;
				InK[(4 * i + 2 * j  + k  - 6 ) - 1][5] = Teta6[i-1][j-1][k-1];// *180 / M_PI;
				t6 = round_digits_(Teta6[i-1][j-1][k-1],4);
			}
		}
	}
	double DInk[8][6];
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 6; j++) {
			InK[i][j] = BeckhoffContext::MatlabMod(InK[i][j] + M_PI, 2 * M_PI) - M_PI;
			DInk[i][j] = BeckhoffContext::abs(InK[i][j] - CurPos[j]);
			DInk[i][j] = BeckhoffContext::MatlabMod(DInk[i][j] + M_PI, 2 * M_PI) - M_PI;
			DInk[i][j] = BeckhoffContext::abs(DInk[i][j]);
			if (j < 3)
			{
				DInk[i][j] *= 5;
			}
		}
	}
	int min = 0;
	double minSum = 0;
	if (H1==1) {
		//%     X = ['The Solution Branch #5 to #8 is Fault'];		
		min = 0;
		minSum = 0;
		for (int j = 0; j < 6; j++) {
			minSum += DInk[0][j];
		}
		double sum;
		for (int i = 0; i < 4; i++) {
			sum = 0;
			for (int j = 0; j < 6; j++) {
				sum += DInk[i][j];
			}
			if (sum < minSum) {
				minSum = sum;
				min = i;
			}
		}
		for (int i = 0; i < 6; i++)
			Q[i] = InK[min][i];
	}
	else {
		min = 0;
		minSum = 0;
		for (int j = 0; j < 6; j++) {
			minSum += DInk[0][j];
		}
		double sum;
		for (int i = 0; i < 8; i++) {
			sum = 0;
			for (int j = 0; j < 6; j++) {
				sum += DInk[i][j];
			}
			if (sum < minSum) {
				minSum = sum;
				min = i;
			}
		}
		for (int i = 0; i < 6; i++)
			Q[i] = InK[min][i];
	}
	double a = Q[5] - CurPos[5];
	if (BeckhoffContext::abs(a) >= 2.0 * M_PI * 359.0 / 360.0) {
		double a0 = a;
		if (a > 0) {
			while (BeckhoffContext::abs(a0) >= (M_PI/180)) {
				Q[5] = Q[5] - 2 * M_PI;
				a0 = Q[5] - CurPos[5];
			}
		}
		else {
			while (BeckhoffContext::abs(a0) >= (M_PI / 180)) {
				Q[5] = Q[5]+ 2 * M_PI;
				a0 = Q[5] - CurPos[5];
			}
		}
	}
	double a1 = Q[3] - CurPos[3];
	if (BeckhoffContext::abs(a1) >= 2.0 * M_PI * 359.0 / 360.0) {
		double a00 = a1;
		if (a1 > 0) {
			while (BeckhoffContext::abs(a00) >= (M_PI / 180)) {
				Q[3] = Q[3] - 2 * M_PI;
				a00 = Q[3] - CurPos[3];
			}
		}
		else {
			while (BeckhoffContext::abs(a00) >= (M_PI / 180)) {
				Q[3] = Q[3] + 2 * M_PI;
				a00 = Q[3] - CurPos[3];
			}
		}
	}
	// Modificatin of Wrist Singularity
	double Q46;
	if (BeckhoffContext::abs(CurPos[4]) < 0.001)
	{
		Q46 = Q[3] + Q[5];
		Q[3] = CurPos[3];
		Q[5] = Q46 - Q[3];
	}
}
TrajectoryPointList<double> Traj7Seg::SingleAxisTraj(TrajectoryPoint p0, TrajectoryPoint p1, double vmax, double amax, double jmax, double TS, double landa)//, TrajectoryPointList<double> out)
{
	TrajectoryPointList<double> trjp;// = new TrajectoryPointList<decimal>();	
	//double[] q = new double[100000];
	//double[] v = new double[100000];
	//double[] a = new double[100000];
	double vlim, alima, alimd;
	if (p1.Q - p0.Q == 0)
	{
		trjp.init = false;
		return trjp;// NULL;
	}
	int sigma = 1;
	if (p1.Q < p0.Q)
	{
		sigma = -1;
		p0.Q *= -1;
		p0.V *= -1;
		p1.Q *= -1;
		p1.V *= -1;
	}
	double Tj1, Ta, Tj2, Td;
	if ((vmax - p0.V) * jmax < (amax * amax))
	{
		Tj1 = sqrt_((vmax - p0.V) / jmax);
		Ta = 2 * Tj1;
	}
	else
	{
		Tj1 = amax / jmax;
		Ta = Tj1 + (vmax - p0.V) / amax;
	}

	if ((vmax - p1.V) * jmax < (amax * amax))
	{
		Tj2 = sqrt_((vmax - p1.V) / jmax);
		Td = 2 * Tj2;
	}
	else
	{
		Tj2 = amax / jmax;
		Td = Tj2 + (vmax - p1.V) / amax;
	}
	double Tv = (p1.Q - p0.Q) / vmax - (Ta / 2) * (1 + p0.V / vmax) - (Td / 2) * (1 + p1.V / vmax);
	if (Tv < 0)
	{
		Tv = 0;
		while (true)
		{
			Tj1 = Tj2 = amax / jmax;
			double amax4 = pow_(amax, 4);
			double delta = (amax4 / (jmax * jmax)) + 2 * (p0.V * p0.V + p1.V * p1.V) + amax * (4 * (p1.Q - p0.Q) - 2 * (amax / jmax) * (p0.V + p1.V));
			Ta = ((amax * amax / jmax) - 2 * p0.V + sqrt_(delta)) / (2 * amax);
			Td = ((amax * amax / jmax) - 2 * p1.V + sqrt_(delta)) / (2 * amax);
			if (Ta < 0)
			{
				Ta = 0;
				Td = 2 * (p1.Q - p0.Q) / (p1.V + p0.V);
				Tj2 = (jmax * (p1.Q - p0.Q) - sqrt_(jmax * (jmax * (p1.Q - p0.Q) * (p1.Q - p0.Q) + (p1.V + p0.V) * (p1.V + p0.V) * (p1.V - p0.V)))) / (jmax * (p1.V + p0.V));
			}
			else if (Td < 0)
			{
				Td = 0;
				Ta = 2 * (p1.Q - p0.Q) / (p1.V + p0.V);
				Tj1 = (jmax * (p1.Q - p0.Q) - sqrt_(jmax * (jmax * (p1.Q - p0.Q) * (p1.Q - p0.Q) + (p1.V + p0.V) * (p1.V + p0.V) * (p1.V - p0.V)))) / (jmax * (p1.V + p0.V));
			}

			if (Ta >= 2 * Tj1 && Td >= 2 * Tj2)
			{
				break;
			}
			else
				amax = landa * amax;
		}
		alima = jmax * Tj1;
		alimd = -jmax * Tj2;
		vlim = p0.V + (Ta - Tj1) * alima;
	}
	else
	{
		alima = amax;
		alimd = -amax;
		vlim = vmax;
	}
	double dur = Ta + Tv + Td; //pre process
	double jmin = -jmax;
	int i = 0;
	double t = 0;
	trjp.TrajLength = 0;
	while (t <= dur)
	{
		if (t < Tj1)
		{
			trjp.AddPoint(
				(double)(p0.Q + p0.V * t + jmax * t * t * t / 6),
				(double)(p0.V + jmax * t * t / 2),
				(double)(jmax * t));

		}
		else if (t < Ta - Tj1)
		{
			trjp.AddPoint(
				(double)(p0.Q + p0.V * t + alima * (3 * t * t - 3 * Tj1 * t + Tj1 * Tj1) / 6),
				(double)(p0.V + alima * (t - Tj1 / 2)),
				(double)(alima));
		}
		else if (t < Ta)
		{
			trjp.AddPoint(
				(double)(p0.Q + (vlim + p0.V) * Ta / 2 - vlim * (Ta - t) - jmin * (Ta - t) * (Ta - t) * (Ta - t) / 6),
				(double)(vlim + jmin * (Ta - t) * (Ta - t) / 2),
				(double)(-jmin * (Ta - t)));
		}
		else if (t < Ta + Tv)
		{
			trjp.AddPoint(
				(double)(p0.Q + (vlim + p0.V) * Ta / 2 + vlim * (t - Ta)),
				(double)(vlim),
				0);
		}
		else if (t < dur - Ta + Tj2)
		{
			trjp.AddPoint(
				(double)(p1.Q - (vlim + p1.V) * Td / 2 + vlim * (t - dur + Td) -
					jmax * (t - dur + Td) * (t - dur + Td) * (t - dur + Td) / 6),
					(double)(vlim - jmax * (t - dur + Td) * (t - dur + Td) / 2),
				(double)(-jmax * (t - dur + Td)));
		}
		else if (t < dur - Tj2)
		{
			trjp.AddPoint(
				(double)(p1.Q - (vlim + p1.V) * Td / 2 + vlim * (t - dur + Td) +
					alimd * (3 * (t - dur + Td) * (t - dur + Td) - 3 * Tj2 * (t - dur + Td) + Tj2 * Tj2) / 6),
					(double)(vlim + alimd * (t - dur + Td - Tj2 / 2)),
				(double)(alimd));
		}
		else if (t <= dur)
		{
			trjp.AddPoint(
				(double)(p1.Q - p1.V * (dur - t) - (jmax * (dur - t) * (dur - t) * (dur - t) / 6)),
				(double)(p1.V + (jmax * (dur - t) * (dur - t) / 2)),
				(double)(-jmax * (dur - t)));
		}
		trjp.q.at(i) = trjp.q.at(i) * sigma;
		trjp.v.at(i) = trjp.v.at(i) * sigma;
		trjp.a.at(i) = trjp.a.at(i) * sigma;
		t += TS;
		i++;
	}
	if (p1.Q >= p0.Q)
	{
		p0.Q *= -1;
		p0.V *= -1;
		p1.Q *= -1;
		p1.V *= -1;
	}
	/*if (trjp == null)
		trjp = new TrajectoryPointList<double>();*/
		//for each (double var in trjp.q)
		//{
		//	out->q[i] = var;
		//}
	return trjp;
	/*out.q= trjp.q;
	out.TrajLength = trjp.TrajLength;
	out.v = trjp.v;
	out.init = trjp.init;
	out.a = trjp.a;*/
	//out = trjp;
}
void Traj7Seg::MultiAxisTraj(TrajectoryPoint p0[], TrajectoryPoint p1[], double vmax[], double amax[], double jmax[], double TS, double landa, TrajectoryPointList<double> out[])
{
	//TrajectoryPointList<double> trjp[6];// = new TrajectoryPointList<decimal>[6];
	for (int i = 0; i < 6; i++)
	{
		TrajectoryPoint p00(p0[i].Q, p0[i].V);// = TrajectoryPoint(p0.at(i).Q, p0.at(i).V);
		TrajectoryPoint p11(p1[i].Q, p1[i].V);// = TrajectoryPoint(p1[i].Q, p1[i].V);
		out[i] = SingleAxisTraj(p00, p11, vmax[i], amax[i], jmax[i], TS, landa);
	}
	//out= trjp;
}
void Traj7Seg::PTPList(double ActualPos[], double vals[], TrajectoryPointList<double> out[])
{
	int wmax_def = 300;
	int almax_def = 100; // Rotational Acceleration
	int gamax_def = 250; // Rotational Jerk
	//auto it = std::find(keys.begin(), keys.end(), "F");
	//int index = std::distance(keys.begin(), it);
	double wmax = std::min(vals[6], (double)wmax_def);
	TrajectoryPointList<double> outputs[6];// = new TrajectoryPointList<decimal>[6];
	TrajectoryPoint X0[6];
	//std::vector<TrajectoryPoint> X0;// = new TrajectoryPoint[6];

	for (int i = 0; i < 6; i++)
	{
		X0[i] = TrajectoryPoint(ActualPos[i], 0);
	}

	TrajectoryPoint X1[6];// = new TrajectoryPoint[6];
	double wmaxS[6];// = new double[6];
	double almaxS[6];// = new double[6];
	double gamaxS[6];// = new double[6];

	for (int i = 0; i < 6; i++)
	{
		X1[i] = TrajectoryPoint(vals[i], 0);
		wmaxS[i] = (double)wmax;
		almaxS[i] = almax_def;
		gamaxS[i] = gamax_def;
	}

	TrajectoryPointList<double> points[6];
	MultiAxisTraj(X0, X1, wmaxS, almaxS, gamaxS, .001, .999, points);
	int maxLength = 0;
	if (points[0].init == false)	//null
		points[0] = TrajectoryPointList<double>();
	for (int i = 1; i < 6; i++)
	{

		if (points[i].init == false)// == null)
			points[i] = TrajectoryPointList<double>();
		if (points[i].TrajLength > points[maxLength].TrajLength)
			maxLength = i;
	}
	for (int i = 0; i < 6; i++)
	{
		if (outputs[i].init == false)// == null)
			outputs[i] = TrajectoryPointList<double>();
		double ratio = (X1[i].Q - X0[i].Q) / (X1[maxLength].Q - X0[maxLength].Q);
		for (int j = 0; j < points[maxLength].TrajLength; j++)
			outputs[i].AddPoint((double)(X0[i].Q + ((double)points[maxLength].q[j] - X0[maxLength].Q) * ratio), 0, 0);
		// outputs[i].q[j] = X0[i].Q + (points[maxLength].q[j] - X0[maxLength].Q) * ratio;

		outputs[i].TrajLength = points[maxLength].TrajLength;
	}
	for (int i = 0; i < 6; i++)
	{
		out[i] = outputs[i];
	}

	//out = outputs;
}
void Traj7Seg::PTPCartesian(double ActualPos[], double vals[], TrajectoryPointList<double> out[])
{
	double res[6];
	double Quat[4];
	toQuaternion(vals[3] * (M_PI / 180.0), vals[4] * (M_PI / 180.0), vals[5] * (M_PI / 180.0), Quat);
	double q[] = { Quat[0], Quat[1], Quat[2], Quat[3], 0, vals[0], vals[1], vals[2] };
	//for (i = 0; i < 6; i++) {
	//	actualPos[i] = (actualPos[i] * M_PI) / 180.0; //to radian
	//}
	double tmpActualPos[6] = { (ActualPos[0] * M_PI) / 180.0 , (ActualPos[1] * M_PI) / 180.0 ,(ActualPos[2] * M_PI) / 180.0 ,(ActualPos[3] * M_PI) / 180.0 ,(ActualPos[4] * M_PI) / 180.0 ,(ActualPos[5] * M_PI) / 180.0 };
	Inversekinematic(q, QbaseGlobal, toolParamGlobal, tmpActualPos, res);//, res);
	double targetPos[] = { res[0] * (180.0 / M_PI),res[1] * (180.0 / M_PI),res[2] * (180.0 / M_PI),res[3] * (180.0 / M_PI),res[4] * (180.0 / M_PI),res[5] * (180.0 / M_PI),vals[6],vals[7]};
	PTPList(ActualPos, targetPos, out);
}
void Traj7Seg::LIN(double DQCurrentPosition[], double targetPosition[], double toolParams[], bool is_first, bool is_end, TrajectoryPointList<double> resultList[])
{
	double DQCurrentPositionInRef[8];
	DQmultiply(QbaseGlobal, DQCurrentPosition, DQCurrentPositionInRef);
	double DQTargetPositionInRef[8];
	// for X,Y,Z
	for (int i = 0; i < 3; i++)
	{
		DQTargetPositionInRef[i + 5] = targetPosition[i];
	}
	//for A,B,C
	double QuatRPYTargetPosition[4];
	double QuatRPYCurrentPosition[4] = { DQCurrentPositionInRef[0], DQCurrentPositionInRef[1], DQCurrentPositionInRef[2], DQCurrentPositionInRef[3] };
	toQuaternion(targetPosition[3] * (M_PI / 180.0), targetPosition[4] * (M_PI / 180.0), targetPosition[5] * (M_PI / 180.0), QuatRPYTargetPosition);
	for (int i = 0; i < 4; i++)
		DQTargetPositionInRef[i] = QuatRPYTargetPosition[i];
	//double tmpVals[8] = { 0, 0, 0, 0, 0, 0, targetPosition[6], 1 }; //mrr
	//double tolerance = .00001;
	//if (BeckhoffContext::abs(targetPosition[6] - (-1)) > tolerance)
	//	tmpVals[6] = targetPosition[6];//mrr
	//if (BeckhoffContext::abs(targetPosition[7] - (-1)) > tolerance)//mrr
	//	tmpVals[7] = targetPosition[7];//mrr
	double distance = sqrt_(pow_(DQTargetPositionInRef[5] - DQCurrentPositionInRef[5], 2) + pow_(DQTargetPositionInRef[6] - DQCurrentPositionInRef[6], 2) + pow_(DQTargetPositionInRef[7] - DQCurrentPositionInRef[7], 2));
	TrajectoryPointList<double> pointList;
	pointList = SingleAxisTraj(TrajectoryPoint(0, 0), TrajectoryPoint(distance, 0), targetPosition[6], 100, 250, .001, .999); //a:5000, j:10000 tmpVals[6]
	//if (targetPosition[7] != 0) // movement is continues
	//{
	//	if (is_first)
	//	{
	//		pointList = SingleAxisTraj(TrajectoryPoint(0, 0), TrajectoryPoint(distance, targetPosition[6]), targetPosition[6], 100, 250, .001, .999); //a:5000, j:10000 tmpVals[6]
	//	}
	//	else if (is_end)
	//	{
	//		pointList = SingleAxisTraj(TrajectoryPoint(0, targetPosition[6]), TrajectoryPoint(distance, 0), targetPosition[6], 100, 250, .001, .999); //a:5000, j:10000 tmpVals[6]
	//	}
	//	else
	//	{
	//		pointList = SingleAxisTraj(TrajectoryPoint(0, targetPosition[6]), TrajectoryPoint(distance, targetPosition[6]), targetPosition[6], 100, 250, .001, .999); //a:5000, j:10000 tmpVals[6]
	//	}
	//}
	//else
	//{
	//	pointList = SingleAxisTraj(TrajectoryPoint(0, 0), TrajectoryPoint(distance, 0), targetPosition[6], 100, 250, .001, .999); //a:5000, j:10000 tmpVals[6]
	//}
	//double tmpTeta[6];
	Quaternion Qend = Quaternion(QuatRPYTargetPosition);
	Quaternion QCurrent = Quaternion(QuatRPYCurrentPosition);
	Quaternion Qnext;
	slerp s;
	for (int i = 0; i < pointList.TrajLength; i++)
	{
		//XYZ
		double x = DQCurrentPositionInRef[5] + (pointList.q[i] / distance) * (DQTargetPositionInRef[5] - DQCurrentPositionInRef[5]);
		double y = DQCurrentPositionInRef[6] + (pointList.q[i] / distance) * (DQTargetPositionInRef[6] - DQCurrentPositionInRef[6]);
		double z = DQCurrentPositionInRef[7] + (pointList.q[i] / distance) * (DQTargetPositionInRef[7] - DQCurrentPositionInRef[7]);
		//Quat ABC
		s.Slerp1(QCurrent, Qend, Qnext, pointList.q[i] / distance);
		double res[6];
		double DQPath[] = {Qnext.u.x, Qnext.u.y, Qnext.u.z, Qnext.w, 0, x, y, z };
		for (int j = 0; j < 8; j++)
		{
			resultList[j].AddPoint(DQPath[j], 0, 0);
		}
		//if (i == 0)
		//{
		//	Inversekinematic(DQPath, QbaseGlobal, toolParams, actualPosition, res);//, res);

		//}
		//else
		//{
		//	
		//	double PrePosition[6];

		//	for (int j = 0; j < 6; j++)
		//	{
		//		PrePosition[j] = resultList[j].q[i - 1] * (M_PI / 180.0);

		//	}
		//	if (i == 5813)
		//		int rr = 0;
		//	Inversekinematic(DQPath, QbaseGlobal, toolParams, PrePosition, res);

		//}
		//for (int j = 0; j < 6; j++)
		//{
		//	resultList[j].AddPoint(res[j] * (180.0 / M_PI), 0, 0);
		//	
		//}
	}
}
void Traj7Seg::toEulerianAngle(double quar[], double output[])
{

	double quar0 = quar[0];
	double quar1 = quar[1];
	double quar2 = quar[2];
	double quar3 = quar[3];
	//quar = quar.Normalize(2).ToArray();
	//double output[3];// = new decimal[3];
	double ysqr = quar2 * quar2;

	// roll (x-axis rotation)
	double t0 = +2.0 * (quar0 * quar1 + quar2 * quar3);
	double t1 = +1.0 - 2.0 * (quar1 * quar1 + ysqr);
	output[0] = (atan2_(t0, t1) * 180) / (double)(M_PI);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (quar0 * quar2 - quar3 * quar1);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	output[1] = (double)((asin_((double)t2) * 180) / M_PI);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (quar0 * quar3 + quar1 * quar2);
	double t4 = +1.0 - 2.0 * (ysqr + quar3 * quar3);
	output[2] = (atan2_(t3, t4) * 180) / (double)(M_PI);

	//return output;
}
void Traj7Seg::toQuaternion(double roll, double pitch, double yaw, double q[])
{
	//double q[4];// = new decimal[4];
	double t0 = cos_((double)yaw * 0.5);
	double t1 = sin_((double)yaw * 0.5);
	double t2 = cos_((double)roll * 0.5);
	double t3 = sin_((double)roll * 0.5);
	double t4 = cos_((double)pitch * 0.5);
	double t5 = sin_((double)pitch * 0.5);

	q[0] = (double)(t0 * t2 * t4 + t1 * t3 * t5);
	q[1] = (double)(t0 * t3 * t4 - t1 * t2 * t5);
	q[2] = (double)(t0 * t2 * t5 + t1 * t3 * t4);
	q[3] = (double)(t1 * t2 * t4 - t0 * t3 * t5);
	//return q;
}
std::vector<double> Slerp(double v0[], double v1[], double t)
{
	// Compute the cosine of the angle between the two vectors.
	std::vector<double> Q0(v0, v0 + sizeof v0 / sizeof v0[0]);
	std::vector<double> Q1(v1, v1 + sizeof v1 / sizeof v1[0]);
	return Q0;
	//Q0 = Q0.Normalize(2).ToArray();
	//Q1 = Q1.Normalize(2).ToArray();
	//double dotP = std::inner_product();// Q0.DotProduct(Q1);

	//const double DOT_THRESHOLD = 0.9995;
	//if (BeckhoffContext::abs(dotP) > DOT_THRESHOLD)
	//{
	//	// If the inputs are too close for comfort, linearly interpolate
	//	// and normalize the result.

	//	std::vector<double> result = Q0.Add(t * (Q1.Subtract(Q0))).ToArray();
	//	result = result.Normalize(2).ToArray();
	//	return result;
	//}

	//// If the dot product is negative, the quaternions
	//// have opposite handed-ness and slerp won't take
	//// the shorter path. Fix by reversing one quaternion.
	//if (dotP < 0.0f)
	//{
	//	Q1 = -Q1;
	//	dotP = -dotP;
	//}
	//if (dotP < -1)
	//	dotP = -1;
	//else if (dotP > 1)
	//	dotP = 1;
	//double theta_0 = Math.Acos(dotP);  // theta_0 = angle between input vectors
	//double theta = theta_0 * t;    // theta = angle between v0 and result 

	//DenseVector Q2 = Q1.Subtract((-Q0).Multiply(dotP)).ToArray();
	//Q2 = Q2.Normalize(2).ToArray();              // { v0, v2 } is now an orthonormal basis

	//return Q0.Multiply(Math.Cos(theta)).Add(Q2.Multiply(Math.Sin(theta))).Normalize(2).ToArray();
}
void Traj7Seg::CIRC(double zero[], double one[], double second[], TrajectoryPointList<double> resultList[])//double zero[], double v0, double one[], double v1, double second[], double vmax, double amax, double jmax, double Ta, TrajectoryPointList<double> resultList[])
{
	//zero: XYZYPR source
	//one: XYZYPR dest
	double v0 = 0, v1 = 0, vmax = one[6], amax = 100, jmax = 250, Ta = second[3];
	double DQCurrentPosition[8];
	GetCartPos(zero, toolParamGlobal, DQCurrentPosition);
	double DQCurrentPositionInRef[8];
	DQmultiply(QbaseGlobal, DQCurrentPosition, DQCurrentPositionInRef);

	double A[3];
	double B[3]; 
	double C[3];
	A[0] = DQCurrentPositionInRef[5];
	A[1] = DQCurrentPositionInRef[6];
	A[2] = DQCurrentPositionInRef[7];
	B[0] = one[0];
	B[1] = one[1];
	B[2] = one[2];
	C[0] = second[0];
	C[1] = second[1];
	C[2] = second[2];
	double u1[3];
	sub(B, A, u1, 3);
	double tmp[3];
	double w1[3];
	sub(C, A, tmp, 3);
	// 	w1 = cross(u1,C-A);
	cross(u1, tmp, w1, 3);
	// u  = u1/norm(u1);
	double u[3];
	double uNorm = normA(u1, 3);
	for (int i = 0; i<3; i++)
		u[i] = u1[i] / uNorm;
	// w  = w1/norm(w1);
	double w[3];
	double wNorm = normA(w1, 3);
	for (int i = 0; i<3; i++)
		w[i] = w1[i] / wNorm;
	// v  = cross(w,u);
	double v[3];
	cross(w, u, v, 3);
	// bx = dot(B-A,u);
	double bx = dot(u1, u, 3);
	// cx = dot(C-A,u);
	double cx = dot(tmp, u, 3);
	// cy = dot(C-A,v);
	double cy = dot(tmp, v, 3);
	// h  = ((cx-bx/2)^2 + cy^2 - (bx/2)^2) / (2*cy);
	double h;
	h = ((cx - bx / 2)*(cx - bx / 2) + cy*cy - (bx / 2)*(bx / 2)) / (2 * cy);
	// A2d= [0 0];
	double A2d[2] = { 0,0 };

	// B2d= [bx 0];
	double B2d[2] = { bx, 0 };

	// C2d= [cx cy];
	double C2d[2] = { cx,cy };


	// Cen2d= [bx/2 h]; %center of circle in 2d
	double Cen2d[2] = { bx / 2, h };

	// R  = norm(Cen2d); %radius of circle
	double R = normA(Cen2d, 2);
	double D0, D1, T0;
	// if nargin==20
	if (Ta == -1) {
		//     Ta = acos(dot((A2d-Cen2d),(B2d-Cen2d))/(norm(A2d-Cen2d)*norm(B2d-Cen2d)));
		double x[2]; double y[2];
		sub(A2d, Cen2d, x, 2);
		sub(B2d, Cen2d, y, 2);
		Ta = acos_(dot(x, y, 2) / (normA(x, 2)*normA(y, 2)));
		//     if h > 0
		if (h>0)
			Ta = 2 * M_PI - Ta;
		//         Ta = 2*pi - Ta;
		//     end
		//     D0 = (pi+Ta)/2;
		D0 = (M_PI + Ta) / 2;
		//     D1 = (pi-Ta)/2;
		D1 = (M_PI - Ta) / 2;

		// else
	}
	else {
		//     T0 = acos(dot((A2d-Cen2d),(B2d-Cen2d))/(norm(A2d-Cen2d)*norm(B2d-Cen2d)));
		double x[2]; double y[2];
		sub(A2d, Cen2d, x, 2);
		sub(B2d, Cen2d, y, 2);
		T0 = acos_(dot(x, y, 2) / (normA(x, 2)*normA(y, 2)));
		//     if h > 0
		//         T0 = 2*pi - T0;
		//     end
		if (h>0)
			T0 = 2 * M_PI - T0;
		//     D0 = (pi+Ta)/2 - (Ta-T0)/2 ;
		D0 = (M_PI + Ta) / 2 - (Ta - T0) / 2;
		//     D1 = (pi-Ta)/2 - (Ta-T0)/2 ;
		D1 = (M_PI - Ta) / 2 - (Ta - T0) / 2;
		// end
	}
	v0 = -1 * v0 / R;
	v1 = -1 * v1 / R;
	vmax /= R;
	amax /= R;
	jmax /= R;
	// [time,L]=Single_Axis_7Segment_Trajectory(D0,D1,v0,v1,vmax,amax,jmax);
	TrajectoryPointList<double> pointList;
	pointList = SingleAxisTraj(TrajectoryPoint(D0, 0), TrajectoryPoint(D1, 0), vmax, amax, jmax, .001, .999); //a:5000, j:10000 tmpVals[6]
	double  x, y, z, ratio;
	slerp s;
	Quaternion Qtmp;
	//cout << pointList.TrajLength << endl;
	for (int i = 0; i < pointList.TrajLength; i++) {
		//XYZ
		// X = (X0 + Cen2d(1,1)*u(1,1) + Cen2d(1,2)*v(1,1))+(R*cos(L)*u(1,1)+ R*sin(L)*v(1,1));
		x=(DQCurrentPositionInRef[5] + Cen2d[0] * u[0] + Cen2d[1] * v[0]) + R*cos_(pointList.q[i])*u[0] + R* sin_(pointList.q[i])*v[0];
		// Y = (Y0 + Cen2d(1,1)*u(1,2) + Cen2d(1,2)*v(1,2))+(R*cos(L)*u(1,2)+ R*sin(L)*v(1,2));
		y=(DQCurrentPositionInRef[6] + Cen2d[0] * u[1] + Cen2d[1] * v[1]) + R*cos_(pointList.q[i])*u[1] + R*sin_(pointList.q[i])*v[1];
		// Z = (Z0 + Cen2d(1,1)*u(1,3) + Cen2d(1,2)*v(1,3))+(R*cos(L)*u(1,3)+ R*sin(L)*v(1,3));
		z= DQCurrentPositionInRef[7] + Cen2d[0] * u[2] + Cen2d[1] * v[2] + R*cos_(pointList.q[i])*u[2] + R*sin_(pointList.q[i])*v[2];
		// ratio=(L-D0)/(D1-D0);
		ratio=(pointList.q[i] - D0) / (D1 - D0);


		//RPY
		double Q0[4] = { DQCurrentPositionInRef[0], DQCurrentPositionInRef[1], DQCurrentPositionInRef[2], DQCurrentPositionInRef[3] };
		//toQuaternion(zero[3]*(M_PI/180.0), zero[4] * (M_PI / 180.0), zero[5] * (M_PI / 180.0), Q0);

		double Q1[4];
		toQuaternion(one[3] * (M_PI / 180.0), one[4] * (M_PI / 180.0), one[5] * (M_PI / 180.0), Q1);

		s.Slerp1(Q0, Q1, Qtmp, ratio);

		double res[6];
		double DQPath[] = { Qtmp.u.x, Qtmp.u.y, Qtmp.u.z, Qtmp.w, 0, x, y, z };
		if (i == 0)
		{
			Inversekinematic(DQPath, QbaseGlobal, toolParamGlobal, zero , res);//, res);
		}
		else
		{
			double PrePosition[6];
			for (int j = 0; j < 6; j++)
			{
				PrePosition[j] = resultList[j].q[i - 1] * (M_PI / 180.0);
			}
			Inversekinematic(DQPath, QbaseGlobal, toolParamGlobal, PrePosition, res);
		}
		for (int j = 0; j < 6; j++)
		{
			resultList[j].AddPoint(res[j] * (180.0 / M_PI), 0, 0);
		}
	}
}
void Traj7Seg::sub(double a[], double b[], double out[], int len) {
	for (int i = 0; i<len; i++)
		out[i] = a[i] - b[i];
}
void Traj7Seg::cross(double a[], double b[], double out[], int len) {
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];

}
double Traj7Seg::dot(double a[], double b[], int len) {
	double s = 0;
	for (int i = 0; i<len; i++)
		s += a[i] * b[i];
	return s;
}
double Traj7Seg::normA(double a[], int len) {
	double s = 0;
	for (int i = 0; i<len; i++)
		s += (a[i] * a[i]);
	// cout<<"s: "<<s<<endl;
	return sqrt_(s);
	// cout<<"out: "<<*out<<endl;
}
void Traj7Seg::Approximation(TrajectoryPointList<double> da1[], TrajectoryPointList<double> da2[], double radius, TrajectoryPointList<double> out[], int &IndPre, int &IndNext)
{
	double ltemp = 0;
	IndPre = da1[0].TrajLength - 1;//ld2 - 1;
	IndNext = 0;
	while (ltemp <= radius/* && n != da2[0].TrajLength - 1*/)
	{
		ltemp = ltemp + sqrt_(pow_((da1[5].q[IndPre - 1] - da1[5].q[IndPre]), 2) + pow_((da1[6].q[IndPre - 1] - da1[6].q[IndPre]), 2) + pow_((da1[7].q[IndPre - 1] - da1[7].q[IndPre]), 2));
		IndPre = IndPre - 1;
		IndNext = IndNext + 1; // number of beginnig data of the second traj
	}

	// Data in the approximation distance
	double *X1p = new double[IndNext];
	double *Y1p = new double[IndNext];
	double *Z1p = new double[IndNext];
	double *X2p = new double[IndNext];
	double *Y2p = new double[IndNext];
	double *Z2p = new double[IndNext];
	int index = 0;
	for (int i = IndPre; i < da1[0].TrajLength; i++)
	{
		X1p[index] = da1[5].q[i];
		Y1p[index] = da1[6].q[i];
		Z1p[index] = da1[7].q[i];
		index++;
	}
	for (int i = 0; i < IndNext; i++)
	{
		X2p[i] = da2[5].q[i];
		Y2p[i] = da2[6].q[i];
		Z2p[i] = da2[7].q[i];
	}

	// Calculates length of the trajectory and ratios
	double L1 = sqrt_(pow_((X1p[IndNext - 1] - X1p[0]), 2) + pow_((Y1p[IndNext - 1] - Y1p[0]), 2) + pow_((Z1p[IndNext - 1] - Z1p[0]), 2));
	double *ratiol = new double[IndNext];
	for (int i = 0; i < IndNext; i++)
	{
		double L = sqrt_(pow_((X1p[i] - X1p[0]), 2) + pow_((Y1p[i] - Y1p[0]), 2) + pow_((Z1p[i] - Z1p[0]), 2));
		ratiol[i] = L / L1;
	}

	// Connect lines and derive the points
	double **P = new double*[IndNext]; // row  = n
	for (int i = 0; i < IndNext; i++)
	{
		P[i] = new double[3]; // column = 3
	}

	double current[4] = {
		da1[0].q[IndPre],
		da1[1].q[IndPre],
		da1[2].q[IndPre],
		da1[3].q[IndPre] };
	Quaternion QCurrent = Quaternion(current);
	double end[4] = {
		da2[0].q[IndNext - 1],
		da2[1].q[IndNext - 1],
		da2[2].q[IndNext - 1],
		da2[3].q[IndNext - 1] };
	Quaternion Qend = Quaternion(end);
	Quaternion Qnext;
	slerp s;
	for (int i = 0; i < IndNext; i++)
	{
		double L = sqrt_(pow_((X2p[i] - X1p[i]), 2) + pow_((Y2p[i] - Y1p[i]), 2) + pow_((Z2p[i] - Z1p[i]), 2));
		double unit_vec[3] = { (X2p[i] - X1p[i]) / L, (Y2p[i] - Y1p[i]) / L, (Z2p[i] - Z1p[i]) / L };
		P[i][0] = (unit_vec[0] * ratiol[i] * L) + X1p[i];
		P[i][1] = (unit_vec[1] * ratiol[i] * L) + Y1p[i];
		P[i][2] = (unit_vec[2] * ratiol[i] * L) + Z1p[i];

		// Rotation approximation
		/*s.Slerp1(QCurrent, Qend, Qnext, ratiol[i]);
		out[0].AddPoint(Qnext.u.x, 0, 0);
		out[1].AddPoint(Qnext.u.y, 0, 0);
		out[2].AddPoint(Qnext.u.z, 0, 0);
		out[3].AddPoint(Qnext.w, 0, 0);
		out[4].AddPoint(0, 0, 0);*/
	}
	for (int i = IndPre; i < da1[0].TrajLength; i+=2)
	{
		out[0].AddPoint(da1[0].q[i], 0, 0);
		out[1].AddPoint(da1[1].q[i], 0, 0);
		out[2].AddPoint(da1[2].q[i], 0, 0);
		out[3].AddPoint(da1[3].q[i], 0, 0);
		out[4].AddPoint(0, 0, 0); 
	}
	for (int i = 0; i < IndNext; i += 2)
	{
		out[0].AddPoint(da2[0].q[i], 0, 0);
		out[1].AddPoint(da2[1].q[i], 0, 0);
		out[2].AddPoint(da2[2].q[i], 0, 0);
		out[3].AddPoint(da2[3].q[i], 0, 0);
		out[4].AddPoint(0, 0, 0);
	}
	while (out[0].TrajLength < IndNext)
	{
		out[0].AddPoint(da2[0].q[IndNext - 1], 0, 0);
		out[1].AddPoint(da2[1].q[IndNext - 1], 0, 0);
		out[2].AddPoint(da2[2].q[IndNext - 1], 0, 0);
		out[3].AddPoint(da2[3].q[IndNext - 1], 0, 0);
	}
	//Q = SLERP(Q1(end - n + 1, :), Q2(n, :), ratiol);

	for (int i = 0; i < IndNext; i++)
	{
		out[5].AddPoint(P[i][0], 0, 0);
		out[6].AddPoint(P[i][1], 0, 0);
		out[7].AddPoint(P[i][2], 0, 0);
	}
}