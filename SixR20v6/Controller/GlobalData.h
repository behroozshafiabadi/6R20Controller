#pragma once
//#include <TcMath.h>
//#include <RtlR0.h>	//abs
//#include <iostream.h>
////#include <concurrent_queue.h>
//#include <queue>
//using namespace std;
#include "list"
class GlobalClass
{
public:		
	int GUI_Manager;
	double GUI_TargetPosition[8];
	long  ActualPosition[6];
	int GUI_GetNextCMD=1;
	std::list<long> m0;
	std::list<long> m1;
	std::list<long> m2;
	std::list<long> m3;
	std::list<long> m4;
	std::list<long> m5;
	//int m_value;
	GlobalClass()
	{
		//m_value = v;
	}
	long get_dataPoint(int idx);
	void set_dataPoint(int idx, long data);
	void set_GUI_GetNextCMD(int data);
	int get_GUI_GetNextCMD();
	//int get_value()
	//{
	//	return m_value;
	//}
	//void set_value(int v)
	//{
	//	m_value = v;
	//}
};