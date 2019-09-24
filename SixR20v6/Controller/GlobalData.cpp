#include "TcPch.h"
#pragma hdrstop
#include "GlobalData.h"
///using namespace std;
long GlobalClass::get_dataPoint(int idx) {
	long result = 0;
	switch (idx)
	{
	case 0:
		result = m0.front();
		m0.pop_front();
		break;
	case 1:
		result = m1.front();
		m1.pop_front();
		break;
	case 2:
		result = m2.front();
		m2.pop_front();
		break;
	case 3:
		result = m3.front();
		m3.pop_front();
		break;
	case 4:
		result = m4.front();
		m4.pop_front();
		break;
	case 5:
		result = m5.front();
		m5.pop_front();
		break;
	default:
		break;
	}
	return result;
}

void  GlobalClass::set_dataPoint(int idx, long data) {
	switch (idx)
	{
	case 0:		
		m0.push_back(data);
		break;
	case 1:
		m1.push_back(data);
		break;
	case 2:
		m2.push_back(data);
		break;
	case 3:
		m3.push_back(data);
		break;
	case 4:
		m4.push_back(data);
		break;
	case 5:
		m5.push_back(data);
		break;
	default:
		break;
	}
}

void GlobalClass::set_GUI_GetNextCMD(int data)
{

}

int GlobalClass::get_GUI_GetNextCMD()
{
	return 0;
}
